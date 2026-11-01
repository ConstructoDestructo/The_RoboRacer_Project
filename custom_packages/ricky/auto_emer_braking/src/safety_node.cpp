// safety_node.cpp
// Automatic Emergency Braking (AEB) for the RoboRacer / F1TENTH car.
//
// HOW IT WORKS: the "ghost car"
// ------------------------------
// Every scan, imagine a see-through copy of the car rolling ahead along the
// path the real car is on RIGHT NOW: same speed, same turn rate (both from
// odometry). Step the ghost forward ~2 cm at a time and ask:
//      "does any LiDAR point end up inside the ghost's footprint?"
// The first moment that happens is the time-to-collision (TTC).
// TTC below ttc_threshold -> brake.
//
// Why this handles the wall cases:
//   - Driving parallel to a wall: the ghost slides past it, never touches it.
//   - Nosing into a wall: the ghost's path crosses the wall, so it's caught.
//   - Turning into a wall you're hugging: the ghost CURVES with the turn, so
//     the brake sees it coming instead of reacting late.
//   - The whole body is checked, including the part behind the LiDAR.
//
// Footprint = car body + safety margins:
//   - Front margin: fixed (a few cm of buffer for braking delay).
//   - Side margin: starts at ZERO and widens the farther the ghost travels,
//     like a flashlight beam spreading out. Near-term the prediction is tight;
//     far ahead it allows for small heading errors. This lets you drive a few
//     cm off a parallel wall without braking, while still catching drift.
//
// Brake state machine:
//   MONITORING --TTC too small--> BRAKING --speed ~ 0--> STOPPED
//       ^                                                   |
//       +---------------- held for hold_time ---------------+
//
// The status line also shows the right-wall angle (a line fitted through many
// right-side points, smoothed over time). It's for your eyes / tuning; the
// ghost car already accounts for heading and turning, so braking doesn't use it.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

// ==== CORE BEGIN (plain C++, no ROS: geometry only) ===========================
namespace aeb
{
constexpr double kPi  = 3.14159265358979323846;
constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

inline double deg2rad(double deg) { return deg * kPi / 180.0; }
inline double rad2deg(double rad) { return rad * 180.0 / kPi; }

// Axis-aligned box in the car's own frame (origin = LiDAR, x forward, y left)
struct Box
{
  double x0, x1, y0, y1;
  bool contains(double x, double y) const
  {
    return x >= x0 && x <= x1 && y >= y0 && y <= y1;
  }
  // How far a point is outside the box (0 if inside)
  double distance(double x, double y) const
  {
    const double dx = std::max({x0 - x, 0.0, x - x1});
    const double dy = std::max({y0 - y, 0.0, y - y1});
    return std::hypot(dx, dy);
  }
};

// Car shape, all measured from the LiDAR
struct CarGeometry
{
  double width;         // m
  double to_front;      // LiDAR -> front bumper (m, forward)
  double to_rear;       // LiDAR -> rear bumper  (m, backward)
  double to_pivot;      // LiDAR -> point the car turns around (base_link), backward
  double side_margin;   // max extra room on each side (m)
  double front_margin;  // extra room in front (m)
  double spread;        // how fast the side margin widens: m of margin per m traveled
};

// The bits of a LaserScan we need
struct ScanView
{
  const std::vector<float> &ranges;
  double angle_min, angle_inc, range_min, range_max;
  bool valid(double r) const { return std::isfinite(r) && r >= range_min && r <= range_max; }
  double angle(std::size_t i) const { return angle_min + static_cast<double>(i) * angle_inc; }
};

// One ghost-car snapshot: where the LiDAR is, which way the car faces,
// and how wide the side margin has grown by then
struct GhostStep { double t, lx, ly, c, s, half_width; };

struct Prediction
{
  double ttc = kInf;      // s until the ghost first touches something
  double closest = kInf;  // m, closest any point gets to the real body along the path
};

// Roll the ghost forward and find the first contact.
//   v = forward speed (m/s, > 0), w = turn rate (rad/s, + = turning left)
Prediction predict_ghost(const ScanView &scan, const CarGeometry &g,
                         double v, double w, double horizon, double step_distance,
                         std::vector<GhostStep> &steps)
{
  Prediction out;

  // ---- 1. Where will the ghost be at each little time step? ----
  // Constant speed + constant turn rate = driving along a circle (or a straight
  // line when w ~ 0). The pivot starts step 'to_pivot' behind the LiDAR.
  const double dt = std::min(0.02, step_distance / v);
  const int n = static_cast<int>(std::ceil(horizon / dt));
  const double hw = g.width / 2.0;
  steps.clear();
  steps.reserve(static_cast<std::size_t>(n));
  for (int k = 1; k <= n; ++k) {
    const double t = k * dt;
    const double th = w * t;  // how much the car has turned by time t
    double dx, dy;            // how far the pivot has moved
    if (std::abs(w) < 1e-3) {
      dx = v * t;
      dy = 0.0;
    } else {
      dx = (v / w) * std::sin(th);
      dy = (v / w) * (1.0 - std::cos(th));
    }
    const double c = std::cos(th), s = std::sin(th);
    // Side margin widens with distance traveled (flashlight-beam spread)
    const double margin = std::min(g.side_margin, g.spread * v * t);
    // LiDAR position = pivot position + LiDAR offset rotated with the car
    steps.push_back({t, -g.to_pivot + dx + g.to_pivot * c, dy + g.to_pivot * s, c, s, hw + margin});
  }

  // ---- 2. The real body, in the car's own frame ----
  const Box body {-g.to_rear, g.to_front, -hw, hw};

  // Anything farther than this can't be reached within the horizon
  const double reach = v * horizon +
    std::hypot(std::max(g.to_rear, g.to_front + g.front_margin), hw + g.side_margin);

  // ---- 3. Check every LiDAR point against every ghost snapshot ----
  for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
    const double r = scan.ranges[i];
    if (!scan.valid(r) || r > reach) {
      continue;
    }
    const double a = scan.angle(i);
    const double px = r * std::cos(a);  // ahead
    const double py = r * std::sin(a);  // left (+) / right (-)

    // Inside our own body right now? That's the car seeing itself or noise.
    if (body.contains(px, py)) {
      continue;
    }

    for (const GhostStep &st : steps) {
      if (st.t >= out.ttc) {
        break;  // can't beat the earliest contact already found
      }
      // Where is this point relative to the ghost at time t?
      const double rx = px - st.lx, ry = py - st.ly;
      const double bx =  st.c * rx + st.s * ry;
      const double by = -st.s * rx + st.c * ry;
      out.closest = std::min(out.closest, body.distance(bx, by));
      // Footprint at this step: body + front margin + side margin grown so far
      const Box test {-g.to_rear, g.to_front + g.front_margin, -st.half_width, st.half_width};
      if (test.contains(bx, by)) {
        out.ttc = st.t;
        break;
      }
    }
  }
  return out;
}

// Angle of the right wall relative to our heading, from a straight line fitted
// through right-side points ("a long ruler instead of two tape measures").
//   + = nose pointing INTO the right wall, - = pulling away, NaN = no wall seen
double fit_right_wall_angle(const ScanView &scan, double max_range)
{
  double sx = 0, sy = 0, sxx = 0, syy = 0, sxy = 0;
  int n = 0;
  for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
    const double r = scan.ranges[i];
    const double a = scan.angle(i);
    if (!scan.valid(r) || r > max_range) continue;
    if (a < deg2rad(-120.0) || a > deg2rad(-30.0)) continue;  // right side only
    const double x = r * std::cos(a), y = r * std::sin(a);
    sx += x; sy += y; sxx += x * x; syy += y * y; sxy += x * y;
    ++n;
  }
  if (n < 10) return kNaN;
  const double mx = sx / n, my = sy / n;
  const double cxx = sxx / n - mx * mx;
  const double cyy = syy / n - my * my;
  const double cxy = sxy / n - mx * my;
  // Direction the points spread out along = direction of the wall
  return 0.5 * std::atan2(2.0 * cxy, cxx - cyy);
}
}  // namespace aeb
// ==== CORE END ================================================================

enum class BrakeState { MONITORING, BRAKING, STOPPED };

class SafetyNode : public rclcpp::Node
{
public:
  SafetyNode() : Node("auto_emergency_brake")
  {
    // ---- Which car are we driving? -----------------------------------------
    //   "sim"  (default): f1tenth_gym car, odom on /ego_racecar/odom
    //   "real"          : our car, odom on /odom
    // Switch with: --ros-args -p platform:=real
    platform_ = declare_parameter<std::string>("platform", "sim");
    const bool is_real = (platform_ == "real");
    const std::string odom_topic = is_real ? "/odom" : "/ego_racecar/odom";

    // ---- Car shape, measured from the LiDAR ---------------------------------
    // Sim: 0.31 x 0.58 m body (f1tenth_gym default), assumed centered on
    //      base_link; LiDAR 0.275 m ahead of base_link (sim.yaml).
    // Real: 0.179 x 0.31 m body. LiDAR position below is a GUESS - measure
    //      LiDAR center -> front bumper, -> rear bumper, -> rear axle.
    geom_.width        = declare_parameter<double>("car_width",      is_real ? 0.179 : 0.31);
    geom_.to_front     = declare_parameter<double>("lidar_to_front", is_real ? 0.05  : 0.015);
    geom_.to_rear      = declare_parameter<double>("lidar_to_rear",  is_real ? 0.26  : 0.565);
    geom_.to_pivot     = declare_parameter<double>("lidar_to_pivot", is_real ? 0.20  : 0.275);
    geom_.side_margin  = declare_parameter<double>("lane_margin", 0.06);   // max side margin, m
    geom_.front_margin = declare_parameter<double>("front_margin", 0.05);  // m
    // Side margin grows by this many m per m traveled. 0.035 ~ allowing a 2 deg
    // heading error: hugging a wall 5 cm off brakes only above ~3.5 m/s.
    geom_.spread       = declare_parameter<double>("margin_spread", 0.035);

    // ---- Braking behavior ----------------------------------------------------
    ttc_threshold_   = declare_parameter<double>("ttc_threshold", 0.4);   // s
    horizon_         = declare_parameter<double>("horizon", 1.5);         // s, how far ahead the ghost drives
    step_distance_   = declare_parameter<double>("step_distance", 0.02);  // m per ghost step
    moving_speed_    = declare_parameter<double>("moving_speed", 0.01);   // m/s, below = "stopped"
    stop_speed_      = declare_parameter<double>("stop_speed", 0.05);     // m/s, BRAKING -> STOPPED
    hold_time_       = declare_parameter<double>("hold_time", 1.0);       // s
    status_period_ms_ = declare_parameter<int>("status_period_ms", 250);
    wall_fit_range_  = declare_parameter<double>("wall_fit_range", 2.0);  // m
    angle_smoothing_ = declare_parameter<double>("angle_smoothing", 0.3); // 0..1, lower = steadier

    acker_publisher_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);

    odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, 10,
      std::bind(&SafetyNode::odom_callback, this, std::placeholders::_1));

    scan_subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10,
      std::bind(&SafetyNode::scan_callback, this, std::placeholders::_1));

    stopped_since_ = now();  // same clock as later now() calls

    RCLCPP_INFO(get_logger(),
      "AEB ready | platform %s (odom %s) | car %.3f m wide, LiDAR %.3f m from front, "
      "%.3f m from rear, pivot %.3f m behind | margins side up to %.2f (spread %.3f/m) / "
      "front %.2f m | trip TTC %.2f s | hold %.1f s",
      platform_.c_str(), odom_topic.c_str(), geom_.width, geom_.to_front, geom_.to_rear,
      geom_.to_pivot, geom_.side_margin, geom_.spread, geom_.front_margin,
      ttc_threshold_, hold_time_);
  }

private:
  // --------------------------------------------------------------------------
  void odom_callback(const nav_msgs::msg::Odometry &msg)
  {
    // twist is in the car's own frame:
    //   linear.x  = forward speed (+ forward, - reverse)
    //   angular.z = turn rate     (+ turning left, - turning right)
    longitudinal_velocity = msg.twist.twist.linear.x;
    yaw_rate_ = msg.twist.twist.angular.z;
  }

  // --------------------------------------------------------------------------
  void scan_callback(const sensor_msgs::msg::LaserScan &msg)
  {
    const double v = longitudinal_velocity;
    const double w = yaw_rate_;
    const aeb::ScanView scan{msg.ranges, msg.angle_min, msg.angle_increment,
                             msg.range_min, msg.range_max};

    // ---- Step 1: steady right-wall angle (display only) --------------------
    const double raw_angle = aeb::fit_right_wall_angle(scan, wall_fit_range_);
    if (std::isnan(raw_angle)) {
      wall_angle_ = aeb::kNaN;
    } else if (std::isnan(wall_angle_)) {
      wall_angle_ = raw_angle;
    } else {
      wall_angle_ += angle_smoothing_ * (raw_angle - wall_angle_);
    }

    // ---- Step 2: roll the ghost car forward ---------------------------------
    // Only when actually moving forward (the LiDAR can't see straight behind,
    // and a stopped car can't hit anything).
    const bool moving_forward = v > moving_speed_;
    aeb::Prediction pred;
    if (moving_forward) {
      pred = aeb::predict_ghost(scan, geom_, v, w, horizon_, step_distance_, steps_);
    }
    const bool danger = moving_forward && pred.ttc < ttc_threshold_;

    // ---- Step 3: brake state machine ----------------------------------------
    switch (state_) {
      case BrakeState::MONITORING:
        if (danger) {
          state_ = BrakeState::BRAKING;
          RCLCPP_WARN(get_logger(),
            "AEB TRIGGERED  v=%.2f m/s  turn=%+.2f rad/s  ghost TTC=%.2f s  closest pass=%.3f m",
            v, w, pred.ttc, pred.closest);
        }
        break;

      case BrakeState::BRAKING:
        if (std::abs(v) < stop_speed_) {
          state_ = BrakeState::STOPPED;
          stopped_since_ = now();
          RCLCPP_INFO(get_logger(), "AEB: car stopped, holding brake.");
        }
        break;

      case BrakeState::STOPPED: {
        const double held_for = (now() - stopped_since_).seconds();
        if (held_for > hold_time_) {
          // Re-arm. Driving at the obstacle again re-triggers right away;
          // backing up or steering away doesn't.
          state_ = BrakeState::MONITORING;
          RCLCPP_INFO(get_logger(), "AEB: hold done, re-armed.");
        }
        break;
      }
    }

    if (state_ != BrakeState::MONITORING) {
      publish_brake();
    }

    // ---- Step 4: live status line (throttled) --------------------------------
    // Example:
    //   [MONITORING] v= 0.49 m/s turn=-0.12 rad/s | ghost TTC 0.72 s (trips < 0.40) | closest pass 0.048 m | wall angle +1.2 deg
    char ttc_txt[40], pass_txt[24], angle_txt[24];
    if (v < -moving_speed_) {
      std::snprintf(ttc_txt, sizeof(ttc_txt), "reversing (not checked)");
    } else if (!moving_forward) {
      std::snprintf(ttc_txt, sizeof(ttc_txt), "stopped");
    } else if (std::isinf(pred.ttc)) {
      std::snprintf(ttc_txt, sizeof(ttc_txt), "clear for %.1f s", horizon_);
    } else {
      std::snprintf(ttc_txt, sizeof(ttc_txt), "%.2f s", pred.ttc);
    }
    if (moving_forward && std::isfinite(pred.closest)) {
      std::snprintf(pass_txt, sizeof(pass_txt), "%.3f m", pred.closest);
    } else {
      std::snprintf(pass_txt, sizeof(pass_txt), "--");
    }
    if (std::isnan(wall_angle_)) {
      std::snprintf(angle_txt, sizeof(angle_txt), "no wall");
    } else {
      std::snprintf(angle_txt, sizeof(angle_txt), "%+.1f deg", aeb::rad2deg(wall_angle_));
    }

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), status_period_ms_,
      "[%-10s] v=%5.2f m/s turn=%+5.2f rad/s | ghost TTC %s (trips < %.2f) | closest pass %s | wall angle %s",
      state_name(state_), v, w, ttc_txt, ttc_threshold_, pass_txt, angle_txt);
  }

  static const char *state_name(BrakeState s)
  {
    switch (s) {
      case BrakeState::MONITORING: return "MONITORING";
      case BrakeState::BRAKING:    return "BRAKING";
      case BrakeState::STOPPED:    return "STOPPED";
    }
    return "?";
  }

  // --------------------------------------------------------------------------
  void publish_brake()
  {
    ackermann_msgs::msg::AckermannDriveStamped cmd;
    cmd.header.stamp = now();
    cmd.drive.speed = 0.0;  // full stop request
    acker_publisher_->publish(cmd);
  }

  // ---- ROS plumbing -------------------------------------------------------
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr acker_publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;

  // ---- State --------------------------------------------------------------
  double longitudinal_velocity = 0.0;
  double yaw_rate_ = 0.0;
  double wall_angle_ = aeb::kNaN;
  BrakeState state_ = BrakeState::MONITORING;
  rclcpp::Time stopped_since_;
  std::vector<aeb::GhostStep> steps_;  // reused every scan

  // ---- Parameters -----------------------------------------------------------
  std::string platform_ = "sim";
  aeb::CarGeometry geom_{};
  double ttc_threshold_   = 0.0;
  double horizon_         = 0.0;
  double step_distance_   = 0.0;
  double moving_speed_    = 0.0;
  double stop_speed_      = 0.0;
  double hold_time_       = 0.0;
  int    status_period_ms_ = 250;
  double wall_fit_range_  = 0.0;
  double angle_smoothing_ = 0.0;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SafetyNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
