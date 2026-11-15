// reactive_node.cpp
// Lab 4: ghost-car obstacle avoidance, paired with a wall follower.
//
// THE IDEA (borrowed from our emergency-brake node)
// -------------------------------------------------
// Every plan, send out a fan of "ghost cars": see-through copies of the car,
// one per candidate heading (-90..+90 deg). Each ghost drives the path the
// real car would take to reach that heading:
//
//      turn (an arc, at the steering angle the car would really use)
//      ... then straight.
//
// At every step along the path the ghost's WHOLE BODY is checked against the
// LiDAR points, not just a line from the LiDAR. The distance the ghost gets
// before touching something is that heading's "free distance".
//
// Why this is better than checking straight corridors: the car can't jump to
// a new heading, it has to swing through an arc, and its body trails behind
// the LiDAR. A ghost on the real arc knows when a turn won't fit.
//
// Each ghost is checked with two margins around the body:
//   - hard margin    (a few cm): real contact. Used for speed and stopping.
//   - comfort margin (side_margin): how much room we'd LIKE. Used to choose
//     between headings, so the car takes the roomy path when there is one
//     but can still squeeze through a tight gap when there isn't.
//
// THE WALL ADVISOR (same math as Lab 3) suggests a preferred heading:
//   - wall beside us        -> the heading that holds wall_distance
//   - wall reading is junk  -> no opinion (straight). "Junk" = the two beams
//                              disagree (one fell into a recess or onto an
//                              obstacle), which shows up as a large wall angle.
//   - a real hallway opened -> turn into it. "Real" = a ghost can turn into it
//     on the wall side         and drive wall_open_depth with room to spare.
//
// CHOOSING: score = comfort free distance
//                   - (distance from the preferred heading)
//                   - (distance from the current heading)
// The pull toward the preferred heading fades when the preferred way is
// blocked, so the car doesn't keep driving at a wall out of preference.
//
// SPEED: never faster than the car could still stop within the chosen ghost's
// free distance, slower in turns (grip), and already down to turning speed
// by the time something ahead is turn_room away.
//
// BACKING UP: if no ghost can move forward, the car stops, then reverses a
// short way with the wheels turned so its nose swings toward the open side,
// and tries again. The LiDAR cannot see straight behind the car (its view is
// 270 deg), so the back-up is short and slow, over floor the car just drove.
//
// DIRECTION: the node has no built-in direction; the car drives the way it
// is facing when it starts. wall_side picks the wall the advisor tracks; use
// the INNER wall of the loop (clockwise -> "right", counter-clockwise -> "left").
//
// Reads only /scan and publishes only /drive (plus its own marker topic).
// It knows its speed from the speed it last commanded, not from odometry.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <vector>

// ==== CORE BEGIN (plain C++, no ROS: all of the planning) =====================
namespace gap
{
constexpr double kPi = 3.14159265358979323846;
inline double radians(double deg) { return deg * kPi / 180.0; }
inline double degrees(double rad) { return rad * 180.0 / kPi; }

struct Params
{
  // ---- Car shape, measured from the LiDAR (defaults: the simulator's car) ----
  double car_half_width = 0.155;      // m
  double lidar_to_front = 0.015;      // LiDAR -> front bumper (m)
  double lidar_to_rear = 0.565;       // LiDAR -> rear bumper (m)
  double lidar_to_rear_axle = 0.446;  // LiDAR -> rear axle, the point the car turns about (m)
  double wheelbase = 0.33;            // m
  double max_steering = 0.34;         // rad

  // ---- Margins around the ghost's body ----------------------------------------
  double side_margin = 0.20;    // comfort margin on each side (m)
  double hard_margin = 0.05;    // contact margin on each side (m)
  double front_margin = 0.05;   // extra room in front (m)
  double margin_spread = 0.5;   // margins grow from 0 by this many m per m traveled

  // ---- Ghost paths --------------------------------------------------------------
  double ghost_step = 0.10;      // m between ghost snapshots
  double reaction_time = 0.05;   // s the ghost keeps going straight before it turns
  double radius_scale = 1.0;     // > 1 = assume the car turns wider than the geometry says
  double lookahead_time = 1.0;   // a heading must be clear for speed x this ...
  double lookahead_min = 1.5;    // ... but at least this (m) ...
  double lookahead_max = 4.0;    // ... and at most this (m)
  double heading_limit_deg = 90.0;
  double heading_step_deg = 5.0;

  // ---- Scoring (meters of free distance "charged" per radian) -------------------
  double turn_weight = 0.6;      // pull toward the preferred heading
  double change_weight = 0.3;    // reluctance to change heading (keep below turn_weight)
  double change_cap_deg = 30.0;  // ... counted up to this much change
  double heading_filter_tau = 0.05;  // s, smoothing of the chosen heading
  double steer_gain = 1.0;       // steering angle = gain x chosen heading

  // ---- Speed --------------------------------------------------------------------
  double max_speed = 3.0;
  double min_speed = 0.5;
  double brake_decel = 4.0;         // m/s^2 the car can count on when slowing
  double lateral_accel = 5.0;       // m/s^2 allowed sideways in a turn
  double stop_buffer = 0.25;        // m kept in hand when stopping for something
  double emergency_distance = 0.15; // chosen ghost blocked closer than this = stop
  double turn_room = 2.0;           // be at turning speed when this far from a blockage

  // ---- Wall advisor ---------------------------------------------------------------
  double wall_distance = 0.75;
  double wall_kp = 1.0;               // tuned in Lab 3
  double wall_lookahead = 0.5;
  double wall_theta_deg = 50.0;
  double wall_beam_window_deg = 1.0;
  double wall_approach_limit_deg = 6.0;  // gentle toward the wall ...
  double wall_away_limit_deg = 20.0;     // ... firmer away from it
  double wall_max_angle_deg = 15.0;      // beyond this the beams aren't on one wall
  double wall_open_depth = 3.0;          // a ghost must get this far into an opening
  double wall_turn_min_deg = 60.0;       // ... on a heading at least this steep
  double wall_turn_extra_margin = 0.10;  // ... with this much extra side room

  // ---- Backing up -------------------------------------------------------------------
  double reverse_speed = 0.5;     // m/s
  double reverse_distance = 0.5;  // m per back-up

  // ---- Switches / counts (not in the table below) ---------------------------------
  bool follow_wall = true;
  double wall_side = -1.0;   // -1 = right wall, +1 = left wall
  bool back_up = true;
  int beam_skip = 4;         // use every Nth LiDAR beam for the ghost checks
  int blocked_plans = 5;     // plans in a row with no way forward before backing up
};

// Name -> field table for every decimal parameter (used to declare them and
// to update them live).
struct ParamRef { const char * name; double Params::* field; };
inline const std::vector<ParamRef> & doubleParams()
{
  static const std::vector<ParamRef> table = {
    {"car_half_width", &Params::car_half_width},
    {"lidar_to_front", &Params::lidar_to_front},
    {"lidar_to_rear", &Params::lidar_to_rear},
    {"lidar_to_rear_axle", &Params::lidar_to_rear_axle},
    {"wheelbase", &Params::wheelbase},
    {"max_steering", &Params::max_steering},
    {"side_margin", &Params::side_margin},
    {"hard_margin", &Params::hard_margin},
    {"front_margin", &Params::front_margin},
    {"margin_spread", &Params::margin_spread},
    {"ghost_step", &Params::ghost_step},
    {"reaction_time", &Params::reaction_time},
    {"radius_scale", &Params::radius_scale},
    {"lookahead_time", &Params::lookahead_time},
    {"lookahead_min", &Params::lookahead_min},
    {"lookahead_max", &Params::lookahead_max},
    {"heading_limit_deg", &Params::heading_limit_deg},
    {"heading_step_deg", &Params::heading_step_deg},
    {"turn_weight", &Params::turn_weight},
    {"change_weight", &Params::change_weight},
    {"change_cap_deg", &Params::change_cap_deg},
    {"heading_filter_tau", &Params::heading_filter_tau},
    {"steer_gain", &Params::steer_gain},
    {"max_speed", &Params::max_speed},
    {"min_speed", &Params::min_speed},
    {"brake_decel", &Params::brake_decel},
    {"lateral_accel", &Params::lateral_accel},
    {"stop_buffer", &Params::stop_buffer},
    {"emergency_distance", &Params::emergency_distance},
    {"turn_room", &Params::turn_room},
    {"wall_distance", &Params::wall_distance},
    {"wall_kp", &Params::wall_kp},
    {"wall_lookahead", &Params::wall_lookahead},
    {"wall_theta_deg", &Params::wall_theta_deg},
    {"wall_beam_window_deg", &Params::wall_beam_window_deg},
    {"wall_approach_limit_deg", &Params::wall_approach_limit_deg},
    {"wall_away_limit_deg", &Params::wall_away_limit_deg},
    {"wall_max_angle_deg", &Params::wall_max_angle_deg},
    {"wall_open_depth", &Params::wall_open_depth},
    {"wall_turn_min_deg", &Params::wall_turn_min_deg},
    {"wall_turn_extra_margin", &Params::wall_turn_extra_margin},
    {"reverse_speed", &Params::reverse_speed},
    {"reverse_distance", &Params::reverse_distance},
  };
  return table;
}

// The bits of a LaserScan the planner needs
struct ScanView
{
  const std::vector<float> & ranges;
  double angle_min, angle_inc, range_min, range_max;
};

// Where a ghost's rear axle is (in the LiDAR's frame: x forward, y left) and
// which way the ghost faces (cos / sin of its heading)
struct Pose { double x, y, c, s; };

struct Free { double hard, comfort; };

struct Output
{
  double steer = 0.0;
  double speed = 0.0;
  const char * mode = "straight";  // what the advisor is doing
  double preferred = 0.0;          // heading the advisor asked for (rad)
  double heading = 0.0;            // heading chosen (rad)
  double free = 0.0;               // how far the chosen ghost gets (m)
  double wall_distance = 0.0;      // m
  bool reversing = false;
  bool started_reverse = false;
  bool boxed_in = false;           // no way forward and no room to back up
  int ghosts = 0;                  // how many ghost cars were driven this plan
};

class Planner
{
public:
  Params p;

  // One planning cycle. dt = seconds since the previous cycle.
  Output plan(const ScanView & scan, double dt)
  {
    Output out;
    ghost_count_ = 0;
    const double length = std::max({p.lookahead_max, p.wall_open_depth, p.turn_room + 1.0});
    collectPoints(scan, length);

    // ---- Backing up -------------------------------------------------------------
    if (reversing_) {
      reversed_ += p.reverse_speed * dt;
      const double behind = reverseFree(reverse_steer_);
      if (reversed_ >= p.reverse_distance || behind < 0.15) {
        reversing_ = false;
        blocked_ = 0;
        heading_ = 0.0;
        last_speed_ = 0.0;
      } else {
        out.steer = reverse_steer_;
        out.speed = -p.reverse_speed;
        out.mode = "BACKING UP";
        out.reversing = true;
        return out;
      }
    }

    // ---- 1. Send out the fan of ghosts ---------------------------------------------
    const double cap = std::clamp(last_speed_ * p.lookahead_time, p.lookahead_min,
                                  std::max(p.lookahead_min, p.lookahead_max));
    const double step = radians(std::max(1.0, p.heading_step_deg));
    const int n = static_cast<int>(std::lround(radians(std::clamp(p.heading_limit_deg, 0.0, 120.0)) / step));
    headings_.clear();
    hard_.clear();
    comfort_.clear();
    for (int k = -n; k <= n; ++k) {
      const double h = k * step;
      const Free f = ghostFree(h, length, p.side_margin);
      headings_.push_back(h);
      hard_.push_back(f.hard);
      comfort_.push_back(f.comfort);
    }

    // ---- 2. The wall advisor's preferred heading ---------------------------------------
    double preferred = 0.0;
    if (p.follow_wall) {
      bool trusted = false;
      preferred = wallHeading(scan, trusted, out.wall_distance);
      out.mode = trusted ? "wall" : "no wall";

      // A real hallway on the wall side? Send ghosts down the steep headings
      // with extra side room; take the one that gets furthest (ties: least steep).
      const double turn_min = radians(p.wall_turn_min_deg) - 1e-9;
      bool found = false;
      double turn = 0.0, turn_free = 0.0;
      for (std::size_t i = 0; i < headings_.size(); ++i) {
        const double h = headings_[i];
        if (p.wall_side * h < turn_min) continue;
        const double f = ghostFree(h, length, p.side_margin + p.wall_turn_extra_margin).comfort;
        if (f < p.wall_open_depth) continue;
        const bool better = !found || f > turn_free + 1e-9 ||
          (std::abs(f - turn_free) <= 1e-9 && std::abs(h) < std::abs(turn));
        if (better) { found = true; turn = h; turn_free = f; }
      }
      if (found) { preferred = turn; out.mode = "turn-in"; }
    }

    // ---- 3. Score every ghost (plus one on the advisor's exact heading) --------------------
    const double preferred_free = ghostFree(preferred, length, p.side_margin).comfort;
    // How open the preferred way is (1 = fully). The less open, the weaker its pull.
    const double open_ratio = std::min(1.0, std::min(preferred_free, cap) / cap);
    const double pull = p.turn_weight * open_ratio * open_ratio;
    const double change_cap = radians(p.change_cap_deg);
    double best_score = -std::numeric_limits<double>::infinity();
    double best_heading = 0.0;
    const auto consider = [&](double h, double comfort) {
      const double score = std::min(comfort, cap)
        - pull * std::abs(h - preferred)
        - p.change_weight * std::min(std::abs(h - heading_), change_cap);
      if (score > best_score) { best_score = score; best_heading = h; }
    };
    for (std::size_t i = 0; i < headings_.size(); ++i) consider(headings_[i], comfort_[i]);
    consider(preferred, preferred_free);

    // ---- 4. Steer toward the winner (smoothed so it can't jump) ------------------------------
    heading_ += dt / (std::max(0.0, p.heading_filter_tau) + dt) * (best_heading - heading_);
    const double steer = std::clamp(p.steer_gain * heading_, -p.max_steering, p.max_steering);

    // ---- 5. Speed ---------------------------------------------------------------------------------
    // How far the ghost on our actual heading gets before real contact.
    const double free_now = ghostFree(heading_, length, p.side_margin).hard;
    // (a) Could still stop in time: v = sqrt(2 a d).
    const double v_brake =
      std::sqrt(std::max(0.0, 2.0 * p.brake_decel * (free_now - p.stop_buffer)));
    // (b) Grip in the turn we're in: v = sqrt(a_lat * R), R = wheelbase / tan(steer).
    const double curvature = std::tan(std::abs(steer)) / std::max(0.01, p.wheelbase);
    const double v_corner = curvature > 1e-6
      ? std::sqrt(std::max(0.0, p.lateral_accel) / curvature) : p.max_speed;
    // (c) Something ahead: be down to full-lock turning speed turn_room before it.
    const double tightest = p.wheelbase / std::tan(std::max(0.05, p.max_steering));
    const double v_turn2 = std::max(0.0, p.lateral_accel) * tightest;
    const double v_ahead =
      std::sqrt(v_turn2 + 2.0 * p.brake_decel * std::max(0.0, free_now - p.turn_room));
    double speed = std::max(std::min({p.max_speed, v_brake, v_corner, v_ahead}), p.min_speed);

    // ---- 6. No way forward: stop, then back up ---------------------------------------------------------
    if (free_now < p.emergency_distance) {
      speed = 0.0;
      ++blocked_;
      if (blocked_ >= std::max(1, p.blocked_plans)) {
        out.boxed_in = true;
        if (p.back_up) {
          // Which side is most open? (ties go to the wall side)
          std::size_t open = 0;
          double open_value = -std::numeric_limits<double>::infinity();
          for (std::size_t i = 0; i < headings_.size(); ++i) {
            const double value = hard_[i] + 1e-3 * p.wall_side * headings_[i];
            if (value > open_value) { open_value = value; open = i; }
          }
          const double h = headings_[open];
          const double direction = std::abs(h) > 1e-3 ? (h > 0.0 ? 1.0 : -1.0) : p.wall_side;
          // Reverse with the wheels turned the other way so the nose swings
          // toward the open side; if that's blocked, reverse straight.
          for (const double candidate : {-direction * p.max_steering, 0.0}) {
            if (reverseFree(candidate) >= 0.15) {
              reversing_ = true;
              reversed_ = 0.0;
              reverse_steer_ = candidate;
              out.started_reverse = true;
              out.boxed_in = false;
              break;
            }
          }
        }
      }
    } else {
      blocked_ = 0;
    }

    last_speed_ = speed;
    out.steer = steer;
    out.speed = speed;
    out.preferred = preferred;
    out.heading = heading_;
    out.free = free_now;
    out.ghosts = ghost_count_;
    return out;
  }

  // Where the ghost for heading h is after driving `distance` along its path:
  // a short straight (reaction time), an arc at the steering the car would
  // use, then straight along the new heading.
  Pose ghostPose(double h, double distance) const
  {
    const double lead = std::max(0.0, last_speed_) * p.reaction_time;
    if (std::abs(h) < 1e-4) return {-p.lidar_to_rear_axle + distance, 0.0, 1.0, 0.0};
    const double steer = std::min(std::abs(p.steer_gain * h), p.max_steering);
    const double radius = p.radius_scale * p.wheelbase / std::tan(std::max(steer, 1e-6));
    const double arc = radius * std::abs(h);
    const double sign = h > 0.0 ? 1.0 : -1.0;
    const double on_lead = std::min(distance, lead);
    const double on_arc = std::clamp(distance - lead, 0.0, arc);
    const double on_straight = std::max(distance - lead - arc, 0.0);
    const double phi = on_arc / radius;
    return {
      -p.lidar_to_rear_axle + on_lead + radius * std::sin(phi) + on_straight * std::cos(h),
      sign * radius * (1.0 - std::cos(phi)) + on_straight * std::sin(h),
      std::cos(sign * phi), std::sin(sign * phi)};
  }

  double lastSpeed() const { return last_speed_; }

private:
  struct Point { double x, y; };

  // LiDAR beams -> floor points near enough to matter. Points inside our own
  // body are the car seeing itself: dropped.
  void collectPoints(const ScanView & scan, double length)
  {
    points_.clear();
    const std::size_t skip = static_cast<std::size_t>(std::max(1, p.beam_skip));
    for (std::size_t i = 0; i < scan.ranges.size(); i += skip) {
      const double r = scan.ranges[i];
      if (!std::isfinite(r) || r < scan.range_min || r > scan.range_max) continue;
      if (r > length + 1.0) continue;
      const double a = scan.angle_min + static_cast<double>(i) * scan.angle_inc;
      const Point pt{r * std::cos(a), r * std::sin(a)};
      const double bx = pt.x + p.lidar_to_rear_axle;
      const bool own_body = bx >= -rearOverhang() && bx <= frontLength() &&
        std::abs(pt.y) <= p.car_half_width;
      if (!own_body) points_.push_back(pt);
    }
  }

  double rearOverhang() const { return p.lidar_to_rear - p.lidar_to_rear_axle; }   // axle -> rear bumper
  double frontLength() const { return p.lidar_to_rear_axle + p.lidar_to_front; }   // axle -> front bumper

  // Drive one ghost toward heading h and report how far it gets before
  //   .hard    : its body (+ hard margin) touches a point
  //   .comfort : a point comes inside its comfort margin
  // A point that is ALREADY alongside the car (inside the comfort band right
  // now) only counts on real contact: we're passing it, not heading for it.
  Free ghostFree(double h, double length, double comfort_margin) const
  {
    ++ghost_count_;
    const double step = std::max(0.02, p.ghost_step);
    const int steps = static_cast<int>(std::ceil(length / step));
    const double rear = rearOverhang();
    const double front = frontLength() + p.front_margin;
    const double reach = std::hypot(std::max(rear, front), p.car_half_width + comfort_margin) + 1e-6;
    const double reach2 = reach * reach;
    int comfort_hit = 0;  // step of the first comfort-margin contact (0 = none yet)
    for (int k = 1; k <= steps; ++k) {
      const double distance = k * step;
      const Pose g = ghostPose(h, distance);
      const double hard = p.car_half_width + std::min(p.hard_margin, p.margin_spread * distance);
      const double comfort = p.car_half_width + std::min(comfort_margin, p.margin_spread * distance);
      for (const Point & pt : points_) {
        const double rx = pt.x - g.x;
        const double ry = pt.y - g.y;
        if (rx * rx + ry * ry > reach2) continue;
        const double bx = g.c * rx + g.s * ry;        // along the ghost's body
        if (bx < -rear || bx > front) continue;
        const double by = std::abs(-g.s * rx + g.c * ry);  // sideways from its centerline
        if (by <= hard) {
          const double free = (k - 1) * step;
          return {free, comfort_hit > 0 ? (comfort_hit - 1) * step : free};
        }
        if (comfort_hit == 0 && by <= comfort) {
          const double bx0 = pt.x + p.lidar_to_rear_axle;
          const bool alongside = bx0 >= -rear && bx0 <= front &&
            std::abs(pt.y) <= p.car_half_width + comfort_margin;
          if (!alongside) comfort_hit = k;
        }
      }
    }
    return {length, comfort_hit > 0 ? (comfort_hit - 1) * step : length};
  }

  // How far the car can back up with this steering before its body (plus a
  // small margin) reaches a point the LiDAR can see. Reversing with the wheels
  // turned left swings the tail left and the nose right.
  double reverseFree(double steer) const
  {
    const double step = std::max(0.02, p.ghost_step);
    const int steps = static_cast<int>(std::ceil(p.reverse_distance / step));
    const double rear = rearOverhang() + 0.05;
    const double front = frontLength();
    const double half = p.car_half_width + 0.03;
    for (int k = 1; k <= steps; ++k) {
      const double distance = k * step;
      Pose g{-p.lidar_to_rear_axle - distance, 0.0, 1.0, 0.0};
      if (std::abs(steer) >= 1e-4) {
        const double radius = p.wheelbase / std::tan(std::abs(steer));
        const double phi = distance / radius;
        const double sign = steer > 0.0 ? 1.0 : -1.0;
        g = {-p.lidar_to_rear_axle - radius * std::sin(phi), sign * radius * (1.0 - std::cos(phi)),
             std::cos(-sign * phi), std::sin(-sign * phi)};
      }
      for (const Point & pt : points_) {
        const double rx = pt.x - g.x;
        const double ry = pt.y - g.y;
        const double bx = g.c * rx + g.s * ry;
        if (bx < -rear || bx > front) continue;
        if (std::abs(-g.s * rx + g.c * ry) <= half) return (k - 1) * step;
      }
    }
    return p.reverse_distance;
  }

  // Range toward angle_deg (0 = straight ahead, + = left), taken as the
  // median of the beams within +/- wall_beam_window_deg (Lab 3's noise fix).
  double medianRange(const ScanView & scan, double angle_deg) const
  {
    const int n = static_cast<int>(scan.ranges.size());
    const double position = (radians(angle_deg) - scan.angle_min) / scan.angle_inc;
    const int idx = std::clamp(static_cast<int>(std::lround(position)), 0, n - 1);
    const int half = std::max(0, static_cast<int>(
      std::lround(radians(p.wall_beam_window_deg) / scan.angle_inc)));
    std::vector<double> window;
    window.reserve(static_cast<std::size_t>(2 * half + 1));
    for (int i = std::max(0, idx - half); i <= std::min(n - 1, idx + half); ++i) {
      const double r = scan.ranges[static_cast<std::size_t>(i)];
      if (!std::isfinite(r) || r > scan.range_max) window.push_back(scan.range_max);
      else window.push_back(std::max(r, scan.range_min));
    }
    auto middle = window.begin() + static_cast<std::ptrdiff_t>(window.size() / 2);
    std::nth_element(window.begin(), middle, window.end());
    return *middle;
  }

  // The wall follower's opinion: the heading that would hold wall_distance.
  // Same two-beam math as Lab 3: beam b points straight at the wall side,
  // beam a is theta degrees further forward. Returns 0 (no opinion) and
  // trusted = false when the beams are not on one wall.
  double wallHeading(const ScanView & scan, bool & trusted, double & distance) const
  {
    const double theta = radians(std::clamp(p.wall_theta_deg, 10.0, 70.0));
    const double b = medianRange(scan, p.wall_side * 90.0);
    const double a = medianRange(scan, p.wall_side * (90.0 - degrees(theta)));
    const double alpha = std::atan2(a * std::cos(theta) - b, a * std::sin(theta));
    distance = b * std::cos(alpha);
    trusted = std::abs(alpha) <= radians(p.wall_max_angle_deg);
    if (!trusted) return 0.0;

    const double projected = distance + p.wall_lookahead * std::sin(alpha);
    const double error = p.wall_distance - projected;    // + = too close
    const double h = -p.wall_side * p.wall_kp * error;   // too close -> steer away
    const bool toward_wall = p.wall_side * h > 0.0;
    const double limit = radians(toward_wall ? p.wall_approach_limit_deg : p.wall_away_limit_deg);
    return std::clamp(h, -limit, limit);
  }

  std::vector<Point> points_;
  std::vector<double> headings_, hard_, comfort_;
  double heading_ = 0.0;      // smoothed chosen heading (rad, + = left)
  double last_speed_ = 0.0;   // speed we last commanded (our "speedometer")
  bool reversing_ = false;
  double reversed_ = 0.0;
  double reverse_steer_ = 0.0;
  int blocked_ = 0;
  mutable int ghost_count_ = 0;  // ghosts driven in the current plan (for the timing report)
};
}  // namespace gap
// ==== CORE END ====================================================================

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

class ReactiveFollowGap : public rclcpp::Node
{
public:
  ReactiveFollowGap() : Node("reactive_node")
  {
    gap::Params & p = planner_.p;

    // Every decimal parameter, straight from the table in the core.
    for (const gap::ParamRef & ref : gap::doubleParams()) {
      p.*(ref.field) = declare_parameter<double>(ref.name, p.*(ref.field));
    }
    p.follow_wall = declare_parameter<bool>("follow_wall", p.follow_wall);
    setWallSide(declare_parameter<std::string>("wall_side", "right"));
    p.back_up = declare_parameter<bool>("back_up", p.back_up);
    p.beam_skip = static_cast<int>(declare_parameter<int>("beam_skip", p.beam_skip));
    p.blocked_plans = static_cast<int>(declare_parameter<int>("blocked_plans", p.blocked_plans));
    // The simulator sends scans far faster than a real LiDAR; plan at this rate.
    plan_rate_hz_ = declare_parameter<double>("plan_rate_hz", 50.0);

    param_cb_ = add_on_set_parameters_callback(
      std::bind(&ReactiveFollowGap::onParams, this, std::placeholders::_1));

    drive_pub_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/gap_follow/markers", 10);
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10, std::bind(&ReactiveFollowGap::onScan, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(),
      "Ghost-car planner: body %.2f m wide, margins %.2f comfort / %.2f hard, headings +/-%.0f deg "
      "every %.0f deg, speed %.1f-%.1f m/s, back-up %s",
      2.0 * p.car_half_width, p.side_margin, p.hard_margin, p.heading_limit_deg,
      p.heading_step_deg, p.min_speed, p.max_speed, p.back_up ? "ON" : "OFF");
    if (p.follow_wall) {
      RCLCPP_INFO(get_logger(),
        "Wall advisor ON: %s wall at %.2f m (kp=%.2f, lookahead=%.2f m) -> %s laps",
        p.wall_side < 0.0 ? "RIGHT" : "LEFT", p.wall_distance, p.wall_kp, p.wall_lookahead,
        p.wall_side < 0.0 ? "clockwise" : "counter-clockwise");
    } else {
      RCLCPP_INFO(get_logger(), "Wall advisor OFF (ghost cars only)");
    }
  }

private:
  void setWallSide(const std::string & side)
  {
    if (side != "right" && side != "left") {
      RCLCPP_WARN(get_logger(), "wall_side must be \"right\" or \"left\"; using right");
    }
    planner_.p.wall_side = (side == "left") ? 1.0 : -1.0;
  }

  // Live tuning: `ros2 param set /reactive_node <name> <value>` while driving.
  rcl_interfaces::msg::SetParametersResult onParams(const std::vector<rclcpp::Parameter> & params)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    gap::Params & p = planner_.p;
    for (const auto & param : params) {
      const std::string & n = param.get_name();
      if (n == "follow_wall" || n == "back_up") {
        if (param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
          (n == "follow_wall" ? p.follow_wall : p.back_up) = param.as_bool();
          RCLCPP_INFO(get_logger(), "Live update: %s = %s", n.c_str(),
            param.as_bool() ? "true" : "false");
        }
        continue;
      }
      if (n == "wall_side") {
        if (param.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
          setWallSide(param.as_string());
          RCLCPP_INFO(get_logger(), "Live update: wall_side = %s",
            p.wall_side < 0.0 ? "right" : "left");
        }
        continue;
      }
      double * target = (n == "plan_rate_hz") ? &plan_rate_hz_ : nullptr;
      for (const gap::ParamRef & ref : gap::doubleParams()) {
        if (n == ref.name) target = &(p.*(ref.field));
      }
      if (!target) continue;
      if (param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
        result.successful = false;
        result.reason = n + " needs a decimal value, e.g. 2.0 instead of 2";
        return result;
      }
      *target = param.as_double();
      RCLCPP_INFO(get_logger(), "Live update: %s = %.3f", n.c_str(), *target);
    }
    return result;
  }

  static geometry_msgs::msg::Point point(double x, double y)
  {
    geometry_msgs::msg::Point pt;
    pt.x = x;
    pt.y = y;
    return pt;
  }

  visualization_msgs::msg::Marker lineMarker(const sensor_msgs::msg::LaserScan & scan, int id,
    int type, float r, float g, float b) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header = scan.header;
    marker.ns = "gap_follow";
    marker.id = id;
    marker.type = type;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.02;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 1.0f;
    marker.lifetime = rclcpp::Duration::from_seconds(0.25);
    return marker;
  }

  // Orange line: the chosen ghost's path. Cyan box: the ghost car where that
  // path ends. Green line: the heading the wall advisor asked for.
  void publishMarkers(const sensor_msgs::msg::LaserScan & scan, const gap::Output & out)
  {
    const gap::Params & p = planner_.p;
    visualization_msgs::msg::MarkerArray markers;

    auto path = lineMarker(scan, 0, visualization_msgs::msg::Marker::LINE_STRIP, 1.0f, 0.6f, 0.0f);
    const double length = std::max(0.3, out.free);
    const double front = p.lidar_to_rear_axle + p.lidar_to_front;
    gap::Pose last{-p.lidar_to_rear_axle, 0.0, 1.0, 0.0};
    for (double d = 0.0; d <= length + 1e-6; d += 0.1) {
      last = planner_.ghostPose(out.heading, d);
      path.points.push_back(point(last.x + front * last.c, last.y + front * last.s));
    }
    markers.markers.push_back(path);

    auto box = lineMarker(scan, 1, visualization_msgs::msg::Marker::LINE_STRIP, 0.0f, 1.0f, 1.0f);
    const double rear = -(p.lidar_to_rear - p.lidar_to_rear_axle);
    const double w = p.car_half_width;
    const double corners[5][2] = {{front, w}, {front, -w}, {rear, -w}, {rear, w}, {front, w}};
    for (const auto & corner : corners) {
      box.points.push_back(point(last.x + corner[0] * last.c - corner[1] * last.s,
                                 last.y + corner[0] * last.s + corner[1] * last.c));
    }
    markers.markers.push_back(box);

    auto wish = lineMarker(scan, 2, visualization_msgs::msg::Marker::LINE_STRIP, 0.0f, 1.0f, 0.0f);
    wish.points.push_back(point(0.0, 0.0));
    wish.points.push_back(point(std::cos(out.preferred), std::sin(out.preferred)));
    if (!p.follow_wall) wish.action = visualization_msgs::msg::Marker::DELETE;
    markers.markers.push_back(wish);

    marker_pub_->publish(markers);
  }

  void onScan(const sensor_msgs::msg::LaserScan & scan)
  {
    if (scan.ranges.empty() || !(scan.angle_increment > 0.0)) return;

    // Plan at plan_rate_hz; the last command stays in effect in between.
    const rclcpp::Time stamp(scan.header.stamp);
    double dt = 1.0 / std::max(1.0, plan_rate_hz_);
    if (has_last_plan_) {
      const double since = (stamp - last_plan_).seconds();
      if (since >= 0.0 && since < 0.9 * dt) return;
      if (since > 0.0 && since < 0.5) dt = since;
    }
    last_plan_ = stamp;
    has_last_plan_ = true;

    const gap::ScanView view{scan.ranges, scan.angle_min, scan.angle_increment,
                             scan.range_min, scan.range_max};
    const double speed_before = planner_.lastSpeed();

    // Time the planning itself: scan in -> steering angle and speed out.
    // steady_clock is a stopwatch: it only counts forward, so it is the right
    // clock for measuring how long something took.
    const auto plan_start = std::chrono::steady_clock::now();
    const gap::Output out = planner_.plan(view, dt);
    const double plan_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - plan_start).count();
    recordPlanTime(plan_ms);

    ackermann_msgs::msg::AckermannDriveStamped command;
    command.header = scan.header;
    command.drive.steering_angle = static_cast<float>(out.steer);
    command.drive.speed = static_cast<float>(out.speed);
    drive_pub_->publish(command);

    if (out.started_reverse) {
      RCLCPP_WARN(get_logger(), "No way forward: stopping, then backing up %.2f m",
        planner_.p.reverse_distance);
    }
    if (out.boxed_in) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
        "Boxed in: no way forward%s", planner_.p.back_up ? " and no room to back up" : "");
    }
    if (out.reversing) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
        "BACKING UP | steer=%+.3f | speed=%.2f", out.steer, out.speed);
      return;
    }
    publishMarkers(scan, out);
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
      "%s | wants %+.0f deg | heading=%+.0f deg | ghost gets %.2f m | wall=%.2f m | "
      "steer=%+.3f | speed=%.2f (was %.2f) | plan %.2f ms for %d ghosts "
      "(last %d plans: avg %.2f, worst %.2f; worst ever %.2f; budget %.0f ms)",
      out.mode, gap::degrees(out.preferred), gap::degrees(out.heading), out.free,
      out.wall_distance, out.steer, out.speed, speed_before, plan_ms, out.ghosts,
      kTimingWindow, window_avg_ms_, window_worst_ms_, worst_ever_ms_,
      1000.0 / std::max(1.0, plan_rate_hz_));
  }

  // Keep the average and worst planning time over each block of
  // kTimingWindow plans (about one second), plus the worst ever seen.
  void recordPlanTime(double ms)
  {
    sum_ms_ += ms;
    block_worst_ms_ = std::max(block_worst_ms_, ms);
    worst_ever_ms_ = std::max(worst_ever_ms_, ms);
    if (++block_count_ >= kTimingWindow) {
      window_avg_ms_ = sum_ms_ / block_count_;
      window_worst_ms_ = block_worst_ms_;
      sum_ms_ = 0.0;
      block_worst_ms_ = 0.0;
      block_count_ = 0;
    }
  }

  gap::Planner planner_;
  double plan_rate_hz_ = 50.0;
  rclcpp::Time last_plan_{0, 0, RCL_ROS_TIME};
  bool has_last_plan_ = false;

  // Planning-time bookkeeping (milliseconds)
  static constexpr int kTimingWindow = 50;
  double sum_ms_ = 0.0, block_worst_ms_ = 0.0;
  int block_count_ = 0;
  double window_avg_ms_ = 0.0, window_worst_ms_ = 0.0, worst_ever_ms_ = 0.0;

  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  OnSetParametersCallbackHandle::SharedPtr param_cb_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ReactiveFollowGap>());
  rclcpp::shutdown();
  return 0;
}
