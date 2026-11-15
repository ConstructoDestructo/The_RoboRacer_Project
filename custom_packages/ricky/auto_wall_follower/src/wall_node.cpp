#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker.hpp"

// Lab 3: right-wall following with PID, plus recess bridging.
//
// FOLLOW (normal driving), every scan:
//   1. Read two beams: b points straight right (-90 deg), a is theta degrees
//      further forward.
//   2. From a and b, estimate alpha (the car's angle to the wall) and the
//      current distance to the wall.
//   3. Project that distance one lookahead ahead (where the car WILL be).
//   4. PID on the error between the desired and projected distance -> steering.
//   5. Pick speed from the steering angle (the lab's speed schedule).
//   Gradual bends need no special handling: beam a sees the bend first and
//   the projected error steers the car around it.
//
// BRIDGE (a hole appeared in the wall: pocket or corner?):
//   A real wall moves only a few cm between scans (40 Hz). If beam a or b
//   suddenly reads much deeper, a beam fell into a hole in the wall.
//   The car drives straight and waits until beam b (pointing straight right)
//   is beside the hole, then measures how deep it is:
//   - Shallow (a recess / sealed doorway)  -> keep driving straight past it,
//                                             resume FOLLOW when the wall returns.
//   - Deep (a whole hallway opens up)      -> it's a real corner: resume FOLLOW
//                                             now, and the PID turns into it.
//   Note: "is there a wall ahead?" is NOT used, because some real corners
//   have open space straight ahead (a dead-end stub or an open hallway end).
class WallFollower : public rclcpp::Node
{
public:
    WallFollower() : Node("WALLFOLLOWER")
    {
        // Geometry

	//This is the desired distance we want to keep from the wall
        desired_distance_ = declare_parameter<double>("WF.desiredDistance", 0.75);

	//This is the angle seperating scan bem a from scan beam b
	// seperation constraint (ow far the angles are apart) is 0 < theta <= 70
        theta_deg_ = declare_parameter<double>("WF.theta", 50.0);

	//This is the distance the vehicle will be ahead of itself in the future
	//In this intance, it is initialized to 0.50 m ahead of its current position
        lookahead_distance_ = declare_parameter<double>("WF.lookaheadDistance", 0.50);

        // Each range = median of the beams within +/- this many degrees.
        // 0 = single beam (old behavior). 1.0 deg ~ 9 beams on the f1tenth LiDAR.
        beam_window_deg_ = declare_parameter<double>("WF.beamWindowDeg", 1.0);

        // PID gains (tune these)
        kp_ = declare_parameter<double>("WF.kp", 1.0);
        ki_ = declare_parameter<double>("WF.ki", 0.0);
        kd_ = declare_parameter<double>("WF.kd", 0.0);

        // Smooths the D term over roughly this many seconds before using it.
        // Raw scan-to-scan differences are mostly noise; 0 = unfiltered.
        d_filter_tau_ = declare_parameter<double>("WF.dFilterTau", 0.08);
        max_steering_ = declare_parameter<double>("WF.maxSteeringAngle", 0.34);

        // Lab speed schedule, chosen by steering angle
        straight_speed_ = declare_parameter<double>("WF.straightSpeed", 1.5);  // < 10 deg
        moderate_speed_ = declare_parameter<double>("WF.moderateSpeed", 1.0);  // 10-20 deg
        sharp_speed_ = declare_parameter<double>("WF.sharpSpeed", 0.5);        // > 20 deg

        // Recess bridging
        // A beam reading this much deeper than last scan = it fell into a hole.
        gap_jump_ = declare_parameter<double>("WF.gapJump", 0.30);

        // Hole deeper than this beside the car = a hallway (real corner), not
        // a recess. Set it between your deepest recess and your narrowest
        // side hallway (measure both in RViz).
        max_pocket_depth_ = declare_parameter<double>("WF.maxPocketDepth", 1.5);

        // Only bridge when the car was tracking calmly, not mid-turn.
        bridge_max_alpha_deg_ = declare_parameter<double>("WF.bridgeMaxAlphaDeg", 15.0);

        // Resume FOLLOW when both beams are back within this of their old values...
        bridge_tolerance_ = declare_parameter<double>("WF.bridgeTolerance", 0.25);

        // ...for this many scans in a row.
        bridge_confirm_scans_ = declare_parameter<int>("WF.bridgeConfirmScans", 3);

        // Safety net: never bridge farther than this.
        max_bridge_length_ = declare_parameter<double>("WF.maxBridgeLength", 3.0);
        bridge_speed_ = declare_parameter<double>("WF.bridgeSpeed", 1.0);

        // Live tuning: `ros2 param set` / `ros2 param load` take effect while driving.
        param_cb_ = add_on_set_parameters_callback(
            std::bind(&WallFollower::onParams, this, std::placeholders::_1));

        drive_pub_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);
        beam_a_pub_ = create_publisher<visualization_msgs::msg::Marker>("/wall_follower/beam_a", 10);
        beam_b_pub_ = create_publisher<visualization_msgs::msg::Marker>("/wall_follower/beam_b", 10);
        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&WallFollower::onScan, this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(),
            "Wall follower (PID + bridging): desired=%.2f m, theta=%.0f deg, "
            "lookahead=%.2f m, beamWindow=+/-%.1f deg, kp=%.3f ki=%.3f kd=%.3f, "
            "gapJump=%.2f m, maxPocketDepth=%.2f m",
            desired_distance_, theta_deg_, lookahead_distance_, beam_window_deg_,
            kp_, ki_, kd_, gap_jump_, max_pocket_depth_);
    }

private:
    static constexpr double pi_ = 3.14159265358979323846;
    static constexpr double integral_limit_ = 2.0;  // anti-windup clamp

    enum class State { FOLLOW, BRIDGE };
    State state_ = State::FOLLOW;

    double desired_distance_, theta_deg_, lookahead_distance_, beam_window_deg_;
    double kp_, ki_, kd_, d_filter_tau_, max_steering_;
    double straight_speed_, moderate_speed_, sharp_speed_;
    double gap_jump_, max_pocket_depth_, bridge_max_alpha_deg_;
    double bridge_tolerance_, max_bridge_length_, bridge_speed_;
    int bridge_confirm_scans_;

    // PID memory
    double integral_ = 0.0;
    double previous_error_ = 0.0;
    double filtered_derivative_ = 0.0;
    rclcpp::Time previous_stamp_{0, 0, RCL_ROS_TIME};
    bool has_previous_ = false;

    // Last beam readings from normal FOLLOW driving (the "where the wall was")
    double last_a_ = 0.0;
    double last_b_ = 0.0;
    double last_alpha_ = 0.0;
    bool has_last_beams_ = false;

    // Bridge bookkeeping
    double bridged_distance_ = 0.0;
    int bridge_confirm_count_ = 0;
    rclcpp::Time last_scan_stamp_{0, 0, RCL_ROS_TIME};
    bool has_last_scan_stamp_ = false;

    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr beam_a_pub_, beam_b_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    OnSetParametersCallbackHandle::SharedPtr param_cb_;

    // Called when parameters change at runtime. Copies the new values into
    // the variables the controller actually uses.
    rcl_interfaces::msg::SetParametersResult onParams(
        const std::vector<rclcpp::Parameter>& params)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for (const auto& p : params) {
            const std::string& name = p.get_name();
            if (name == "WF.bridgeConfirmScans") {
                if (p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
                    bridge_confirm_scans_ = static_cast<int>(p.as_int());
                continue;
            }
            double* target =
                name == "WF.kp" ? &kp_ :
                name == "WF.ki" ? &ki_ :
                name == "WF.kd" ? &kd_ :
                name == "WF.dFilterTau" ? &d_filter_tau_ :
                name == "WF.desiredDistance" ? &desired_distance_ :
                name == "WF.theta" ? &theta_deg_ :
                name == "WF.lookaheadDistance" ? &lookahead_distance_ :
                name == "WF.beamWindowDeg" ? &beam_window_deg_ :
                name == "WF.maxSteeringAngle" ? &max_steering_ :
                name == "WF.straightSpeed" ? &straight_speed_ :
                name == "WF.moderateSpeed" ? &moderate_speed_ :
                name == "WF.sharpSpeed" ? &sharp_speed_ :
                name == "WF.gapJump" ? &gap_jump_ :
                name == "WF.maxPocketDepth" ? &max_pocket_depth_ :
                name == "WF.bridgeMaxAlphaDeg" ? &bridge_max_alpha_deg_ :
                name == "WF.bridgeTolerance" ? &bridge_tolerance_ :
                name == "WF.maxBridgeLength" ? &max_bridge_length_ :
                name == "WF.bridgeSpeed" ? &bridge_speed_ : nullptr;
            if (!target) continue;
            if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
                result.successful = false;
                result.reason = name + " needs a decimal value, e.g. 2.0 instead of 2";
                return result;
            }
            *target = p.as_double();
            if (name == "WF.ki") integral_ = 0.0;  // drop windup from old gain
            RCLCPP_INFO(get_logger(), "Live update: %s = %.3f", name.c_str(), *target);
        }
        return result;
    }

    static double radians(double deg) { return deg * pi_ / 180.0; }
    static double degrees(double rad) { return rad * 180.0 / pi_; }

    // One beam's range, cleaned up.
    // NaN/inf means "nothing hit": treat it as far away so the math stays finite.
    static double cleanRange(const sensor_msgs::msg::LaserScan& scan, int i)
    {
        const double r = scan.ranges[i];
        if (!std::isfinite(r) || r > scan.range_max) return scan.range_max;
        if (r < scan.range_min) return scan.range_min;
        return r;
    }

    // Range toward angle_deg (0 = straight ahead, -90 = right), taken as the
    // MEDIAN of the beams within +/- WF.beamWindowDeg of it. One beam carries
    // ~1 cm of random noise every scan; the median of several cancels most of
    // it, and unlike an average it ignores a few beams that slip past a wall
    // edge onto a farther surface.
    double getRange(const sensor_msgs::msg::LaserScan& scan, double angle_deg, int& idx) const
    {
        const int n = static_cast<int>(scan.ranges.size());
        const double position = (radians(angle_deg) - scan.angle_min) / scan.angle_increment;
        idx = std::clamp(static_cast<int>(std::lround(position)), 0, n - 1);
        const int half = std::max(0, static_cast<int>(
            std::lround(radians(beam_window_deg_) / scan.angle_increment)));
        std::vector<double> window;
        window.reserve(2 * half + 1);
        for (int i = std::max(0, idx - half); i <= std::min(n - 1, idx + half); ++i)
            window.push_back(cleanRange(scan, i));
        auto middle = window.begin() + window.size() / 2;
        std::nth_element(window.begin(), middle, window.end());
        return *middle;
    }

    // Lab speed schedule. Uses the UNCLAMPED PID output: with a 0.34 rad
    // (~19.5 deg) steering limit, the clamped angle could never exceed 20 deg.
    double speedFor(double raw_steer) const
    {
        const double steer_deg = std::abs(degrees(raw_steer));
        if (steer_deg < 10.0) return straight_speed_;
        if (steer_deg < 20.0) return moderate_speed_;
        return sharp_speed_;
    }

    void publishDrive(const sensor_msgs::msg::LaserScan& scan, double steer, double speed)
    {
        ackermann_msgs::msg::AckermannDriveStamped command;
        command.header = scan.header;
        command.drive.steering_angle = std::clamp(steer, -max_steering_, max_steering_);
        command.drive.speed = speed;
        drive_pub_->publish(command);
    }

    void resetPid()
    {
        integral_ = 0.0;
        filtered_derivative_ = 0.0;
        has_previous_ = false;
    }

    void beamMarker(const sensor_msgs::msg::LaserScan& scan, int idx, double range, bool green,
                    const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr& pub)
    {
        visualization_msgs::msg::Marker marker;
        marker.header = scan.header;
        marker.ns = "wall_follower_beams";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::ARROW;
        marker.action = visualization_msgs::msg::Marker::ADD;
        const double angle = scan.angle_min + idx * scan.angle_increment;
        geometry_msgs::msg::Point start, tip;
        tip.x = range * std::cos(angle);
        tip.y = range * std::sin(angle);
        marker.points = {start, tip};
        marker.scale.x = 0.01; marker.scale.y = 0.03; marker.scale.z = 0.03;
        marker.color.r = green ? 0.0f : 1.0f;
        marker.color.g = green ? 1.0f : 0.0f;
        marker.color.a = 1.0f;
        marker.lifetime = rclcpp::Duration::from_seconds(0.25);
        pub->publish(marker);
    }

    void onScan(const sensor_msgs::msg::LaserScan& scan)
    {
        if (scan.ranges.empty() || !(scan.angle_increment > 0.0)) return;
        const rclcpp::Time stamp(scan.header.stamp);
        double scan_dt = 0.0;
        if (has_last_scan_stamp_) scan_dt = (stamp - last_scan_stamp_).seconds();
        last_scan_stamp_ = stamp;
        has_last_scan_stamp_ = true;

        // 1. Two beams on the right side
        const double theta = radians(std::clamp(theta_deg_, 1.0, 70.0));
        int b_idx = 0, a_idx = 0;
        const double b = getRange(scan, -90.0, b_idx);
        const double a = getRange(scan, -90.0 + degrees(theta), a_idx);
        beamMarker(scan, b_idx, b, false, beam_b_pub_);
        beamMarker(scan, a_idx, a, true, beam_a_pub_);

        // 2. Angle to the wall
        const double alpha = std::atan2(a * std::cos(theta) - b, a * std::sin(theta));

        // ---- Hole detection: a beam suddenly reads much deeper ----
        if (state_ == State::FOLLOW && has_last_beams_) {
            const bool hole = (a - last_a_ > gap_jump_) || (b - last_b_ > gap_jump_);
            const bool calm = std::abs(last_alpha_) < radians(bridge_max_alpha_deg_);
            if (hole && calm) {
                state_ = State::BRIDGE;
                bridged_distance_ = 0.0;
                bridge_confirm_count_ = 0;
                RCLCPP_WARN(get_logger(),
                    "-> BRIDGE: hole in wall (a=%.2f was %.2f, b=%.2f was %.2f)",
                    a, last_a_, b, last_b_);
            }
        }

        // ---- BRIDGE: drive straight until we know pocket vs. corner ----
        if (state_ == State::BRIDGE) {
            if (scan_dt > 0.0 && scan_dt < 0.5) bridged_distance_ += bridge_speed_ * scan_dt;

            // Beam b is beside the hole and it goes deep: a hallway, so turn.
            const bool real_corner = b - last_b_ > max_pocket_depth_;

            // Both beams see the wall where it was: we're past the pocket.
            const bool wall_back =
                std::abs(a - last_a_) < bridge_tolerance_ &&
                std::abs(b - last_b_) < bridge_tolerance_ &&
                std::abs(alpha) < radians(bridge_max_alpha_deg_);
            bridge_confirm_count_ = wall_back ? bridge_confirm_count_ + 1 : 0;
            const bool past_pocket = bridge_confirm_count_ >= std::max(1, bridge_confirm_scans_);

            const bool too_long = bridged_distance_ > max_bridge_length_;

            if (!real_corner && !past_pocket && !too_long) {
                publishDrive(scan, 0.0, bridge_speed_);
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
                    "BRIDGE | a=%.2f (was %.2f) | b=%.2f (was %.2f, depth %.2f) | "
                    "bridged=%.2f m",
                    a, last_a_, b, last_b_, b - last_b_, bridged_distance_);
                return;
            }
            state_ = State::FOLLOW;
            resetPid();
            RCLCPP_WARN(get_logger(), "BRIDGE -> FOLLOW: %s (hole depth %.2f m)",
                real_corner ? "deep opening beside us, real corner: turning"
                : past_pocket ? "wall is back where it was: recess skipped"
                              : "bridged too far: following whatever is there",
                b - last_b_);
        }

        // ---- FOLLOW: normal PID wall following ----
        const double distance = b * std::cos(alpha);
        // 3. Where the car will be one lookahead ahead
        const double projected = distance + lookahead_distance_ * std::sin(alpha);

        // 4. PID (right wall: too close -> positive error -> steer left)
        const double error = desired_distance_ - projected;
        if (has_previous_) {
            const double dt = (stamp - previous_stamp_).seconds();
            if (dt > 0.0 && dt < 0.5) {
                integral_ = std::clamp(integral_ + error * dt, -integral_limit_, integral_limit_);
                const double raw_derivative = (error - previous_error_) / dt;
                // Low-pass filter: move only part of the way toward each new
                // raw value, so one noisy scan can't jerk the steering.
                const double blend = dt / (std::max(0.0, d_filter_tau_) + dt);
                filtered_derivative_ += blend * (raw_derivative - filtered_derivative_);
            }
        }
        const double derivative = filtered_derivative_;
        previous_error_ = error;
        previous_stamp_ = stamp;
        has_previous_ = true;

        const double raw_steer = kp_ * error + ki_ * integral_ + kd_ * derivative;

        // 5. Speed from steering
        const double speed = speedFor(raw_steer);
        publishDrive(scan, raw_steer, speed);

        // Remember where the wall was, for hole detection on the next scan
        last_a_ = a;
        last_b_ = b;
        last_alpha_ = alpha;
        has_last_beams_ = true;

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
            "FOLLOW | alpha=%.1f deg | dist=%.3f | projected=%.3f | error=%.3f | "
            "steer=%.3f | speed=%.2f",
            degrees(alpha), distance, projected, error,
            std::clamp(raw_steer, -max_steering_, max_steering_), speed);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WallFollower>());
    rclcpp::shutdown();
    return 0;
}
