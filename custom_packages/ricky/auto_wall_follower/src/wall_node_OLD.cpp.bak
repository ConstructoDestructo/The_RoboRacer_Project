#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker.hpp"

// Experiment 5: corner speed from physics instead of fixed crawl speeds:
// grip-limited turn speed v = sqrt(aLat * R), late braking along
// v^2 = vc^2 + 2*a*d, faster exit acceleration, speed-scaled pure pursuit
// lookahead, and a latency-compensated turn start. Path logic unchanged.
// Experiment 4: separate new-wall DETECTION from ALIGNMENT.
// A changed-orientation wall on the right is tracked (line fit) as soon as it
// is geometrically reliable, regardless of the vehicle's heading relative to
// it, and steering pursues its clearance-offset line. ALIGN_WALL is entered
// only when its own preconditions hold, so it cannot bounce back into CHASE.
// Experiment 3.1: prevent unverified steering reversals during wall chasing.
// Retains adaptive corner steering and speed from Experiment 3.
// The turning target advances along the outgoing wall rather than remaining
// at a fixed offset from the original corner. Commands are rate-limited;
// this is experimental boundary following, NOT collision-free path planning.
// Corner tracking remains approximate without odometry.
class WallFollower : public rclcpp::Node
{
public:
    WallFollower() : Node("WALLFOLLOWER")
    {
        // Preserve the previously tested wall-following configuration.
        desired_distance_ = declare_parameter<double>("WF.desiredDistance", 0.75);
        max_scan_angle_deg_ = declare_parameter<double>("WF.maxWallScansAngle", 70.0);
        lookahead_distance_ = declare_parameter<double>("WF.lookaheadDistance", 0.50);
        kp_ = declare_parameter<double>("WF.kp", 0.5);
        ki_ = declare_parameter<double>("WF.ki", 0.0);
        kd_ = declare_parameter<double>("WF.kd", 0.0);
        speed_ = declare_parameter<double>("WF.speed", 2.0);
        max_steering_ = declare_parameter<double>("WF.maxSteeringAngle", 0.34);

        // Retain previous turn-related defaults for compatibility.
        turn_speed_ = declare_parameter<double>("WF.turnSpeed", 0.40);
        turn_steering_ = declare_parameter<double>("WF.turnSteeringAngle", 0.30);
        turn_distance_ = declare_parameter<double>("WF.turnInitiationDistance", 1.50);
        turn_timeout_ = declare_parameter<double>("WF.turnTimeout", 6.0);
        reacquisition_tolerance_ =
            declare_parameter<double>("WF.wallReacquisitionTolerance", 0.35);

        // Align faster, per the latest simulator experiment.
        align_speed_ = declare_parameter<double>("WF.alignSpeed", 0.75);
        align_angle_tolerance_deg_ =
            declare_parameter<double>("WF.alignAngleToleranceDeg", 5.0);
        align_confirm_scans_ = declare_parameter<int>("WF.alignConfirmScans", 5);
        align_kp_ = declare_parameter<double>("WF.alignKp", 0.7);

        // Keep the previous CHASE speed parameter as a backwards-compatible default.
        chase_speed_ = declare_parameter<double>("WF.chaseSpeed", 0.20);
        chase_approach_speed_ =
            declare_parameter<double>("WF.chaseApproachSpeed", 0.75);
        chase_turn_speed_ =
            declare_parameter<double>("WF.chaseTurnSpeed", chase_speed_);
        // Start turning when the corner is this far ahead in the LiDAR frame.
        // This is NOT a proven collision-free point: adjust for car geometry.
        chase_turn_start_distance_ =
            declare_parameter<double>("WF.chaseTurnStartDistance", 0.20);
        chase_approach_slowdown_distance_ =
            declare_parameter<double>("WF.chaseApproachSlowdownDistance", 1.50);
        chase_approach_decel_ =
            declare_parameter<double>("WF.chaseApproachDecel", 0.80);
        wheelbase_ = declare_parameter<double>("WF.wheelbase", 0.33);
        chase_corner_offset_ =
            declare_parameter<double>("WF.chaseCornerForwardOffset", 0.20);
        chase_min_turn_deg_ =
            declare_parameter<double>("WF.chaseMinTurnDeg", 30.0);
        chase_reacquire_scans_ =
            declare_parameter<int>("WF.chaseReacquireScans", 4);
        chase_max_duration_ =
            declare_parameter<double>("WF.chaseMaxDuration", 20.0);

        // Turn guidance and speed control: no changes to the proven PID gains.
        chase_turn_lookahead_ =
            declare_parameter<double>("WF.chaseTurnLookahead", 0.90);
        chase_reacquire_blend_ =
            declare_parameter<double>("WF.chaseReacquireBlend", 0.80);
        // Command rate limits applied in drive(). The f1tenth model allows
        // 9.51 m/s^2; deceleration must exceed WF.chaseBrakeDecel so the
        // planned braking curve is actually achievable.
        chase_acceleration_ =
            declare_parameter<double>("WF.chaseAcceleration", 3.00);
        chase_deceleration_ =
            declare_parameter<double>("WF.chaseDeceleration", 4.00);
        chase_right_front_warning_ =
            declare_parameter<double>("WF.chaseRightFrontWarning", 0.60);
        chase_right_front_stop_ =
            declare_parameter<double>("WF.chaseRightFrontStop", 0.30);

        // Experiment 4: new-wall tracking.
        // f1tenth_gym_ros (ST model): laser is 0.275 m ahead of base_link (CG)
        // and the rear axle is lr = 0.17145 m behind the CG. Pure pursuit and
        // the bicycle model rotate about the rear axle. 0.0 = previous geometry.
        lidar_to_rear_axle_ =
            declare_parameter<double>("WF.lidarToRearAxle", 0.446);
        chase_track_scans_ =
            declare_parameter<int>("WF.chaseTrackScans", 2);
        chase_min_wall_length_ =
            declare_parameter<double>("WF.chaseMinWallLength", 0.50);

        // Experiment 5: physics-based corner speed.
        // Lateral acceleration v^2/R used through a turn. Tyre limit is about
        // mu*g = 10 m/s^2; raise in steps (2 -> 3 -> 4 -> 5) while checking
        // exit distance, since the ST model understeers near the limit.
        chase_lateral_accel_ =
            declare_parameter<double>("WF.chaseLateralAccel", 3.0);
        chase_max_corner_speed_ =
            declare_parameter<double>("WF.chaseMaxCornerSpeed", 2.0);
        // Planned braking: approach speed and forward-clearance limits.
        chase_brake_decel_ =
            declare_parameter<double>("WF.chaseBrakeDecel", 3.0);
        chase_clearance_margin_ =
            declare_parameter<double>("WF.chaseClearanceMargin", 0.15);
        // Pure pursuit lookahead = chaseTurnLookahead + gain * speed.
        chase_lookahead_gain_ =
            declare_parameter<double>("WF.chaseLookaheadGain", 0.35);
        // Steering/command latency: start the turn v * t earlier. Default 0:
        // in testing, 0.15 s moved the apex 0.12 m closer to the inside
        // corner with no lap-time gain. Raise only if turns start late.
        steer_response_time_ =
            declare_parameter<double>("WF.steerResponseTime", 0.0);

        drive_pub_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);
        beam_a_pub_ = create_publisher<visualization_msgs::msg::Marker>("/wall_follower/beam_a", 10);
        beam_b_pub_ = create_publisher<visualization_msgs::msg::Marker>("/wall_follower/beam_b", 10);
        corner_pub_ = create_publisher<visualization_msgs::msg::Marker>("/wall_follower/corner", 10);
        goal_pub_ = create_publisher<visualization_msgs::msg::Marker>("/wall_follower/chase_target", 10);
        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&WallFollower::onScan, this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(),
            "Wall chase v5: ALIGN=%.2f, FOLLOW=%.2f m/s; corner aLat=%.1f m/s^2 "
            "(<=%.2f m/s), brake=%.1f m/s^2, accel=%.1f m/s^2; "
            "turn starts at corner x<=%.2f m + v*%.2f s",
            align_speed_, speed_, chase_lateral_accel_, chase_max_corner_speed_,
            chase_brake_decel_, chase_acceleration_,
            chase_turn_start_distance_, steer_response_time_);
    }

private:
    static constexpr double pi_ = 3.14159265358979323846;
    static constexpr double gap_threshold_ = 0.45;
    static constexpr double forward_stop_distance_ = 0.35;
    static constexpr int min_segment_points_ = 5;
    static constexpr double min_chase_seconds_ = 0.65;
    static constexpr double chase_line_rms_limit_ = 0.09;

    enum class State { ALIGN_WALL, FOLLOW_WALL, CHASE_WALL };
    enum class ChasePhase { APPROACH_CORNER, TURN_CORNER };
    State state_ = State::ALIGN_WALL;
    ChasePhase chase_phase_ = ChasePhase::TURN_CORNER;

    struct Point { double x = 0.0; double y = 0.0; };
    struct WallEstimate {
        bool valid = false;
        bool discontinuity = false;
        bool outward_gap = false;
        bool inward_gap = false;
        int b_idx = -1;
        int a_idx = -1;
        int gap_edge_idx = -1;
        int count = 0;
        double a = 0.0;
        double b = 0.0;
        double alpha = 0.0;
        double perpendicular = 0.0;
        double projected = 0.0;
        double line_rms = std::numeric_limits<double>::infinity();
    };
    // Least-squares line fitted to one straight piece of the scan.
    // direction is oriented along the expected travel direction; the wall
    // must lie on the RIGHT of that direction (right-wall following).
    struct WallSegment {
        bool valid = false;
        Point centroid;
        Point direction;
        double distance = 0.0;  // perpendicular, LiDAR to wall line
        double alpha = 0.0;     // same convention as WallEstimate::alpha
        double turn = 0.0;      // signed change from the CHASE entry wall
        double length = 0.0;
        double rms = std::numeric_limits<double>::infinity();
    };

    int aligned_scans_ = 0;
    double integral_ = 0.0;
    double previous_error_ = 0.0;
    rclcpp::Time previous_pid_stamp_{0, 0, RCL_ROS_TIME};
    bool has_previous_pid_ = false;

    // The corner is stored in the current scan frame. Its position is propagated
    // between scans with an APPROXIMATE commanded-speed bicycle model (not odometry).
    Point corner_;
    bool corner_valid_ = false;
    double chase_yaw_change_ = 0.0;
    double entry_alpha_ = 0.0;
    int chase_direction_ = -1;  // -1: right outward turn, +1: left inward turn
    int chase_wall_count_ = 0;   // consecutive scans satisfying the ALIGN handoff
    int chase_track_count_ = 0;  // consecutive scans with a new-wall segment
    double tracked_alpha_ = 0.0;
    rclcpp::Time chase_started_{0, 0, RCL_ROS_TIME};
    rclcpp::Time turn_started_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_scan_time_{0, 0, RCL_ROS_TIME};
    bool have_last_scan_time_ = false;
    double last_speed_command_ = 0.0;
    double last_steer_command_ = 0.0;

    double desired_distance_, max_scan_angle_deg_, lookahead_distance_;
    double kp_, ki_, kd_, speed_, max_steering_;
    double turn_speed_, turn_steering_, turn_distance_, turn_timeout_;
    double reacquisition_tolerance_;
    double align_speed_, align_angle_tolerance_deg_, align_kp_;
    int align_confirm_scans_;
    double chase_speed_, wheelbase_, chase_corner_offset_;
    double chase_approach_speed_, chase_turn_speed_;
    double chase_turn_start_distance_, chase_approach_slowdown_distance_;
    double chase_approach_decel_;
    double chase_min_turn_deg_, chase_max_duration_;
    double chase_turn_lookahead_, chase_reacquire_blend_;
    double chase_acceleration_, chase_deceleration_;
    double chase_right_front_warning_, chase_right_front_stop_;
    rclcpp::Time last_command_stamp_{0, 0, RCL_ROS_TIME};
    bool have_last_command_stamp_ = false;
    int chase_reacquire_scans_;
    double lidar_to_rear_axle_, chase_min_wall_length_;
    double chase_lateral_accel_, chase_max_corner_speed_;
    double chase_brake_decel_, chase_clearance_margin_;
    double chase_lookahead_gain_, steer_response_time_;
    int chase_track_scans_;

    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr beam_a_pub_, beam_b_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr corner_pub_, goal_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

    static double radians(double deg) { return deg * pi_ / 180.0; }
    static double degrees(double rad) { return rad * 180.0 / pi_; }

    bool indexAt(const sensor_msgs::msg::LaserScan& scan, double degrees_angle, int& idx) const
    {
        if (!(scan.angle_increment > 0.0) || scan.ranges.empty()) return false;
        const double position =
            (radians(degrees_angle) - scan.angle_min) / scan.angle_increment;
        if (!std::isfinite(position)) return false;
        idx = static_cast<int>(std::lround(position));
        return idx >= 0 && idx < static_cast<int>(scan.ranges.size());
    }

    bool rangeAt(const sensor_msgs::msg::LaserScan& scan, int idx, double& range) const
    {
        if (idx < 0 || idx >= static_cast<int>(scan.ranges.size())) return false;
        range = scan.ranges[idx];
        return std::isfinite(range) && range >= scan.range_min && range <= scan.range_max;
    }

    static Point lidarPoint(const sensor_msgs::msg::LaserScan& scan, int idx, double r)
    {
        const double angle = scan.angle_min + idx * scan.angle_increment;
        return {r * std::cos(angle), r * std::sin(angle)};
    }

    // Speed commands accelerate/decelerate smoothly between states. Any
    // requested stop is applied IMMEDIATELY (no ramp-down for emergencies).
    // This regulates commands, not actual measured vehicle speed.
    void drive(const sensor_msgs::msg::LaserScan& scan, double steer, double velocity)
    {
        const rclcpp::Time stamp(scan.header.stamp);
        double speed = std::max(0.0, velocity);
        if (speed > 0.0 && have_last_command_stamp_) {
            const double dt = (stamp - last_command_stamp_).seconds();
            if (dt > 0.0 && dt < 0.5) {
                const double lower = std::max(0.0, last_speed_command_ -
                    std::max(0.01, chase_deceleration_) * dt);
                const double upper = last_speed_command_ +
                    std::max(0.01, chase_acceleration_) * dt;
                speed = std::clamp(speed, lower, upper);
            }
        }
        ackermann_msgs::msg::AckermannDriveStamped command;
        command.header = scan.header;
        command.drive.steering_angle = std::clamp(steer, -max_steering_, max_steering_);
        command.drive.speed = speed;
        drive_pub_->publish(command);
        last_steer_command_ = command.drive.steering_angle;
        last_speed_command_ = command.drive.speed;
        last_command_stamp_ = stamp;
        have_last_command_stamp_ = true;
    }

    void beamMarker(const sensor_msgs::msg::LaserScan& scan, int idx, double range,
                    bool green,
                    const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr& pub)
    {
        visualization_msgs::msg::Marker marker;
        marker.header = scan.header;
        marker.ns = "wall_follower_beams";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::ARROW;
        marker.action = visualization_msgs::msg::Marker::ADD;
        geometry_msgs::msg::Point start, tip;
        const Point p = lidarPoint(scan, idx, range);
        tip.x = p.x; tip.y = p.y;
        marker.points = {start, tip};
        marker.scale.x = 0.01; marker.scale.y = 0.03; marker.scale.z = 0.03;
        marker.color.r = green ? 0.0f : 1.0f;
        marker.color.g = green ? 1.0f : 0.0f;
        marker.color.a = 1.0f;
        marker.lifetime = rclcpp::Duration::from_seconds(0.25);
        pub->publish(marker);
    }

    void sphereMarker(const sensor_msgs::msg::LaserScan& scan, Point location,
                      int id, bool corner)
    {
        visualization_msgs::msg::Marker marker;
        marker.header = scan.header;
        marker.ns = "wall_chase";
        marker.id = id;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = location.x;
        marker.pose.position.y = location.y;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.12;
        marker.scale.y = 0.12;
        marker.scale.z = 0.12;
        marker.color.r = corner ? 1.0f : 0.0f;
        marker.color.g = corner ? 0.8f : 0.5f;
        marker.color.b = corner ? 0.0f : 1.0f;
        marker.color.a = 1.0f;
        marker.lifetime = rclcpp::Duration::from_seconds(0.25);
        (corner ? corner_pub_ : goal_pub_)->publish(marker);
    }

    void resetPid()
    {
        integral_ = 0.0;
        has_previous_pid_ = false;
    }

    void toAlign(const char* explanation)
    {
        RCLCPP_INFO(get_logger(), "-> ALIGN_WALL: %s", explanation);
        state_ = State::ALIGN_WALL;
        aligned_scans_ = 0;
        chase_wall_count_ = 0;
        chase_track_count_ = 0;
        corner_valid_ = false;
        chase_phase_ = ChasePhase::TURN_CORNER;
        resetPid();
    }

    // First contiguous wall segment starting at right-facing beam B.
    // This keeps the original proven two-beam geometry out of invalid gaps.
    WallEstimate findRightWall(const sensor_msgs::msg::LaserScan& scan) const
    {
        WallEstimate result;
        if (!indexAt(scan, -90.0, result.b_idx) ||
            !rangeAt(scan, result.b_idx, result.b)) return result;

        const double limit = radians(std::clamp(max_scan_angle_deg_, 0.1, 70.0));
        const int last = std::min(static_cast<int>(scan.ranges.size()) - 1,
            static_cast<int>(std::floor(result.b_idx + limit / scan.angle_increment)));
        int previous_idx = result.b_idx;
        double previous_r = result.b;
        result.count = 1;

        for (int idx = result.b_idx + 1; idx <= last; ++idx) {
            double r = 0.0;
            if (!rangeAt(scan, idx, r)) {
                result.discontinuity = true;
                result.gap_edge_idx = previous_idx;
                break;
            }
            const double da = (idx - previous_idx) * scan.angle_increment;
            const double gap = std::sqrt(std::max(0.0,
                previous_r * previous_r + r * r -
                2.0 * previous_r * r * std::cos(da)));
            if (gap > gap_threshold_) {
                result.discontinuity = true;
                result.outward_gap = r > previous_r;
                result.inward_gap = r < previous_r;
                result.gap_edge_idx = previous_idx;
                break;
            }
            result.a_idx = idx;
            result.a = r;
            result.count++;
            previous_idx = idx;
            previous_r = r;
        }

        if (result.a_idx < 0 || result.count < min_segment_points_) return result;
        const double theta = (result.a_idx - result.b_idx) * scan.angle_increment;
        result.alpha = std::atan2(
            result.a * std::cos(theta) - result.b,
            result.a * std::sin(theta));
        result.perpendicular = result.b * std::cos(result.alpha);
        result.projected = result.perpendicular +
            lookahead_distance_ * std::sin(result.alpha);
        result.valid = std::isfinite(result.alpha) &&
            std::isfinite(result.projected) && result.perpendicular > 0.0 &&
            theta >= radians(10.0);
        if (!result.valid) return result;

        // RMS deviation from the chord between A and B. An L-shaped collection
        // of adjacent but differently oriented walls should not count as a line.
        const Point start = lidarPoint(scan, result.b_idx, result.b);
        const Point end = lidarPoint(scan, result.a_idx, result.a);
        const double vx = end.x - start.x;
        const double vy = end.y - start.y;
        const double length = std::hypot(vx, vy);
        if (length < 0.05) { result.valid = false; return result; }
        double sum_squared = 0.0;
        for (int idx = result.b_idx; idx <= result.a_idx; ++idx) {
            double r = 0.0;
            if (!rangeAt(scan, idx, r)) { result.valid = false; return result; }
            const Point p = lidarPoint(scan, idx, r);
            const double cross = vx * (p.y - start.y) - vy * (p.x - start.x);
            sum_squared += (cross / length) * (cross / length);
        }
        result.line_rms = std::sqrt(sum_squared / result.count);
        return result;
    }

    // Recursive end-point split: an L-shaped cluster (two walls meeting at an
    // inward corner without a range gap) becomes two straight pieces.
    static void splitStraight(const std::vector<Point>& pts, size_t first, size_t last,
                              int depth, std::vector<std::pair<size_t, size_t>>& out)
    {
        if (last < first + 7) return;
        const Point& a = pts[first];
        const Point& b = pts[last];
        const double vx = b.x - a.x;
        const double vy = b.y - a.y;
        const double length = std::hypot(vx, vy);
        size_t worst = first;
        double worst_distance = 0.0;
        if (length > 1e-6) {
            for (size_t i = first + 1; i < last; ++i) {
                const double d = std::abs(vx * (pts[i].y - a.y) -
                                          vy * (pts[i].x - a.x)) / length;
                if (d > worst_distance) { worst_distance = d; worst = i; }
            }
        }
        if (worst_distance > 0.10 && depth < 8) {
            splitStraight(pts, first, worst, depth + 1, out);
            splitStraight(pts, worst, last, depth + 1, out);
            return;
        }
        out.emplace_back(first, last);
    }

    // Find the NEAREST straight wall on the vehicle's right whose orientation
    // differs from the CHASE entry wall. This does NOT require the vehicle to
    // be parallel to it: heading misalignment is ALIGN's job, not a gate here.
    // Generalizes to bends other than 90 degrees (the turn sign comes from
    // chase_direction_, the magnitude from the measured line).
    WallSegment findNewWallSegment(const sensor_msgs::msg::LaserScan& scan) const
    {
        WallSegment best;
        int first = -1;
        int last = -1;
        if (!indexAt(scan, -135.0, first) || !indexAt(scan, +45.0, last))
            return best;

        // Original wall direction expressed in the current LiDAR frame.
        const double old_direction = -entry_alpha_ - chase_yaw_change_;
        const Point rear{-lidar_to_rear_axle_, 0.0};

        std::vector<Point> cluster;
        cluster.reserve(static_cast<size_t>(last - first + 1));
        const auto evaluate = [&]() {
            std::vector<std::pair<size_t, size_t>> pieces;
            if (cluster.size() >= 8)
                splitStraight(cluster, 0, cluster.size() - 1, 0, pieces);
            for (const auto& piece : pieces) {
                const double n = static_cast<double>(piece.second - piece.first + 1);
                double cx = 0.0, cy = 0.0;
                for (size_t i = piece.first; i <= piece.second; ++i) {
                    cx += cluster[i].x;
                    cy += cluster[i].y;
                }
                cx /= n;
                cy /= n;
                double sxx = 0.0, syy = 0.0, sxy = 0.0;
                for (size_t i = piece.first; i <= piece.second; ++i) {
                    const double dx = cluster[i].x - cx;
                    const double dy = cluster[i].y - cy;
                    sxx += dx * dx; syy += dy * dy; sxy += dx * dy;
                }
                // Total least squares: principal axis of the point cloud.
                double phi = 0.5 * std::atan2(2.0 * sxy, sxx - syy);
                // Orient the undirected line so the change from the old wall
                // has the sign of the chase (right turn: clockwise).
                double turn = std::remainder(phi - old_direction, 2.0 * pi_);
                if (turn * chase_direction_ < 0.0) {
                    phi += pi_;
                    turn = std::remainder(phi - old_direction, 2.0 * pi_);
                }
                const Point dir{std::cos(phi), std::sin(phi)};
                const Point left{-dir.y, dir.x};
                double residual = 0.0, s_min = 1e9, s_max = -1e9;
                for (size_t i = piece.first; i <= piece.second; ++i) {
                    const double e = (cluster[i].x - cx) * left.x +
                                     (cluster[i].y - cy) * left.y;
                    residual += e * e;
                    const double s = (cluster[i].x - rear.x) * dir.x +
                                     (cluster[i].y - rear.y) * dir.y;
                    s_min = std::min(s_min, s);
                    s_max = std::max(s_max, s);
                }
                WallSegment candidate;
                candidate.centroid = {cx, cy};
                candidate.direction = dir;
                candidate.distance = -(cx * left.x + cy * left.y);  // > 0: on the right
                candidate.alpha = std::remainder(-phi, 2.0 * pi_);
                candidate.turn = turn;
                candidate.length = s_max - s_min;
                candidate.rms = std::sqrt(residual / n);
                // s_max > 0.2: the wall must continue AHEAD of the rear axle;
                // a piece only behind the car is no use as a reference.
                candidate.valid =
                    candidate.rms <= chase_line_rms_limit_ &&
                    candidate.length >= chase_min_wall_length_ &&
                    s_max > 0.20 &&
                    std::abs(turn) >= radians(chase_min_turn_deg_) &&
                    std::abs(turn) <= radians(160.0) &&
                    candidate.distance > 0.15 &&
                    candidate.distance < desired_distance_ + reacquisition_tolerance_ + 1.0;
                if (candidate.valid &&
                    (!best.valid || candidate.distance < best.distance))
                    best = candidate;
            }
        };

        for (int idx = first; idx <= last; ++idx) {
            double r = 0.0;
            if (!rangeAt(scan, idx, r)) {
                evaluate();
                cluster.clear();
                continue;
            }
            const Point p = lidarPoint(scan, idx, r);
            if (!cluster.empty() &&
                std::hypot(p.x - cluster.back().x, p.y - cluster.back().y)
                    > gap_threshold_) {
                evaluate();
                cluster.clear();
            }
            cluster.push_back(p);
        }
        evaluate();
        return best;
    }

    // Pure pursuit about the REAR AXLE (the bicycle model's rotation point).
    // The target is given in the LiDAR frame.
    double pursuitSteer(const Point& target) const
    {
        const double x = target.x + lidar_to_rear_axle_;
        const double y = target.y;
        if (x < 0.05) return y >= 0.0 ? max_steering_ : -max_steering_;
        return std::atan2(2.0 * wheelbase_ * y, x * x + y * y);
    }

    // Lookahead point on the line offset desired_distance_ to the LEFT of
    // the tracked wall, measured from the rear axle's projection onto it.
    Point segmentTarget(const WallSegment& segment) const
    {
        const Point left{-segment.direction.y, segment.direction.x};
        const Point base{segment.centroid.x + desired_distance_ * left.x,
                         segment.centroid.y + desired_distance_ * left.y};
        const double along =
            (-lidar_to_rear_axle_ - base.x) * segment.direction.x +
            (0.0 - base.y) * segment.direction.y;
        const double progress = along + pursuitLookahead();
        return {base.x + progress * segment.direction.x,
                base.y + progress * segment.direction.y};
    }

    // Sector minimum: an unknown return is NOT interpreted as free space.
    bool sectorMinimum(const sensor_msgs::msg::LaserScan& scan,
                       double min_deg, double max_deg, double step_deg,
                       double& minimum) const
    {
        minimum = std::numeric_limits<double>::infinity();
        for (double angle = min_deg; angle <= max_deg + 1e-6; angle += step_deg) {
            int idx = -1;
            double r = 0.0;
            if (!indexAt(scan, angle, idx) || !rangeAt(scan, idx, r)) return false;
            minimum = std::min(minimum, r);
        }
        return std::isfinite(minimum);
    }

    // Rough bicycle integration for tracking a scan-frame corner between scans.
    // Commanded speed/steering are NOT a substitute for measured odometry.
    void advanceCornerEstimate(const rclcpp::Time& time)
    {
        if (have_last_scan_time_) {
            const double dt = (time - last_scan_time_).seconds();
            if (state_ == State::CHASE_WALL && dt > 0.0 && dt < 0.5 &&
                wheelbase_ > 0.01) {
                const double yaw_delta = last_speed_command_ *
                    std::tan(last_steer_command_) * dt / wheelbase_;
                chase_yaw_change_ += yaw_delta;
                if (corner_valid_) {
                    double dx = last_speed_command_ * dt;
                    double dy = 0.0;
                    if (std::abs(yaw_delta) > 1e-6) {
                        const double radius = last_speed_command_ * dt / yaw_delta;
                        dx = radius * std::sin(yaw_delta);
                        dy = radius * (1.0 - std::cos(yaw_delta));
                    }
                    // The arc is traced by the rear axle, not the LiDAR.
                    const double px = corner_.x + lidar_to_rear_axle_ - dx;
                    const double py = corner_.y - dy;
                    corner_.x = std::cos(yaw_delta) * px + std::sin(yaw_delta) * py
                        - lidar_to_rear_axle_;
                    corner_.y = -std::sin(yaw_delta) * px + std::cos(yaw_delta) * py;
                }
            }
        }
        last_scan_time_ = time;
        have_last_scan_time_ = true;
    }

    void startChasing(const sensor_msgs::msg::LaserScan& scan,
                      const WallEstimate& wall, int turn_direction,
                      const char* reason)
    {
        state_ = State::CHASE_WALL;
        chase_direction_ = turn_direction;
        chase_started_ = rclcpp::Time(scan.header.stamp);
        chase_yaw_change_ = 0.0;
        chase_wall_count_ = 0;
        chase_track_count_ = 0;
        entry_alpha_ = wall.valid ? wall.alpha : 0.0;
        corner_valid_ = false;
        chase_phase_ = ChasePhase::TURN_CORNER;
        turn_started_ = chase_started_;
        aligned_scans_ = 0;
        resetPid();

        if (turn_direction < 0 && wall.gap_edge_idx >= 0) {
            double r = 0.0;
            if (rangeAt(scan, wall.gap_edge_idx, r)) {
                corner_ = lidarPoint(scan, wall.gap_edge_idx, r);
                corner_valid_ = true;
            }
        }
        // Only a measured outward wall edge supports a slow approach.
        // Forward-obstruction or inward-corner cases retain the prior immediate
        // turning behavior (there may not be a forward approach lane).
        if (turn_direction < 0 && wall.outward_gap && corner_valid_ &&
            corner_.x > chase_turn_start_distance_) {
            chase_phase_ = ChasePhase::APPROACH_CORNER;
        }
        RCLCPP_WARN(get_logger(),
            "-> CHASE_WALL (%s) | %s | phase=%s | corner=%s x=%.2f",
            turn_direction < 0 ? "RIGHT" : "LEFT", reason,
            chase_phase_ == ChasePhase::APPROACH_CORNER ? "APPROACH" : "TURN",
            corner_valid_ ? "recorded" : "unavailable", corner_.x);
    }

    // Line orientation is undirected. Ignore 180-degree sign ambiguity.
    static double undirectedAngleDifference(double one, double two)
    {
        return std::acos(std::clamp(std::abs(std::cos(one - two)), 0.0, 1.0));
    }

    // The corner remains fixed in the map, but its coordinates in the LiDAR
    // frame change as the vehicle moves. Use actual observed edge points when
    // they agree with the dead-reckoned position; otherwise continue predicting.
    // Gate matching to avoid switching to a different discontinuity.
    void refreshCornerFromScan(const sensor_msgs::msg::LaserScan& scan,
                               const WallEstimate& wall)
    {
        if (!corner_valid_ || !wall.outward_gap || wall.gap_edge_idx < 0)
            return;
        double r = 0.0;
        if (!rangeAt(scan, wall.gap_edge_idx, r)) return;
        const Point observed = lidarPoint(scan, wall.gap_edge_idx, r);
        if (std::hypot(observed.x - corner_.x, observed.y - corner_.y) <= 0.40) {
            // Slight blending makes the marker less susceptible to scan noise.
            corner_.x = 0.30 * corner_.x + 0.70 * observed.x;
            corner_.y = 0.30 * corner_.y + 0.70 * observed.y;
        }
    }

    // Grip-limited speed for a steering angle: v = sqrt(a_lat * R), with the
    // rear-axle turning radius R = L / tan|delta|. Floored at the old crawl
    // speed so the car can always creep; capped by WF.chaseMaxCornerSpeed.
    double cornerSpeed(double steering) const
    {
        const double floor_speed = std::max(0.0, chase_turn_speed_);
        const double cap = std::max(floor_speed, chase_max_corner_speed_);
        const double curvature = std::tan(std::abs(steering)) / std::max(0.01, wheelbase_);
        if (curvature < 1e-6) return cap;
        const double grip = std::sqrt(std::max(0.0, chase_lateral_accel_) / curvature);
        return std::clamp(grip, floor_speed, cap);
    }

    // Highest speed that can still stop before forward clearance falls to the
    // emergency-stop distance plus a margin: v = sqrt(2 * a_brake * d).
    double clearanceSpeed(double forward_min) const
    {
        const double free_distance = std::max(0.0,
            forward_min - forward_stop_distance_ - chase_clearance_margin_);
        return std::sqrt(2.0 * std::max(0.01, chase_brake_decel_) * free_distance);
    }

    // Pure pursuit lookahead grows with speed so the turn stays smooth at pace.
    double pursuitLookahead() const
    {
        return std::max(0.35, chase_turn_lookahead_ +
            std::max(0.0, chase_lookahead_gain_) * last_speed_command_);
    }

    // Braking curve to the TURN point: the fastest speed from which the car
    // can still slow to the expected corner speed at chaseBrakeDecel.
    // The expected corner uses WF.turnSteeringAngle as a conservative bound.
    double approachSpeed(double distance_to_turn) const
    {
        const double corner_speed =
            cornerSpeed(std::min(max_steering_, turn_steering_));
        const double braking_speed = std::sqrt(
            corner_speed * corner_speed +
            2.0 * std::max(0.01, chase_brake_decel_) *
                std::max(0.0, distance_to_turn));
        return std::min(std::max(corner_speed, speed_), braking_speed);
    }

    // Build a MOVING lookahead target on the new right-wall trajectory.
    // The outgoing wall's tangent is 90 degrees clockwise for a right turn,
    // counterclockwise for a left turn, relative to the chase-entry heading.
    // As the vehicle turns, rotate that tangent into the current LiDAR frame.
    // Offset the target LEFT of the path tangent to keep the wall on our right.
    // Advancing the target along the tangent avoids chasing a point that is
    // already behind the vehicle (the primary failure in experiment 2).
    bool outgoingTarget(Point& target) const
    {
        if (!corner_valid_ || wheelbase_ <= 0.01) return false;
        const double desired_heading = chase_direction_ * pi_ / 2.0;
        const double relative_heading = desired_heading - chase_yaw_change_;
        const Point tangent{std::cos(relative_heading), std::sin(relative_heading)};
        const Point normal{-tangent.y, tangent.x};
        const Point base{
            corner_.x + desired_distance_ * normal.x,
            corner_.y + desired_distance_ * normal.y
        };
        const double projection = (-lidar_to_rear_axle_ - base.x) * tangent.x -
            base.y * tangent.y;
        const double progress = std::max(0.0, projection + pursuitLookahead());
        target = {base.x + progress * tangent.x,
                  base.y + progress * tangent.y};
        return std::isfinite(target.x) && std::isfinite(target.y) &&
            target.x + lidar_to_rear_axle_ > 0.05;
    }

    // Turn speed: the grip limit for the current steering, never more than
    // the speed that can still stop short of the forward wall.
    double adaptiveTurnSpeed(double steering, double forward_min) const
    {
        return std::min(cornerSpeed(steering), clearanceSpeed(forward_min));
    }

    void chase(const sensor_msgs::msg::LaserScan& scan, const WallEstimate& wall,
               const rclcpp::Time& stamp, double forward_min, bool forward_known)
    {
        const double elapsed = (stamp - chase_started_).seconds();
        refreshCornerFromScan(scan, wall);
        if (elapsed < 0.0 || elapsed > chase_max_duration_) {
            drive(scan, 0.0, 0.0);
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
                "CHASE_WALL exceeded %.1f seconds; stopping", chase_max_duration_);
            return;
        }
        if (!forward_known || forward_min < forward_stop_distance_) {
            drive(scan, 0.0, 0.0);
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                "CHASE_WALL forward clearance unsafe/unknown: stopping");
            return;
        }

        // Do not begin steering as soon as the LiDAR detects the opening.
        // First advance toward the saved wall edge along the original corridor.
        // Keep FOLLOW speed and brake only as late as the braking curve to
        // the grip-limited corner speed requires.
        if (chase_phase_ == ChasePhase::APPROACH_CORNER) {
            if (!corner_valid_) {
                drive(scan, 0.0, 0.0);
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                    "APPROACH_CORNER lost its corner reference; stopping");
                return;
            }
            // Start earlier at speed to cover steering/command latency.
            const double remaining = corner_.x - chase_turn_start_distance_ -
                std::max(0.0, steer_response_time_) * last_speed_command_;
            // Forward obstruction means the approach is no longer safe. If
            // a right passage is available, hand off to turning immediately;
            // otherwise stop and require further navigation logic.
            if (forward_min < turn_distance_ && remaining > 0.0) {
                double right_min = 0.0;
                const bool right_known = sectorMinimum(
                    scan, -65.0, -35.0, 5.0, right_min);
                if (!right_known || right_min <= 1.10) {
                    drive(scan, 0.0, 0.0);
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                        "APPROACH_CORNER blocked before turning point; stopping");
                    return;
                }
                RCLCPP_WARN(get_logger(),
                    "APPROACH_CORNER: forward wall approaching; start turn early");
            }
            else if (remaining > 0.0) {
                // Follow the still-visible ORIGINAL wall's orientation to
                // avoid accidentally steering into the opening. When no
                // trustworthy wall is available, use straight steering;
                // without odometry this is not full heading hold.
                double steer = 0.0;
                if (wall.valid && wall.line_rms <= chase_line_rms_limit_)
                    steer = std::clamp(-align_kp_ * wall.alpha, -0.12, 0.12);
                const double speed = approachSpeed(remaining);
                sphereMarker(scan, corner_, 0, true);
                drive(scan, steer, speed);
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
                    "APPROACH_CORNER | edgeX=%.2f m | toTurn=%.2f m | "
                    "speed=%.2f m/s | steer=%.3f | forward=%.2f m",
                    corner_.x, remaining, speed, steer, forward_min);
                return;
            }
            chase_phase_ = ChasePhase::TURN_CORNER;
            turn_started_ = stamp;
            chase_wall_count_ = 0;
            RCLCPP_WARN(get_logger(),
                "APPROACH_CORNER -> TURN_CORNER | corner x=%.2f m",
                corner_.x);
        }

        // (A) DETECT: nearest straight right-hand wall whose orientation
        // changed from the entry wall. No heading gate: the old check
        // |alpha| < 25 deg rejected the 90-degree wall until the car had
        // already turned ~65 deg more, and the stop rule below fired first.
        const WallSegment segment = findNewWallSegment(scan);
        if (segment.valid) {
            // A sudden jump in the tracked line means a different surface.
            if (chase_track_count_ > 0 &&
                std::abs(std::remainder(segment.alpha - tracked_alpha_, 2.0 * pi_))
                    > radians(20.0))
                chase_track_count_ = 0;
            chase_track_count_++;
            tracked_alpha_ = segment.alpha;
        } else {
            chase_track_count_ = 0;
        }
        const bool tracking =
            chase_track_count_ >= std::max(1, chase_track_scans_);

        // (C) HANDOFF: enter ALIGN_WALL only when ALIGN's own preconditions
        // hold (otherwise onScan() would immediately restart CHASE), and the
        // ordinary beam A/B estimate measures the SAME line as the tracker.
        const bool align_can_run = wall.valid && !wall.discontinuity &&
            wall.line_rms <= chase_line_rms_limit_ && forward_min >= turn_distance_;
        const bool same_wall = tracking && align_can_run &&
            std::abs(std::remainder(wall.alpha - segment.alpha, 2.0 * pi_))
                <= radians(12.0) &&
            std::abs(wall.perpendicular - segment.distance) <= 0.30;
        if ((stamp - turn_started_).seconds() >= min_chase_seconds_ && same_wall)
            chase_wall_count_++;
        else
            chase_wall_count_ = 0;

        if (chase_wall_count_ >= std::max(1, chase_reacquire_scans_)) {
            toAlign("new wall tracked and measurable by beams A/B; align to it");
            drive(scan, -align_kp_ * wall.alpha, align_speed_);
            return;
        }

        // Continuous steering: pursue an advancing waypoint on the outgoing
        // wall's clearance-offset path. Never force a permanent rightward
        // minimum: steering must be able to unwind and correct LEFT as needed.
        double steer = chase_direction_ * std::min(max_steering_, turn_steering_);
        Point target;
        if (outgoingTarget(target)) {
            steer = pursuitSteer(target);
            sphereMarker(scan, target, 1, false);
        }
        if (corner_valid_) sphereMarker(scan, corner_, 0, true);

        // (B) STEER BY THE NEW WALL: once measured, the (dead-reckoned,
        // 90-degree-assuming) corner target is phased out in favour of pure
        // pursuit on the measured wall's clearance-offset line.
        if (segment.valid) {
            const Point wall_target = segmentTarget(segment);
            const double wall_steer = pursuitSteer(wall_target);
            const double weight = std::clamp(
                chase_reacquire_blend_ * static_cast<double>(chase_track_count_) /
                static_cast<double>(std::max(1, chase_track_scans_)),
                0.0, 1.0);
            steer = (1.0 - weight) * steer + weight * wall_steer;
            target = wall_target;
            sphereMarker(scan, wall_target, 1, false);

            // Wall ahead while the tracked wall says we still must turn
            // further in the chase direction: demand at least a proportional
            // share of lock. Evidence-based (measured line), not a blind turn.
            const double still_to_turn = chase_direction_ *
                std::atan2(segment.direction.y, segment.direction.x);
            if (tracking && forward_min < turn_distance_ &&
                still_to_turn > radians(10.0)) {
                const double urgency = std::clamp(
                    (turn_distance_ - forward_min) /
                    std::max(0.05, turn_distance_ - forward_stop_distance_),
                    0.0, 1.0);
                steer = chase_direction_ < 0
                    ? std::min(steer, -urgency * max_steering_)
                    : std::max(steer, urgency * max_steering_);
            }
        }

        // If we are turning toward an uncomfortably close front-right wall,
        // bias the command LEFT and reduce speed. This is only a local
        // clearance heuristic; it does not guarantee collision avoidance.
        double right_front_min = 0.0;
        const bool right_front_known = sectorMinimum(
            scan, -55.0, -20.0, 5.0, right_front_min);
        if (chase_direction_ < 0 && right_front_known) {
            if (right_front_min < chase_right_front_stop_) {
                drive(scan, 0.0, 0.0);
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                    "TURN_CORNER: right-front clearance critical; STOP");
                return;
            }
            if (right_front_min < chase_right_front_warning_) {
                const double danger = std::clamp(
                    (chase_right_front_warning_ - right_front_min) /
                    std::max(0.05, chase_right_front_warning_ - chase_right_front_stop_),
                    0.0, 1.0);
                steer += 0.25 * danger;
            }
        }
        steer = std::clamp(steer, -max_steering_, max_steering_);

        // Experiment 3 exposed an unsafe shortcut: when the forward beam
        // became short, the controller automatically reversed direction and
        // commanded +0.30 rad (LEFT) based solely on an open left-side sector.
        // This did not establish that the tracked RIGHT wall actually bent left.
        // Do not turn left without reacquiring a relevant boundary segment.
        // Fail safely instead of inventing a left turn in an unverified space.
        // Gate on TRACKING (wall evidence exists), not on ALIGN readiness;
        // gating on the latter caused the permanent stop in Experiment 3.1.
        if (chase_direction_ < 0 && forward_min < 0.90 &&
            chase_yaw_change_ < -radians(35.0) && !tracking) {
            drive(scan, 0.0, 0.0);
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                "CHASE_WALL: forward path narrowing (%.2f m), "
                "no new right-wall segment tracked; STOP instead of blind LEFT turn "
                "| yaw=%.1f deg",
                forward_min, degrees(chase_yaw_change_));
            return;
        }

        double turn_speed = adaptiveTurnSpeed(steer, forward_min);
        if (right_front_known && right_front_min < chase_right_front_warning_)
            turn_speed = std::min(turn_speed, chase_turn_speed_);
        drive(scan, steer, turn_speed);
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
            "TURN_CORNER | direction=%s | elapsed=%.1f | yaw=%.1f deg | "
            "newWall=%s change=%.1f deg alpha=%.1f deg dist=%.2f len=%.2f "
            "track=%d | abAlpha=%.1f abValid=%d abGap=%d | handoff=%d/%d | "
            "cornerX=%.2f | target=(%.2f,%.2f) | steer=%.3f | "
            "speed=%.2f | forward=%.2f | rightFront=%.2f",
            chase_direction_ < 0 ? "RIGHT" : "LEFT", elapsed,
            degrees(chase_yaw_change_), segment.valid ? "yes" : "no",
            degrees(segment.turn), degrees(segment.alpha), segment.distance,
            segment.length, chase_track_count_, degrees(wall.alpha),
            wall.valid ? 1 : 0, wall.discontinuity ? 1 : 0,
            chase_wall_count_, chase_reacquire_scans_,
            corner_.x, target.x, target.y, steer, last_speed_command_,
            forward_min, right_front_known ? right_front_min : -1.0);
    }

    void onScan(const sensor_msgs::msg::LaserScan& scan)
    {
        const rclcpp::Time stamp(scan.header.stamp);
        advanceCornerEstimate(stamp);

        double forward_min = 0.0;
        const bool forward_known = sectorMinimum(
            scan, -10.0, +10.0, 2.0, forward_min);
        if (!forward_known || forward_min < forward_stop_distance_) {
            drive(scan, 0.0, 0.0);
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                "Forward LiDAR unknown or collision clearance violated; stopping");
            return;
        }

        const WallEstimate wall = findRightWall(scan);
        if (wall.b_idx >= 0) {
            double b = 0.0;
            if (rangeAt(scan, wall.b_idx, b))
                beamMarker(scan, wall.b_idx, b, false, beam_b_pub_);
        }
        if (wall.a_idx >= 0 && wall.valid)
            beamMarker(scan, wall.a_idx, wall.a, true, beam_a_pub_);

        if (state_ == State::CHASE_WALL) {
            chase(scan, wall, stamp, forward_min, forward_known);
            return;
        }

        // A drop in front clearance with a navigable right-hand sector
        // suggests a right turn; otherwise a clear left sector suggests a
        // left-going inward corner. These are only geometric heuristics.
        if (forward_min < turn_distance_) {
            double right_min = 0.0;
            const bool right_known = sectorMinimum(scan, -65.0, -35.0, 5.0, right_min);
            if (right_known && right_min > 1.10) {
                startChasing(scan, wall, -1, "forward wall; right-hand turn available");
                chase(scan, wall, stamp, forward_min, forward_known);
                return;
            }
            double left_min = 0.0;
            const bool left_known = sectorMinimum(scan, 25.0, 55.0, 5.0, left_min);
            if (left_known && left_min > 0.75) {
                startChasing(scan, wall, +1, "forward wall; left-hand bend");
                chase(scan, wall, stamp, forward_min, forward_known);
                return;
            }
            drive(scan, 0.0, 0.0);
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                "Blocked ahead; no verified turning direction; stopping");
            return;
        }

        // If the connected wall ends ahead, do not feed a false line into PID.
        // Intentionally follow the corner in this first CHASE_WALL experiment.
        if (wall.discontinuity && wall.outward_gap) {
            startChasing(scan, wall, -1, "outward discontinuity in right wall");
            chase(scan, wall, stamp, forward_min, forward_known);
            return;
        }
        if (!wall.valid) {
            // Enter chase when a previously acquired reference disappears.
            // From initial ALIGN_WALL, stopping is safer than chasing blindly.
            if (state_ == State::FOLLOW_WALL && forward_min > turn_distance_) {
                startChasing(scan, wall, -1, "right wall no longer measurable");
                chase(scan, wall, stamp, forward_min, forward_known);
            } else {
                aligned_scans_ = 0;
                drive(scan, 0.0, 0.0);
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                    "ALIGN_WALL: cannot establish a continuous right wall; stopping");
            }
            return;
        }
        if (wall.discontinuity || wall.line_rms > chase_line_rms_limit_) {
            const int direction = wall.inward_gap ? +1 : -1;
            startChasing(scan, wall, direction, "wall segment changed orientation");
            chase(scan, wall, stamp, forward_min, forward_known);
            return;
        }

        const double alpha_deg = degrees(wall.alpha);
        if (state_ == State::ALIGN_WALL) {
            const double steer = -align_kp_ * wall.alpha;
            aligned_scans_ = (std::abs(alpha_deg) <= align_angle_tolerance_deg_)
                ? aligned_scans_ + 1 : 0;
            if (aligned_scans_ < std::max(1, align_confirm_scans_)) {
                drive(scan, steer, align_speed_);
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
                    "ALIGN_WALL | alpha=%.2f deg | stable=%d/%d",
                    alpha_deg, aligned_scans_, align_confirm_scans_);
                return;
            }
            state_ = State::FOLLOW_WALL;
            resetPid();
            RCLCPP_INFO(get_logger(), "ALIGN_WALL -> FOLLOW_WALL");
        }

        // Keep the tested right-wall PID convention.
        const double error = desired_distance_ - wall.projected;
        double derivative = 0.0;
        if (has_previous_pid_) {
            const double dt = (stamp - previous_pid_stamp_).seconds();
            if (dt > 0.0 && dt < 1.0) {
                derivative = (error - previous_error_) / dt;
                integral_ = std::clamp(integral_ + error * dt, -2.0, 2.0);
            } else {
                integral_ = 0.0;
            }
        }
        previous_error_ = error;
        previous_pid_stamp_ = stamp;
        has_previous_pid_ = true;
        const double steer = kp_ * error + ki_ * integral_ + kd_ * derivative;
        drive(scan, steer, speed_);
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
            "FOLLOW_WALL | alpha=%.2f deg | perpendicular=%.3f | projected=%.3f | "
            "error=%.3f | steer=%.3f",
            alpha_deg, wall.perpendicular, wall.projected, error,
            std::clamp(steer, -max_steering_, max_steering_));
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WallFollower>());
    rclcpp::shutdown();
    return 0;
}
