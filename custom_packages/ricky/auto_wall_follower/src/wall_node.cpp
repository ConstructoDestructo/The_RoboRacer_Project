#include <functional>
#include <memory>
#include <string>
#include <cmath>
#include <vector>
#include <map>
#include <algorithm>
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "visualization_msgs/msg/marker.hpp"

class WallFollower : public rclcpp::Node
{
public:

    WallFollower()
        : Node("WALLFOLLOWER"),
          refWallDistance(0.0),
          headingAngleAlpha(0.0),
          wallScansAngle(0.0),
          maxWallScansAngle(70.0),
          lookaheadDistance(0.10),
          actWallDistance(0.0),
          scanA(0.0),
          scanB(0.0),
          kp_(0.5), ki_(0.0), kd_(0.0),
          speed_(0.5), max_steering_angle_(0.34)
    {
        std::map<std::string, double> paraMap =
        {
            {"desiredDistance", 0.75},
            {"maxWallScansAngle", 70.0},
            {"lookaheadDistance", 0.50},
            {"kp", 0.5},
            {"ki", 0.0},
            {"kd", 0.0},
            {"speed", 1.0},
            {"maxSteeringAngle", 0.34},
            {"turnSpeed", 0.30},
            {"turnSteeringAngle", 0.30},
            {"turnInitiationDistance", 1.50},
            {"turnTimeout", 6.0},
            {"wallReacquisitionTolerance", 0.35}
        };

        std::vector<std::string> paramNames;
        paramNames.reserve(paraMap.size());

        for (const auto& pair : paraMap)
        {
            paramNames.push_back("WF." + pair.first);
        }

        this->declare_parameters("WF", paraMap);

        paramVector = get_parameters(paramNames);

        for (const auto& parameter : paramVector)
        {
            if (parameter.get_name() == "WF.desiredDistance")
            {
                refWallDistance = parameter.as_double();
            }
            else if (parameter.get_name() == "WF.maxWallScansAngle")
            {
                maxWallScansAngle = parameter.as_double();
            }
            else if (parameter.get_name() == "WF.lookaheadDistance")
            {
                lookaheadDistance = parameter.as_double();
            }
            else if (parameter.get_name() == "WF.kp") kp_ = parameter.as_double();
            else if (parameter.get_name() == "WF.ki") ki_ = parameter.as_double();
            else if (parameter.get_name() == "WF.kd") kd_ = parameter.as_double();
            else if (parameter.get_name() == "WF.speed") speed_ = parameter.as_double();
            else if (parameter.get_name() == "WF.maxSteeringAngle") max_steering_angle_ = parameter.as_double();
            else if (parameter.get_name() == "WF.turnSpeed") turn_speed_ = parameter.as_double();
            else if (parameter.get_name() == "WF.turnSteeringAngle") turn_steering_angle_ = parameter.as_double();
            else if (parameter.get_name() == "WF.turnInitiationDistance") turn_initiation_distance_ = parameter.as_double();
            else if (parameter.get_name() == "WF.turnTimeout") turn_timeout_ = parameter.as_double();
            else if (parameter.get_name() == "WF.wallReacquisitionTolerance") turn_wall_tolerance_ = parameter.as_double();
        }

        paramSetHandle_ =
            this->add_on_set_parameters_callback(
                std::bind(
                    &WallFollower::paramSetCallback,
                    this,
                    std::placeholders::_1));

        scan_subscription_ =
            this->create_subscription<sensor_msgs::msg::LaserScan>(
                "/scan",
                10,
                std::bind(
                    &WallFollower::scan_callback,
                    this,
                    std::placeholders::_1));

        beam_a_marker_publisher_ =
            this->create_publisher<visualization_msgs::msg::Marker>(
                "/wall_follower/beam_a",
                10);

        drive_publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);

        beam_b_marker_publisher_ =
            this->create_publisher<visualization_msgs::msg::Marker>(
                "/wall_follower/beam_b",
                10);
    }

private:

    double refWallDistance;
    double headingAngleAlpha;
    double wallScansAngle;
    double maxWallScansAngle;
    double lookaheadDistance;
    double actWallDistance;

    double scanA;
    double scanB;

    double kp_, ki_, kd_, speed_, max_steering_angle_;

    // First experiment: right-wall recess handling. All distances in meters.
    enum class FollowState { FOLLOW_WALL, PASS_RECESS, EXECUTE_TURN };
    FollowState follow_state_ = FollowState::FOLLOW_WALL;
    double wall_distance_before_recess_ = 0.0;
    double turn_speed_ = 0.30;
    double turn_steering_angle_ = 0.30;
    double turn_initiation_distance_ = 1.50;
    double turn_timeout_ = 10.0;
    double turn_wall_tolerance_ = 0.35;
    rclcpp::Time turn_start_time_{0, 0, RCL_ROS_TIME};
    int turn_wall_count_ = 0;
    static constexpr int turn_wall_scans_ = 4;
    static constexpr double minimum_turn_seconds_ = 0.65;
    static constexpr double turn_forward_stop_distance_ = 0.35;
    int wall_return_count_ = 0;
    static constexpr double recess_jump_threshold_ = 0.45;
    static constexpr double recess_cartesian_gap_ = 0.45;
    static constexpr double forward_clearance_threshold_ = 1.5;
    static constexpr double recess_speed_ = 0.5;
    static constexpr double wall_return_tolerance_ = 0.25;
    static constexpr int wall_return_scans_ = 4;
    double integral_error_ = 0.0;
    double previous_error_ = 0.0;
    rclcpp::Time previous_time_{0, 0, RCL_ROS_TIME};
    bool has_previous_sample_ = false;

    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_publisher_;

    std::vector<rclcpp::Parameter> paramVector;

    rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr
        paramSetHandle_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
        scan_subscription_;

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
        beam_a_marker_publisher_;

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
        beam_b_marker_publisher_;

    // Estimate a wall line using PCA on consecutive LiDAR points.
    // The returned angle is undirected (modulo pi), and length is the
    // projected visible extent, NOT the physical length of an occluded wall.
    struct LineSegment
    {
        double angle = 0.0;
        double length = 0.0;
        double rms = 0.0;
        int count = 0;
    };

    bool fitLine(const sensor_msgs::msg::LaserScan& scan,
                 int first, int last, LineSegment& result) const
    {
        if (last - first + 1 < 6) return false;
        double sx = 0.0, sy = 0.0, sxx = 0.0, syy = 0.0, sxy = 0.0;
        int count = 0;
        for (int i = first; i <= last; ++i)
        {
            double r = 0.0;
            if (!getRangeAtIndex(scan, i, r)) return false;
            const double a = scan.angle_min + i * scan.angle_increment;
            const double x = r * std::cos(a), y = r * std::sin(a);
            sx += x; sy += y;
            sxx += x*x; syy += y*y; sxy += x*y;
            ++count;
        }
        const double mx = sx/count, my = sy/count;
        const double xx = sxx/count - mx*mx;
        const double yy = syy/count - my*my;
        const double xy = sxy/count - mx*my;
        const double angle = 0.5 * std::atan2(2.0*xy, xx-yy);
        const double ux = std::cos(angle), uy = std::sin(angle);
        const double along = xx*ux*ux + 2*xy*ux*uy + yy*uy*uy;
        const double across = std::max(0.0, xx+yy-along);
        double minProj = 1e30, maxProj = -1e30;
        for (int i = first; i <= last; ++i)
        {
            const double a = scan.angle_min + i*scan.angle_increment;
            const double r = scan.ranges[i];
            const double projection = r*(std::cos(a)*ux + std::sin(a)*uy);
            minProj = std::min(minProj, projection);
            maxProj = std::max(maxProj, projection);
        }
        result = {angle, maxProj-minProj, std::sqrt(across), count};
        return true;
    }

    // Look for a long, straight, near-perpendicular wall beyond the first
    // outward discontinuity. A short back wall or distant parallel wall
    // must not trigger a right turn.
    bool findBranchWall(const sensor_msgs::msg::LaserScan& scan,
                        int beamBIndex, int gapIndex,
                        double& bestLength) const
    {
        bestLength = 0.0;
        LineSegment original;
        if (!fitLine(scan, beamBIndex, gapIndex-1, original) ||
            original.rms > 0.12 || original.length < 0.20)
            return false;

        int last = -1;
        if (!getIndexAtAngle(scan, 10.0, last))
            last = static_cast<int>(scan.ranges.size())-1;
        const int first = gapIndex;
        if (last < first) return false;

        // Segment the post-gap scan by spatial continuity. Unlike the
        // original beam-A search, this continues beyond its 70-degree cap.
        int start = -1, previous = -1;
        auto evaluate = [&](int a, int b)
        {
            LineSegment candidate;
            if (!fitLine(scan, a, b, candidate) || candidate.rms > 0.12)
                return;
            constexpr double pi = 3.14159265358979323846;
            const double delta = std::abs(candidate.angle-original.angle);
            const double acute = std::acos(std::abs(std::cos(delta)));
            if (acute >= 70.0*pi/180.0 &&
                candidate.length > bestLength)
                bestLength = candidate.length;
        };
        for (int i = first; i <= last+1; ++i)
        {
            double r = 0.0;
            const bool valid = i <= last && getRangeAtIndex(scan, i, r);
            bool connected = false;
            if (valid && previous >= 0)
            {
                const double prior = scan.ranges[previous];
                const double da = (i-previous)*scan.angle_increment;
                const double gap2 = prior*prior + r*r - 2*prior*r*std::cos(da);
                connected = (i == previous+1 && gap2 <= 0.35*0.35);
            }
            if (!valid || (previous >= 0 && !connected))
            {
                if (start >= 0) evaluate(start, previous);
                start = -1;
                previous = -1;
            }
            if (valid)
            {
                if (start < 0) start = i;
                previous = i;
            }
        }
        return bestLength >= 2.0;
    }

    bool getIndexAtAngle(
        const sensor_msgs::msg::LaserScan& scan,
        double angleDeg,
        int& index) const
    {
        constexpr double pi =
            3.14159265358979323846;

        if (scan.angle_increment <= 0.0)
        {
            return false;
        }

        const double angleRad =
            angleDeg * pi / 180.0;

        const double indexFloat =
            (angleRad - scan.angle_min) /
            scan.angle_increment;

        index =
            static_cast<int>(std::lround(indexFloat));

        if (index < 0 ||
            index >= static_cast<int>(scan.ranges.size()))
        {
            return false;
        }

        return true;
    }

    bool getRangeAtIndex(
        const sensor_msgs::msg::LaserScan& scan,
        int index,
        double& range) const
    {
        if (index < 0 ||
            index >= static_cast<int>(scan.ranges.size()))
        {
            return false;
        }

        const double measuredRange =
            scan.ranges[index];

        if (!std::isfinite(measuredRange) ||
            measuredRange < scan.range_min ||
            measuredRange > scan.range_max)
        {
            return false;
        }

        range = measuredRange;
        return true;
    }

    void publishBeamMarker(
        const sensor_msgs::msg::LaserScan& scan,
        int beamIndex,
        double range,
        const std_msgs::msg::ColorRGBA& color,
        const rclcpp::Publisher<
            visualization_msgs::msg::Marker>::SharedPtr& publisher)
    {
        visualization_msgs::msg::Marker marker;

        marker.header = scan.header;
        marker.ns = "wall_follower_beams";
        marker.id = 0;
        marker.type =
            visualization_msgs::msg::Marker::ARROW;
        marker.action =
            visualization_msgs::msg::Marker::ADD;

        geometry_msgs::msg::Point startPoint;

        startPoint.x = 0.0;
        startPoint.y = 0.0;
        startPoint.z = 0.0;

        geometry_msgs::msg::Point endPoint;

        const double angle =
            scan.angle_min +
            beamIndex * scan.angle_increment;

        endPoint.x = range * std::cos(angle);
        endPoint.y = range * std::sin(angle);
        endPoint.z = 0.0;

        marker.points.push_back(startPoint);
        marker.points.push_back(endPoint);

        marker.scale.x = 0.01;
        marker.scale.y = 0.03;
        marker.scale.z = 0.03;

        marker.color = color;

        marker.lifetime =
            rclcpp::Duration::from_seconds(0.2);

        publisher->publish(marker);
    }

    rcl_interfaces::msg::SetParametersResult
    paramSetCallback(
        const std::vector<rclcpp::Parameter>& setParamVector)
    {
        rcl_interfaces::msg::SetParametersResult result;

        result.successful = true;
        result.reason =
            "Parameters updated successfully";

        for (const auto& parameter : setParamVector)
        {
            if (parameter.get_name() ==
                "WF.desiredDistance")
            {
                if (parameter.get_type_name() != "double")
                {
                    result.successful = false;
                    result.reason =
                        "Desired distance must be a double";
                }
                else if (parameter.as_double() < 0.0)
                {
                    result.successful = false;
                    result.reason =
                        "Desired distance cannot be negative";
                }
            }

            if (parameter.get_name() ==
                "WF.maxWallScansAngle")
            {
                if (parameter.get_type_name() != "double")
                {
                    result.successful = false;
                    result.reason =
                        "Maximum wall scan angle must be a double";
                }
                else if (parameter.as_double() <= 0.0 ||
                         parameter.as_double() > 70.0)
                {
                    result.successful = false;
                    result.reason =
                        "Maximum wall scan angle must be between 0 and 70 degrees";
                }
            }

            if (parameter.get_name() ==
                "WF.lookaheadDistance")
            {
                if (parameter.get_type_name() != "double")
                {
                    result.successful = false;
                    result.reason =
                        "Lookahead distance must be a double";
                }
                else if (parameter.as_double() <= 0.0)
                {
                    result.successful = false;
                    result.reason =
                        "Lookahead distance must be greater than zero";
                }
            }
            if (parameter.get_name() == "WF.kp" ||
                parameter.get_name() == "WF.ki" ||
                parameter.get_name() == "WF.kd" ||
                parameter.get_name() == "WF.speed" ||
                parameter.get_name() == "WF.maxSteeringAngle" ||
                parameter.get_name() == "WF.turnSpeed" ||
                parameter.get_name() == "WF.turnSteeringAngle" ||
                parameter.get_name() == "WF.turnInitiationDistance" ||
                parameter.get_name() == "WF.turnTimeout" ||
                parameter.get_name() == "WF.wallReacquisitionTolerance")
            {
                if (parameter.get_type_name() != "double" ||
                    !std::isfinite(parameter.as_double()) ||
                    parameter.as_double() < 0.0 ||
                    ((parameter.get_name() == "WF.maxSteeringAngle" ||
                      parameter.get_name() == "WF.turnSteeringAngle" ||
                      parameter.get_name() == "WF.turnInitiationDistance" ||
                      parameter.get_name() == "WF.turnTimeout" ||
                      parameter.get_name() == "WF.wallReacquisitionTolerance") && parameter.as_double() == 0.0))
                {
                    result.successful = false;
                    result.reason = "PID gains, speed, and steering limit must be finite nonnegative doubles; steering limit must be positive";
                }
            }
        }

        if (result.successful)
        {
            for (const auto& parameter : setParamVector)
            {
                if (parameter.get_name() ==
                    "WF.desiredDistance")
                {
                    refWallDistance =
                        parameter.as_double();
                }
                else if (parameter.get_name() ==
                         "WF.maxWallScansAngle")
                {
                    maxWallScansAngle =
                        parameter.as_double();
                }
                else if (parameter.get_name() ==
                         "WF.lookaheadDistance")
                {
                    lookaheadDistance =
                        parameter.as_double();
                }
                else if (parameter.get_name() == "WF.kp") kp_ = parameter.as_double();
                else if (parameter.get_name() == "WF.ki") ki_ = parameter.as_double();
                else if (parameter.get_name() == "WF.kd") kd_ = parameter.as_double();
                else if (parameter.get_name() == "WF.speed") speed_ = parameter.as_double();
                else if (parameter.get_name() == "WF.maxSteeringAngle") max_steering_angle_ = parameter.as_double();
                    else if (parameter.get_name() == "WF.turnSpeed") turn_speed_ = parameter.as_double();
                else if (parameter.get_name() == "WF.turnSteeringAngle") turn_steering_angle_ = parameter.as_double();
                else if (parameter.get_name() == "WF.turnInitiationDistance") turn_initiation_distance_ = parameter.as_double();
                else if (parameter.get_name() == "WF.turnTimeout") turn_timeout_ = parameter.as_double();
                else if (parameter.get_name() == "WF.wallReacquisitionTolerance") turn_wall_tolerance_ = parameter.as_double();
            }
            // Avoid derivative spikes after runtime gain/parameter updates.
            has_previous_sample_ = false;
            integral_error_ = 0.0;
        }

        return result;
    }

    void publishDrive(const sensor_msgs::msg::LaserScan& scan,
                      double steering, double speed)
    {
        ackermann_msgs::msg::AckermannDriveStamped command;
        command.header = scan.header;
        command.drive.steering_angle = std::clamp(
            steering, -max_steering_angle_, max_steering_angle_);
        command.drive.speed = std::max(0.0, speed);
        drive_publisher_->publish(command);
    }

    void beginTurn(const rclcpp::Time& stamp)
    {
        follow_state_ = FollowState::EXECUTE_TURN;
        turn_start_time_ = stamp;
        turn_wall_count_ = 0;
        integral_error_ = 0.0;
        has_previous_sample_ = false;
        RCLCPP_WARN(get_logger(), "Entering EXECUTE_TURN");
    }

    void scan_callback(
        const sensor_msgs::msg::LaserScan& msg)
    {
        constexpr double pi =
            3.14159265358979323846;

        constexpr double rangeTolerance =
            0.001;

        const double beamBAngleDeg = -90.0;

        int beamBIndex = 0;

        if (!getIndexAtAngle(msg, beamBAngleDeg, beamBIndex))
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "Could not obtain valid beam B; stopping");
            ackermann_msgs::msg::AckermannDriveStamped stop;
            stop.header = msg.header;
            stop.drive.speed = 0.0;
            drive_publisher_->publish(stop);
            return;
        }

        const bool beamBValid = getRangeAtIndex(msg, beamBIndex, scanB);
        if (!beamBValid && follow_state_ == FollowState::FOLLOW_WALL)
        {
            publishDrive(msg, 0.0, 0.0);
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                "Beam B invalid during wall following; stopping");
            return;
        }

        const double beamBAngleRad =
            msg.angle_min +
            beamBIndex * msg.angle_increment;

        const double searchAngleDeg =
            maxWallScansAngle > 70.0
                ? 70.0
                : maxWallScansAngle;

        const double maximumSearchAngleRad =
            beamBAngleRad +
            searchAngleDeg * pi / 180.0;

        int maximumSearchIndex =
            static_cast<int>(std::floor(
                (maximumSearchAngleRad -
                 msg.angle_min) /
                msg.angle_increment));

        const int finalScanIndex =
            static_cast<int>(
                msg.ranges.size()) - 1;

        if (maximumSearchIndex >
            finalScanIndex)
        {
            maximumSearchIndex =
                finalScanIndex;
        }

        if (maximumSearchIndex <= beamBIndex)
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "No laser beams are available in the search region");

            return;
        }

        // Detect the first outward jump along the right wall, BEFORE
        // accepting beam A for the PID geometry. Compare neighboring
        // scan points in Cartesian space, not only their radial ranges.
        int beamAIndex = -1;
        double largestRange = -1.0;
        bool foundMaximumRange = false;
        bool outwardDiscontinuity = false;
        int firstGapIndex = -1;
        int previousValidIndex = beamBValid ? beamBIndex : -1;

        for (int index = beamBIndex + 1;
             index <= maximumSearchIndex; ++index)
        {
            double candidateRange = 0.0;
            if (!getRangeAtIndex(msg, index, candidateRange))
            {
                // Do not bridge a missing measurement when deciding
                // whether two points belong to the same wall.
                previousValidIndex = -1;
                continue;
            }

            if (previousValidIndex >= 0)
            {
                const double priorRange = msg.ranges[previousValidIndex];
                const double deltaAngle =
                    (index - previousValidIndex) * msg.angle_increment;
                const double gap = std::sqrt(
                    priorRange * priorRange + candidateRange * candidateRange -
                    2.0 * priorRange * candidateRange * std::cos(deltaAngle));

                if (candidateRange - priorRange > recess_jump_threshold_ &&
                    gap > recess_cartesian_gap_)
                {
                    outwardDiscontinuity = true;
                    firstGapIndex = index;
                    break;
                }
            }
            previousValidIndex = index;

            if (candidateRange > largestRange)
            {
                largestRange = candidateRange;
                beamAIndex = index;
            }
            if (candidateRange >= msg.range_max - rangeTolerance)
            {
                foundMaximumRange = true;
                // Keep scanning to check continuity beyond this beam.
            }
        }

        // Evaluate a forward-facing sector. Unknown returns do NOT
        // establish a clear path. Require valid samples throughout.
        bool forwardClear = true;
        bool forwardKnown = true;
        double forwardMinimum = msg.range_max;
        int forwardSamples = 0;
        for (double degrees = -10.0; degrees <= 10.0; degrees += 2.0)
        {
            int index = -1;
            double distance = 0.0;
            if (!getIndexAtAngle(msg, degrees, index) ||
                !getRangeAtIndex(msg, index, distance))
            {
                forwardClear = false;
                forwardKnown = false;
                break;
            }
            ++forwardSamples;
            forwardMinimum = std::min(forwardMinimum, distance);
            if (distance < forward_clearance_threshold_)
            {
                forwardClear = false;
            }
        }
        forwardKnown = forwardKnown && forwardSamples == 11;
        forwardClear = forwardKnown && forwardClear;

        // Check the front-right opening, not merely the side-facing beam.
        // All sampled rays must show a navigable gap for this conservative test.
        bool rightOpen = true;
        int rightSamples = 0;
        for (double degrees = -65.0; degrees <= -35.0; degrees += 5.0)
        {
            int index = -1;
            double distance = 0.0;
            if (!getIndexAtAngle(msg, degrees, index) ||
                !getRangeAtIndex(msg, index, distance))
            {
                rightOpen = false;
                break;
            }
            ++rightSamples;
            if (distance < 1.10) rightOpen = false;
        }
        rightOpen = rightOpen && rightSamples == 7;
        const bool forwardBlocked = forwardKnown &&
            forwardMinimum < turn_initiation_distance_;
        const bool turnAvailable = forwardBlocked && rightOpen;
        // Case 3: forward clear, right opening, and a long wall extending
        // approximately perpendicular to the original right wall.
        // This geometric test is deliberately stricter than rightOpen alone.
        double branchWallLength = 0.0;
        const bool case3Turn =
            outwardDiscontinuity && forwardClear && rightOpen &&
            findBranchWall(msg, beamBIndex, firstGapIndex, branchWallLength);
        if (outwardDiscontinuity && forwardClear)
        {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
                "Case 3 geometry | perpendicular visible wall=%.2f m | branch=%s",
                branchWallLength, case3Turn ? "yes" : "no");
        }
        const rclcpp::Time scanTime(msg.header.stamp);


        // Beam B can be invalid while passing the opening; the state
        // machine still publishes a conservative command below.
        // A forward obstruction can trigger a turn even without a gap.
        // Re-evaluate continuously while passing a suspected recess.
        if (follow_state_ != FollowState::EXECUTE_TURN &&
            (turnAvailable || case3Turn))
        {
            RCLCPP_WARN(get_logger(), "Right turn: %s",
                turnAvailable ? "Case 2 (forward blocked)" :
                                "Case 3 (long perpendicular wall)");
            beginTurn(scanTime);
        }

        if (follow_state_ == FollowState::EXECUTE_TURN)
        {
            const double elapsed = (scanTime - turn_start_time_).seconds();
            if (elapsed < 0.0 || elapsed > turn_timeout_ || !forwardKnown)
            {
                publishDrive(msg, 0.0, 0.0);
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                    "Turn timed out or forward LiDAR unknown; stopping");
                return;
            }

            // The new right wall must be close to the desired distance,
            // and both selected beams must belong to one connected segment.
            bool consistentWall = false;
            if (beamBValid && !outwardDiscontinuity && beamAIndex >= 0)
            {
                double candidateA = 0.0;
                if (getRangeAtIndex(msg, beamAIndex, candidateA))
                {
                    const double theta =
                        (beamAIndex - beamBIndex) * msg.angle_increment;
                    const double alpha = std::atan2(
                        candidateA * std::cos(theta) - scanB,
                        candidateA * std::sin(theta));
                    const double perpendicular = scanB * std::cos(alpha);
                    consistentWall = std::abs(alpha) < 0.44 &&
                        std::abs(perpendicular - refWallDistance) <
                            turn_wall_tolerance_;
                }
            }
            if (elapsed >= minimum_turn_seconds_ && forwardClear && consistentWall)
                ++turn_wall_count_;
            else
                turn_wall_count_ = 0;

            if (turn_wall_count_ < turn_wall_scans_)
            {
                // Keep checking for an imminent collision during the turn.
                if (forwardMinimum < turn_forward_stop_distance_)
                {
                    publishDrive(msg, 0.0, 0.0);
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                        "Turn blocked at close range; stopping");
                }
                else
                {
                    publishDrive(msg, -turn_steering_angle_,
                                 std::min(speed_, turn_speed_));
                }
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
                    "EXECUTE_TURN | elapsed=%.2f | forward=%.2f | wall=%d/%d",
                    elapsed, forwardMinimum, turn_wall_count_, turn_wall_scans_);
                return;
            }
            follow_state_ = FollowState::FOLLOW_WALL;
            integral_error_ = 0.0;
            has_previous_sample_ = false;
            RCLCPP_INFO(get_logger(), "New right wall acquired; resuming PID");
        }

        if (follow_state_ == FollowState::FOLLOW_WALL &&
            beamBValid && outwardDiscontinuity && forwardClear)
        {
            follow_state_ = FollowState::PASS_RECESS;
            wall_distance_before_recess_ = scanB;
            wall_return_count_ = 0;
            integral_error_ = 0.0;
            has_previous_sample_ = false;
            RCLCPP_WARN(this->get_logger(),
                "Entering PASS_RECESS: gap at beam %d, prior wall %.2f m",
                firstGapIndex, wall_distance_before_recess_);
        }

        if (follow_state_ == FollowState::PASS_RECESS)
        {
            // Only reacquire after the side wall returns near the
            // pre-recess distance AND no forward-side jump is present.
            const bool wallReturned =
                beamBValid &&
                std::abs(scanB - wall_distance_before_recess_) <
                    wall_return_tolerance_ &&
                !outwardDiscontinuity;
            wall_return_count_ = wallReturned ? wall_return_count_ + 1 : 0;

            if (wall_return_count_ < wall_return_scans_)
            {
                ackermann_msgs::msg::AckermannDriveStamped drive_msg;
                drive_msg.header = msg.header;
                drive_msg.drive.steering_angle = 0.0;
                drive_msg.drive.speed = forwardClear ?
                    std::min(speed_, recess_speed_) : 0.0;
                drive_publisher_->publish(drive_msg);
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(),
                    500, "PASS_RECESS | forward=%s | return=%d/%d | speed=%.2f",
                    forwardClear ? "clear" : "blocked/unknown",
                    wall_return_count_, wall_return_scans_, drive_msg.drive.speed);
                return;
            }

            follow_state_ = FollowState::FOLLOW_WALL;
            integral_error_ = 0.0;
            has_previous_sample_ = false;
            RCLCPP_INFO(this->get_logger(), "Wall reacquired; resuming PID");
        }

        // If the scan is discontinuous but forward is blocked/unknown,
        // do not let the original PID steer into an unverified opening.
        if (outwardDiscontinuity && !forwardClear)
        {
            ackermann_msgs::msg::AckermannDriveStamped drive_msg;
            drive_msg.header = msg.header;
            drive_msg.drive.steering_angle = 0.0;
            drive_msg.drive.speed = 0.0;
            drive_publisher_->publish(drive_msg);
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(),
                1000, "Wall discontinuity but forward clearance uncertain: STOP");
            return;
        }

        // Emergency clearance guard for unclassified obstacles.
        if (forwardKnown && forwardMinimum < turn_forward_stop_distance_)
        {
            publishDrive(msg, 0.0, 0.0);
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                "Forward obstacle too close; stopping");
            return;
        }

        if (beamAIndex < 0 || !getRangeAtIndex(msg, beamAIndex, scanA))
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(),
                1000, "No valid connected beam A; stopping");
            ackermann_msgs::msg::AckermannDriveStamped stop;
            stop.header = msg.header;
            stop.drive.speed = 0.0;
            drive_publisher_->publish(stop);
            return;
        }

        const double beamAAngleRad =
            msg.angle_min + beamAIndex * msg.angle_increment;
        wallScansAngle =
            (beamAAngleRad - beamBAngleRad) * 180.0 / pi;

        // Ideal-case assumption: beams A and B hit the same
        // continuous, approximately straight right-side wall.
        // The angle between the beams must be expressed in radians.
        const double theta = wallScansAngle * pi / 180.0;

        // Estimate the heading of the vehicle relative to the wall.
        // atan2 handles the numerator and denominator without division.
        headingAngleAlpha = std::atan2(
            scanA * std::cos(theta) - scanB,
            scanA * std::sin(theta));

        // Perpendicular distance from the LiDAR to the right wall.
        actWallDistance = scanB * std::cos(headingAngleAlpha);

        // Estimate wall distance at a point ahead of the vehicle.
        const double projectedWallDistance =
            actWallDistance +
            lookaheadDistance * std::sin(headingAngleAlpha);

        // Right-wall error: negative means too far from the right wall.
        const double wallDistanceError =
            refWallDistance - projectedWallDistance;

        // Discrete-time PID. Use scan timestamps for elapsed time.
        const rclcpp::Time sample_time(msg.header.stamp);
        double dt = 0.0;
        if (has_previous_sample_)
        {
            dt = (sample_time - previous_time_).seconds();
        }
        double derivative_error = 0.0;
        if (has_previous_sample_ && dt > 0.0 && dt < 1.0)
        {
            derivative_error = (wallDistanceError - previous_error_) / dt;
            integral_error_ += wallDistanceError * dt;
            // Simple anti-windup bound on the accumulated error.
            integral_error_ = std::clamp(integral_error_, -2.0, 2.0);
        }
        else if (has_previous_sample_)
        {
            integral_error_ = 0.0;
        }
        previous_error_ = wallDistanceError;
        previous_time_ = sample_time;
        has_previous_sample_ = true;

        const double steering_unclamped =
            kp_ * wallDistanceError +
            ki_ * integral_error_ +
            kd_ * derivative_error;
        const double steering = std::clamp(
            steering_unclamped, -max_steering_angle_, max_steering_angle_);

        ackermann_msgs::msg::AckermannDriveStamped drive_msg;
        drive_msg.header.stamp = msg.header.stamp;
        drive_msg.header.frame_id = msg.header.frame_id;
        drive_msg.drive.steering_angle = steering; // radians; positive = left
        drive_msg.drive.speed = speed_; // m/s
        drive_publisher_->publish(drive_msg);

        RCLCPP_INFO(
            this->get_logger(),
            "scanA: %.3f m | scanB: %.3f m | "
            "wallScansAngle: %.3f deg | "
            "headingAngleAlpha: %.3f deg | "
            "actWallDistance: %.3f m | "
            "projectedWallDistance: %.3f m | "
            "wallDistanceError: %.3f m | steering: %.3f rad%s",
            scanA,
            scanB,
            wallScansAngle,
            headingAngleAlpha * 180.0 / pi,
            actWallDistance,
            projectedWallDistance,
            wallDistanceError,
            steering,
            foundMaximumRange
                ? " | max range found"
                : " | largest valid range selected");

        // Green marker for beam A.
        std_msgs::msg::ColorRGBA beamAColor;

        beamAColor.r = 0.0;
        beamAColor.g = 1.0;
        beamAColor.b = 0.0;
        beamAColor.a = 1.0;

        // Red marker for beam B.
        std_msgs::msg::ColorRGBA beamBColor;

        beamBColor.r = 1.0;
        beamBColor.g = 0.0;
        beamBColor.b = 0.0;
        beamBColor.a = 1.0;

        publishBeamMarker(
            msg,
            beamAIndex,
            scanA,
            beamAColor,
            beam_a_marker_publisher_);

        publishBeamMarker(
            msg,
            beamBIndex,
            scanB,
            beamBColor,
            beam_b_marker_publisher_);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node =
        std::make_shared<WallFollower>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}












































































































































































































































