#include <functional>
#include <memory>
#include <chrono>
#include <string>
#include <cmath>
#include <limits>
#include <cstddef>
#include <algorithm>
#include <vector>

using namespace std::chrono_literals;

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

class SafetyNode : public rclcpp::Node
{
        public:

        SafetyNode() : Node("auto_emergency_brake")
        {

		acker_publisher_ = this->create_publisher
		<ackermann_msgs::msg::AckermannDriveStamped>
		("/drive",10);

                odom_subscription_ = this->create_subscription
                <nav_msgs::msg::Odometry>
                ("/ego_racecar/odom",10,
                std::bind(&SafetyNode::odom_callback,
                this,std::placeholders::_1));

                scan_subscription_ = this->create_subscription
                <sensor_msgs::msg::LaserScan>
                ("/scan",10,
                std::bind(&SafetyNode::scan_callback,
                this,std::placeholders::_1));

        }
        private:

	rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr acker_publisher_;
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
	rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;

	double longitudinal_velocity = 0.0;
	std::vector<float> previous_ranges_;
	rclcpp::Time previous_scan_time_;
	bool have_previous_scan_ = false;

	void scan_callback(const sensor_msgs::msg::LaserScan &msg)
	{
		double iTTC_min = std::numeric_limits<double>::infinity();
		rclcpp::Time current_scan_time(msg.header.stamp);

		bool can_compare_scans =
			have_previous_scan_ &&
			previous_ranges_.size() == msg.ranges.size();

		double delta_time = 0.0;
		if (can_compare_scans)
		{
			delta_time =
				(current_scan_time - previous_scan_time_).seconds();

			if (delta_time <= 0.0)
			{
				can_compare_scans = false;
			}
		}

		for(std::size_t i = 0; i < msg.ranges.size(); i++)
		{
			double range = msg.ranges[i];

			if(can_compare_scans &&
				std::isfinite(range) &&
				range >= msg.range_min &&
				range <= msg.range_max &&
				std::isfinite(previous_ranges_[i]) &&
				previous_ranges_[i] >= msg.range_min &&
				previous_ranges_[i] <= msg.range_max)
			{
				double scan_beam_angle =
					msg.angle_min +
					(static_cast<double>(i) * msg.angle_increment);

				// A negative range rate means the measured distance is
				// shrinking.  This uses what the LiDAR actually observed,
				// so a wall running parallel to the car should have a range
				// rate near zero instead of being treated as a collision.
				double range_rate =
					(range - static_cast<double>(previous_ranges_[i])) /
					delta_time;

				double closing_speed = -range_rate;

				// Ignore increasing or nearly constant ranges.  The small
				// threshold prevents sensor noise from creating a false TTC.
				constexpr double minimum_closing_speed = 0.05;
				if (closing_speed <= minimum_closing_speed)
				{
					continue;
				}

				 double current_iTTC = range / closing_speed;

				// Give the area directly in front of the car the highest
				// priority.  Beams outside this center band are gradually
				// down-weighted so that nearly parallel walls do not dominate
				// the minimum iTTC.
				constexpr double pi = 3.14159265358979323846;
				constexpr double center_half_angle = 2 * pi / 180.0;
				constexpr double lowest_priority_angle = 90.0 * pi / 180.0;
				constexpr double minimum_angle_priority = 0.15;

				double angle_priority = 1.0;
				double absolute_angle = std::fabs(scan_beam_angle);

				if (absolute_angle > center_half_angle)
				{
					double side_progress =
						(absolute_angle - center_half_angle) /
						(lowest_priority_angle - center_half_angle);

					side_progress =
						std::clamp(side_progress, 0.0, 1.0);

					angle_priority =
						1.0 - side_progress *
						(1.0 - minimum_angle_priority);
				}

				// A lower priority makes this obstacle appear farther away
				// for the purpose of the braking decision.  The actual LiDAR
				// range is not changed.
				double weighted_iTTC = current_iTTC / angle_priority;

				if(weighted_iTTC < iTTC_min)
				{
					iTTC_min = weighted_iTTC;
				}

			}
		}

		// Save this scan for the next callback.  The first scan cannot
		// produce a measured range rate, so it will not trigger braking.
		previous_ranges_ = msg.ranges;
		previous_scan_time_ = current_scan_time;
		have_previous_scan_ = true;

		constexpr double brake_start_ttc = 2.0;  // Begin braking
		constexpr double full_brake_ttc  = 0.5;  // Emergency stop threshold

		// Do not overwrite the planner's normal drive command unless a
		// closing obstacle has entered the braking region.
		if (!std::isfinite(iTTC_min) || iTTC_min >= brake_start_ttc)
		{
			return;
		}

		auto message = ackermann_msgs::msg::AckermannDriveStamped();

		// 1.0 at 2 seconds, 0.0 at 0.5 seconds.
		double speed_fraction =
			(iTTC_min - full_brake_ttc) /
			(brake_start_ttc - full_brake_ttc);

		speed_fraction =
			std::clamp(speed_fraction, 0.0, 1.0);

		message.drive.speed =
			longitudinal_velocity * speed_fraction;

		if (iTTC_min <= full_brake_ttc)
		{
    			message.drive.speed = 0.0;

    		// Negative acceleration requests braking.
    		// Use this only if your simulator honors this field.
    		message.drive.acceleration = -3.0;
		}

		acker_publisher_->publish(message);

	}

	void odom_callback(const nav_msgs::msg::Odometry &msg)
	{
		longitudinal_velocity = msg.twist.twist.linear.x;	}
};

int main(int argc, char **argv)
{
        rclcpp::init(argc,argv);

        auto node = std::make_shared<SafetyNode>();

        rclcpp::spin(node);

        rclcpp::shutdown();

        return 0;
}










