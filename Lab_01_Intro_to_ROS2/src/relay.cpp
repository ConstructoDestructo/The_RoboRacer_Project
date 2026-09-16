#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
//#include "std_msgs/msg/string.hpp"
#include "ackermann_msgs/msg/ackermann_drive.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

//using std::placeholders::_1;

class MyNode: public rclcpp::Node
{

	public:

		MyNode(): rclcpp::Node("RELAY")
		{
			//subscription_ = this->create_subscription<std_msgs::msg::String>("Topic",10,
			//std::bind(&MyNode::relay_Topic_callback,this,std::placeholders::_1));

			subscription_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
			"drive",10,std::bind(&MyNode::relay_drive_callback,this,std::placeholders::_1));

			publisher_ = this -> create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
			"drive_relay",10);

			timer_ = this->create_wall_timer(std::chrono::milliseconds(500),
			std::bind(&MyNode::relay_timer_callback,this));
		}
	private:

		//void relay_Topic_callback(const std_msgs::msg::String &msg) const
		//{
		//	RCLCPP_INFO(this->get_logger(),"I heard: %s",msg.data.c_str());
		//}

		//rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;

		void relay_drive_callback(const ackermann_msgs::msg::AckermannDriveStamped &msg)
		{
			RCLCPP_INFO(this->get_logger(),"I heard VELOCITY: %s",std::to_string(msg.drive.speed).c_str());
			RCLCPP_INFO(this->get_logger(),"I heard STEERING_ANGLE: %s",std::to_string(msg.drive.steering_angle).c_str());

			velocity_v_3x = static_cast<float>(msg.drive.speed) * 3.0;
			steering_angle_d_3x = static_cast<float>(msg.drive.steering_angle) * 3.0;

		}

		void relay_timer_callback()
		{
			auto message = ackermann_msgs::msg::AckermannDriveStamped();
			message.drive.speed = velocity_v_3x;
			message.drive.steering_angle = steering_angle_d_3x;

			publisher_ -> publish(message);
		}

		float velocity_v_3x,steering_angle_d_3x;

		rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr subscription_;
		rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_;
		rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
	//remember to initiate ROS2 communications
	rclcpp::init(argc,argv);

	//It is now time to create the actual node
	auto talker_node = std::make_shared<MyNode>();

	//Now we have to spin the node so that it can be responsive to input/stimuli
	rclcpp::spin(talker_node);

	rclcpp::shutdown();

	return 0;
}
