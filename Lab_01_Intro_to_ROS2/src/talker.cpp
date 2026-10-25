#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
//#include "std_msgs/msg/string.hpp"
#include "ackermann_msgs/msg/ackermann_drive.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

using namespace std::chrono_literals;

class MyNode: public rclcpp::Node
{

	public:

		rcl_interfaces::msg::SetParametersResult talker_param_callback
		(const std::vector<rclcpp::Parameter> &talker_param_request_changes)
		{
			rcl_interfaces::msg::SetParametersResult result;

			result.successful = true;
			//result.reason = "success";

			for(auto &individual_request_change : talker_param_request_changes)
			{
				if(individual_request_change.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE)
				{
					result.successful = false;
					result.reason = "PARAMETERS MUST BE NUMBER VALUES";
				}
				else if(individual_request_change.get_name() == "v")
				{
					velocity_v_value = individual_request_change.as_double();
				}
				else if(individual_request_change.get_name() == "d")
				{
					steering_angle_d_value = individual_request_change.as_double();
				}
			}
			for(auto &individual_request_change : talker_param_request_changes)
			{
				if(result.successful)
				{
					RCLCPP_INFO(this->get_logger(),"%s",
					individual_request_change.get_name().c_str());
					RCLCPP_INFO(this->get_logger(),"%s",
					individual_request_change.get_type_name().c_str());
					RCLCPP_INFO(this->get_logger(),"NEW/UPDATED VALUE: %s",
					individual_request_change.value_to_string().c_str());
				}
			}

			return result;
		}

		MyNode(): rclcpp::Node("TALKER"), count_(0)
		{
			//create a vector filled with the names of all the parameters of this current node
			const std::vector<std::string> talker_param_names{"v","d"};

			//I believe that the v parameter refers to velocity
			this->declare_parameter("v",0.0);

			//I believe that the d parameter refers to steering angle
			this->declare_parameter("d",0.0);

			std::vector<rclcpp::Parameter> talker_params = this->get_parameters(talker_param_names);

			for(auto &individual_param : talker_params)
			{
				RCLCPP_INFO(this->get_logger(),"%s",individual_param.get_name().c_str());
				RCLCPP_INFO(this->get_logger(),"%s",individual_param.get_type_name().c_str());
				RCLCPP_INFO(this->get_logger(),"DEFAULT VALUE: %s",
				individual_param.value_to_string().c_str());
			}

			velocity_v_value = talker_params[0].as_double();
			steering_angle_d_value = talker_params[1].as_double();

			//publisher_ = this->create_publisher<std_msgs::msg::String>("Topic",10 );
			publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("drive",10);

			timer_ = this->create_wall_timer(std::chrono::milliseconds(500),
			bind(&MyNode::talker_timer_callback,this));

			talker_param_callback_handle_ = 
			add_on_set_parameters_callback(
			bind(&MyNode::talker_param_callback,this,std::placeholders::_1)
			);
		}
	private:

		OnSetParametersCallbackHandle::SharedPtr talker_param_callback_handle_;

		rclcpp::TimerBase::SharedPtr timer_;

		//rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
		rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_;

		double velocity_v_value, steering_angle_d_value;

		//publish count. The number of messages that have been published by our node
		size_t count_;

		 void talker_timer_callback()
		{
			//auto message = std_msgs::msg::String();
			//message.data ="Hello, world!!" + std::to_string(count_++);

			//RCLCPP_INFO(this->get_logger(),"%s",message.data.c_str());
			//publisher_->publish(message);

			auto message = ackermann_msgs::msg::AckermannDriveStamped();
			message.drive.speed = static_cast<float>(velocity_v_value);
			message.drive.steering_angle = static_cast<float>(steering_angle_d_value);

			RCLCPP_INFO(this->get_logger(),"VELOCITY: %s", 
			std::to_string(velocity_v_value).c_str());

			RCLCPP_INFO(this->get_logger(),"STEERING_ANGLE: %s", 
			std::to_string(steering_angle_d_value).c_str());

			publisher_ ->publish(message);
		}
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
