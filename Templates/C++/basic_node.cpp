
#include "rclcpp/rclcpp.hpp"

class MyNode : public rclcpp::Node
{
  public:
    MyNode() : Node("Enter Name of Node Here")
    {
      //Constructor Body Here
    }
  private:
      // class attributes go here
};

int main(int argc, char **argv)
{ 
  //Initiate ROS2 Communications
  rclcpp::init(argc,argv);

  //Allocate memory for the node (the executable)
  //Create shared smart pointer to the newly created MyNode object
  auto node = std::make_shared<MyNode>();

  //run the node
  rclcpp::spin(node);

  //stop ROS2 Communications (Shut Down ROS2)
  rclcpp::shutdown();

  return 0;
}
