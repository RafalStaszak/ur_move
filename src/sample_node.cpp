#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>


int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "sample_node",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Create a ROS logger
auto const logger = rclcpp::get_logger("get_joints");
RCLCPP_INFO(logger, "We are not there yet");
  // Create the MoveIt MoveGroup Interface
using moveit::planning_interface::MoveGroupInterface;
RCLCPP_INFO(logger, "Joint we here");
RCLCPP_INFO(logger, "Joint we are past it");


  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}
