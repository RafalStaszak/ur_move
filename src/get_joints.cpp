#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>


int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "get_joints",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  using rclcpp::executors::MultiThreadedExecutor;
MultiThreadedExecutor executor;
// executor.add_node(node);
// executor.add_node(...);
std::thread executor_thread(std::bind(&MultiThreadedExecutor::spin, &executor));

  // Create a ROS logger
auto const logger = rclcpp::get_logger("get_joints");
RCLCPP_INFO(logger, "We are not there yet");
  // Create the MoveIt MoveGroup Interface
using moveit::planning_interface::MoveGroupInterface;
auto move_group = MoveGroupInterface(node, "ur_manipulator"); 

// Set a target Pos

// Get the current joint values
std::vector<double> joint_values;
RCLCPP_INFO(logger, "Joint we here");
const auto jmg_names = move_group.getJointModelGroupNames();

for (std::size_t i = 0; i < jmg_names.size(); ++i)
{
  RCLCPP_INFO(logger, "Joint model %s", jmg_names[i].c_str());
};

auto joint_model_group = move_group.getRobotModel()->getJointModelGroup("ur_manipulator");
RCLCPP_INFO(logger, "Joint we are past it");

const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();
move_group.getCurrentState()->copyJointGroupPositions(joint_model_group, joint_values);

for (std::size_t i = 0; i < joint_names.size(); ++i)
{
  RCLCPP_INFO(logger, "Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
};

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}
