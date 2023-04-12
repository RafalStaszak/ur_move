#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>


int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "pose_to_joints",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("move_to_target");

  // Create the MoveIt MoveGroup Interface
using moveit::planning_interface::MoveGroupInterface;
auto move_group_interface = MoveGroupInterface(node, "ur_manipulator"); 

// Set a target Pose
auto const target_pose = []{
  geometry_msgs::msg::Pose msg;
  msg.orientation.w = 1.0;
  msg.position.x = 0.28;
  msg.position.y = -0.2;
  msg.position.z = 0.5;
  return msg;
}();

moveit::core::RobotState robot_state(*move_group_interface.getCurrentState());
const moveit::core::JointModelGroup* joint_model_group = robot_state.getJointModelGroup("ur_manipulator");
const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();
robot_state.setFromIK(joint_model_group, target_pose);

std::vector<double> joint_values;
robot_state.copyJointGroupPositions(joint_model_group, joint_values);
for (std::size_t i = 0; i < joint_names.size(); ++i)
{
  RCLCPP_INFO(logger, "Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
};


// move_group_interface.setPoseTarget(target_pose);

// // Create a plan to that target pose
// auto const [success, plan] = [&move_group_interface]{
//   moveit::planning_interface::MoveGroupInterface::Plan msg;
//   auto const ok = static_cast<bool>(move_group_interface.plan(msg));
//   return std::make_pair(ok, msg);
// }();
// // Execute the plan
// if(success) {
//   move_group_interface.execute(plan);
// } else {
//   RCLCPP_ERROR(logger, "Planing failed!");
// }

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}
