#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");

int main(int argc, char **argv) {
  // Initialize ROS 2
  rclcpp::init(argc, argv);

  // Create node with parameters auto-declared
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node =
      rclcpp::Node::make_shared("move_group_interface_tutorial", node_options);

  // Use a single-threaded executor
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  // Define planning groups
  static const std::string PLANNING_GROUP_ARM = "manipulator";
  static const std::string PLANNING_GROUP_GRIPPER = "gripper";

  // Create MoveGroupInterface for the arm and gripper
  moveit::planning_interface::MoveGroupInterface move_group_arm(
      move_group_node, PLANNING_GROUP_ARM);
  moveit::planning_interface::MoveGroupInterface move_group_gripper(
      move_group_node, PLANNING_GROUP_GRIPPER);

  // Get the JointModelGroup pointers
  const moveit::core::JointModelGroup *joint_model_group_arm =
      move_group_arm.getCurrentState()->getJointModelGroup(PLANNING_GROUP_ARM);
  const moveit::core::JointModelGroup *joint_model_group_gripper =
      move_group_gripper.getCurrentState()->getJointModelGroup(
          PLANNING_GROUP_GRIPPER);

  rclcpp::sleep_for(std::chrono::seconds(2));

  // Get the current state of the robot
  moveit::core::RobotStatePtr current_state_arm =
      move_group_arm.getCurrentState(10);
  moveit::core::RobotStatePtr current_state_gripper =
      move_group_gripper.getCurrentState(10);

  std::vector<double> joint_group_positions_arm;
  std::vector<double> joint_group_positions_gripper;
  current_state_arm->copyJointGroupPositions(joint_model_group_arm,
                                             joint_group_positions_arm);
  current_state_gripper->copyJointGroupPositions(joint_model_group_gripper,
                                                 joint_group_positions_gripper);

  // Set the start state to the current state (Go Home)
  move_group_arm.setStartStateToCurrentState();
  move_group_gripper.setStartStateToCurrentState();

  // Get the current pose of the end effector
  geometry_msgs::msg::Pose current_pose = move_group_arm.getCurrentPose().pose;

  // Log the current position of the end effector
  RCLCPP_INFO(LOGGER, "Current End-Effector Position:");
  RCLCPP_INFO(LOGGER, "x: %f, y: %f, z: %f", 
              current_pose.position.x, 
              current_pose.position.y, 
              current_pose.position.z);

  // Log the current orientation of the end effector (quaternion)
  RCLCPP_INFO(LOGGER, "Current End-Effector Orientation (Quaternion):");
  RCLCPP_INFO(LOGGER, "x: %f, y: %f, z: %f, w: %f", 
              current_pose.orientation.x, 
              current_pose.orientation.y, 
              current_pose.orientation.z, 
              current_pose.orientation.w);

  // Shut down ROS 2
  rclcpp::shutdown();
  return 0;
}





// #include <moveit/move_group_interface/move_group_interface.h>
// #include <moveit/planning_scene_interface/planning_scene_interface.h>

// #include <moveit_msgs/msg/display_robot_state.hpp>
// #include <moveit_msgs/msg/display_trajectory.hpp>

// static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");

// int main(int argc, char **argv) {
//   rclcpp::init(argc, argv);
//   rclcpp::NodeOptions node_options;
//   node_options.automatically_declare_parameters_from_overrides(true);
//   auto move_group_node =
//       rclcpp::Node::make_shared("move_group_interface_tutorial", node_options);

//   rclcpp::executors::SingleThreadedExecutor executor;
//   executor.add_node(move_group_node);
//   std::thread([&executor]() { executor.spin(); }).detach();

//   static const std::string PLANNING_GROUP = "manipulator";

//   moveit::planning_interface::MoveGroupInterface move_group(move_group_node,
//                                                             PLANNING_GROUP);

//   const moveit::core::JointModelGroup *joint_model_group =
//       move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

//   RCLCPP_INFO(LOGGER, "Planning frame: %s",
//               move_group.getPlanningFrame().c_str());

//   RCLCPP_INFO(LOGGER, "End effector link: %s",
//               move_group.getEndEffectorLink().c_str());

//   RCLCPP_INFO(LOGGER, "Available Planning Groups:");
//   std::copy(move_group.getJointModelGroupNames().begin(),
//             move_group.getJointModelGroupNames().end(),
//             std::ostream_iterator<std::string>(std::cout, ", "));

//   geometry_msgs::msg::Pose target_pose1;
//   target_pose1.orientation.x = -1.0;
//   target_pose1.orientation.y = 0.00;
//   target_pose1.orientation.z = 0.00;
//   target_pose1.orientation.w = 0.00;
//   target_pose1.position.x = 0.343;
//   target_pose1.position.y = 0.132;
//   target_pose1.position.z = 0.284;
//   move_group.setPoseTarget(target_pose1);

//   moveit::planning_interface::MoveGroupInterface::Plan my_plan;

//   bool success =
//       (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

//   move_group.execute(my_plan);

//   rclcpp::shutdown();
//   return 0;
// }