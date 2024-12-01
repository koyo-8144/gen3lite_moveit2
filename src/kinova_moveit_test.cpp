// #include <memory>
// #include <rclcpp/rclcpp.hpp>
// #include <moveit/move_group_interface/move_group_interface.h>

// class KinovaMoveIt
// {
// public:
//   KinovaMoveIt()
//   : node_(std::make_shared<rclcpp::Node>("kinova_moveit", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))),
//     logger_(rclcpp::get_logger("kinova_moveit")),
//     // move_group_interface_(node_, "manipulator")
//     move_group_interface_(node_, "gen3_lite_arm")
//   {}

//   void planAndExecute()
//   {
//     // Set a target Pose
//     auto const target_pose = []{
//       geometry_msgs::msg::Pose msg;
//       msg.orientation.w = 1.0;
//       msg.position.x = 0.28;
//       msg.position.y = 0.2;
//       msg.position.z = -0.5;
//       return msg;
//     }();

//     move_group_interface_.setPoseTarget(target_pose);

//     // Create a plan to that target pose
//     auto const [success, plan] = [&]{
//       moveit::planning_interface::MoveGroupInterface::Plan msg;
//       auto const ok = static_cast<bool>(move_group_interface_.plan(msg));
//       return std::make_pair(ok, msg);
//     }();

//     // Execute the plan
//     if(success) {
//       move_group_interface_.execute(plan);
//     } else {
//       RCLCPP_ERROR(logger_, "Planning failed!");
//     }
//   }

// private:
//   rclcpp::Node::SharedPtr node_;
//   rclcpp::Logger logger_;
//   moveit::planning_interface::MoveGroupInterface move_group_interface_;
// };

// int main(int argc, char * argv[])
// {
//   // Initialize ROS
//   rclcpp::init(argc, argv);

//   // Create an instance of the KinovaMoveIt class and execute the plan
//   KinovaMoveIt kinova_moveit;
//   kinova_moveit.planAndExecute();

//   // Shutdown ROS
//   rclcpp::shutdown();
//   return 0;
// }


#include <memory>
#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>


int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "kinova_moveit_test",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("kinova_moveit_test");

  // Next step goes here
  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "gen3_lite_arm");

  // Set a target Pose
  auto const target_pose = []{
    geometry_msgs::msg::Pose msg;
    msg.orientation.w = 1.0;
    msg.position.x = 0.28;
    msg.position.y = -0.2;
    msg.position.z = 0.1;
    return msg;
  }();
  move_group_interface.setPoseTarget(target_pose);

  std::cout << "Waiting for 10 seconds..." << std::endl;
  move_group_interface.setPlanningTime(10.0);  // set to 10 seconds or more


  // Create a plan to that target pose
  auto const [success, plan] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Execute the plan
  if(success) {
    move_group_interface.execute(plan);
  } else {
    RCLCPP_ERROR(logger, "Planing failed!");
  }

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}
