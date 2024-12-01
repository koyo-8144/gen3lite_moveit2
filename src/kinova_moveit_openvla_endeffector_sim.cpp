#include <memory>
#include <vector>
#include <sstream>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/srv/get_position_fk.hpp>

class KinovaMoveItOpenvlaSim
{
public:
  KinovaMoveItOpenvlaSim()
      : node_(std::make_shared<rclcpp::Node>("kinova_moveit_openvla_sim", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))),
        logger_(node_->get_logger()),
        move_group_arm(node_, PLANNING_GROUP_ARM),
        move_group_gripper(node_, PLANNING_GROUP_GRIPPER),
        callback_count_(0),
        action_data_received_(false),
        joint_data_received_(false)
  {
    callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    timer_cb_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // Subscriptions
    rclcpp::SubscriptionOptions options_;
    options_.callback_group = callback_group_;

    subscription_ = node_->create_subscription<std_msgs::msg::Float32MultiArray>(
        "openvla_action_dummy", 10,
        std::bind(&KinovaMoveItOpenvlaSim::action_callback, this, std::placeholders::_1),
        options_);

    joint_subscription_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", 10,
        std::bind(&KinovaMoveItOpenvlaSim::joint_callback, this, std::placeholders::_1),
        options_);

    fk_client_ = node_->create_client<moveit_msgs::srv::GetPositionFK>(
        "compute_fk", rmw_qos_profile_services_default, callback_group_);

    timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(200),
        std::bind(&KinovaMoveItOpenvlaSim::timer_callback, this),
        timer_cb_group_);

    // Log available planning groups
    RCLCPP_INFO(logger_, "Planning frame: %s",
                move_group_arm.getPlanningFrame().c_str());
    RCLCPP_INFO(logger_, "End effector link: %s",
                move_group_arm.getEndEffectorLink().c_str());
    RCLCPP_INFO(logger_, "Available Planning Groups:");
    for (const auto &group : move_group_arm.getJointModelGroupNames())
      RCLCPP_INFO(logger_, " - %s", group.c_str());
  }

  // Getter for the node pointer
  rclcpp::Node::SharedPtr get_node() { return node_; }

private:
  static const std::string PLANNING_GROUP_ARM;
  static const std::string PLANNING_GROUP_GRIPPER;

  void action_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    callback_count_++;
    action_data_ = msg->data;
    action_data_received_ = true;

    std::ostringstream oss;
    for (size_t i = 0; i < action_data_.size(); ++i)
      oss << action_data_[i] << (i < action_data_.size() - 1 ? ", " : "");

    RCLCPP_INFO(logger_, "Received action data: [%s], #%d", oss.str().c_str(), callback_count_);
  }

  void joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    joint_positions_ = msg->position;
    joint_data_received_ = true;
  }

  void timer_callback()
  {
    // Get the JointModelGroup pointers
    const moveit::core::JointModelGroup *joint_model_group_arm =
        move_group_arm.getCurrentState()->getJointModelGroup(PLANNING_GROUP_ARM);
    const moveit::core::JointModelGroup *joint_model_group_gripper =
        move_group_gripper.getCurrentState()->getJointModelGroup(PLANNING_GROUP_GRIPPER);

    // Sleep for 2 seconds to ensure state update
    rclcpp::sleep_for(std::chrono::seconds(2));

    // Get the current state of the robot
    moveit::core::RobotStatePtr current_state_arm = move_group_arm.getCurrentState(10);
    moveit::core::RobotStatePtr current_state_gripper = move_group_gripper.getCurrentState(10);

    // Copy joint positions
    std::vector<double> joint_group_positions_arm;
    std::vector<double> joint_group_positions_gripper;
    current_state_arm->copyJointGroupPositions(joint_model_group_arm, joint_group_positions_arm);
    current_state_gripper->copyJointGroupPositions(joint_model_group_gripper, joint_group_positions_gripper);

    // Log joint positions
    RCLCPP_INFO(logger_, "Joint Positions for Arm:");
    for (size_t i = 0; i < joint_group_positions_arm.size(); ++i)
    {
        RCLCPP_INFO(logger_, "  Joint %lu: %f", i, joint_group_positions_arm[i]);
    }

    RCLCPP_INFO(logger_, "Joint Positions for Gripper:");
    for (size_t i = 0; i < joint_group_positions_gripper.size(); ++i)
    {
        RCLCPP_INFO(logger_, "  Joint %lu: %f", i, joint_group_positions_gripper[i]);
    }

    // Set the start state to the current state
    move_group_arm.setStartStateToCurrentState();
    move_group_gripper.setStartStateToCurrentState();

    // Get the current pose of the end effector
    geometry_msgs::msg::Pose current_pose = move_group_arm.getCurrentPose().pose;

    // Log the current position of the end effector
    RCLCPP_INFO(logger_, "Current End-Effector Position:");
    RCLCPP_INFO(logger_, "  x: %f, y: %f, z: %f", 
                current_pose.position.x, 
                current_pose.position.y, 
                current_pose.position.z);

    // Log the current orientation of the end effector (quaternion)
    RCLCPP_INFO(logger_, "Current End-Effector Orientation (Quaternion):");
    RCLCPP_INFO(logger_, "  x: %f, y: %f, z: %f, w: %f", 
                current_pose.orientation.x, 
                current_pose.orientation.y, 
                current_pose.orientation.z, 
                current_pose.orientation.w);
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger logger_;
  rclcpp::CallbackGroup::SharedPtr callback_group_, timer_cb_group_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscription_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_subscription_;
  rclcpp::Client<moveit_msgs::srv::GetPositionFK>::SharedPtr fk_client_;
  rclcpp::TimerBase::SharedPtr timer_;

  moveit::planning_interface::MoveGroupInterface move_group_arm;
  moveit::planning_interface::MoveGroupInterface move_group_gripper;

  std::vector<float> action_data_;
  std::vector<double> joint_positions_;
  std::vector<double> end_effector_position_;

  int callback_count_;
  bool action_data_received_, joint_data_received_;
  
};

const std::string KinovaMoveItOpenvlaSim::PLANNING_GROUP_ARM = "manipulator";
const std::string KinovaMoveItOpenvlaSim::PLANNING_GROUP_GRIPPER = "gripper";

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<KinovaMoveItOpenvlaSim>();
  rclcpp::spin(node->get_node());
  rclcpp::shutdown();
  return 0;
}
