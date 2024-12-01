#include <memory>
#include <vector>
#include <sstream>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/srv/get_position_fk.hpp>
#include <moveit_msgs/srv/get_position_ik.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

class KinovaNoMoveItOpenvla
{
public:
  KinovaNoMoveItOpenvla()
  : node_(std::make_shared<rclcpp::Node>("kinova_no_moveit_openvla_", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))),
    logger_(node_->get_logger()),
    move_group_interface_(node_, "gen3_lite_arm"),
    callback_count_(0),
    joint_data_received_(false)
  {

    callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    timer_cb_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // Subscriptions
    rclcpp::SubscriptionOptions options_;
    options_.callback_group = callback_group_;

    // Subscriptions
    subscription_ = node_->create_subscription<std_msgs::msg::Float32MultiArray>(
        "openvla_action", 10, 
        std::bind(&KinovaNoMoveItOpenvla::action_callback, this, std::placeholders::_1), 
        options_);

    joint_subscription_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", 10, 
        std::bind(&KinovaNoMoveItOpenvla::joint_callback, this, std::placeholders::_1), 
        options_);

    fk_client_ = node_->create_client<moveit_msgs::srv::GetPositionFK>(
        "compute_fk", rmw_qos_profile_services_default, timer_cb_group_);

    ik_client_ = node_->create_client<moveit_msgs::srv::GetPositionIK>(
        "compute_ik", rmw_qos_profile_services_default, timer_cb_group_);

    trajectory_publisher_ = node_->create_publisher<trajectory_msgs::msg::JointTrajectory>(
        "joint_trajectory_controller/joint_trajectory", rclcpp::QoS(10));

      
    // We print the current planning frame for this group.
    RCLCPP_INFO(logger_, "Planning frame: %s",
                  move_group_interface_.getPlanningFrame().c_str());

    // We can also print the name of the end-effector link for this group.
    RCLCPP_INFO(logger_, "End effector link: %s",
              move_group_interface_.getEndEffectorLink().c_str());

    // We can get a list of all the groups in the robot:
    RCLCPP_INFO(logger_, "Available Planning Groups:");
    std::copy(move_group_interface_.getJointModelGroupNames().begin(),
            move_group_interface_.getJointModelGroupNames().end(),
            std::ostream_iterator<std::string>(std::cout, ", "));

  }

  // Getter for node_ pointer
  rclcpp::Node::SharedPtr get_node() { return node_; }

private:

  // Callback function to handle joint states data and update joint_positions_
  void joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    joint_names_ = msg->name;
    joint_positions_ = msg->position;
    joint_data_received_ = true;  // Mark that joint data has been received

    // // Output joint names and positions
    // std::cout << "joint names: ";
    // for (const auto& name : joint_names_) {
    //     std::cout << name << " ";
    // }
    // std::cout << std::endl;

    // std::cout << "joint positions: ";
    // for (const auto& position : joint_positions_) {
    //     std::cout << position << " ";
    // }
    // std::cout << std::endl;


    // Desired order of joints without "right_finger_bottom_joint"
    std::vector<std::string> desired_order = {
        "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"
    };

    // Create a map to store positions indexed by joint names
    std::unordered_map<std::string, double> name_position_map;
    for (size_t i = 0; i < joint_names_.size(); ++i) {
        name_position_map[joint_names_[i]] = joint_positions_[i];
    }

    // Reorder joint_names_ and joint_positions_ based on desired_order
    std::vector<double> reordered_positions;
    std::vector<std::string> reordered_names;
    for (const auto& name : desired_order) {
        if (name_position_map.find(name) != name_position_map.end()) {
            reordered_positions.push_back(name_position_map[name]);
            reordered_names.push_back(name);
        }
    }

    joint_names_ = reordered_names;      // Update joint_names_
    joint_positions_ = reordered_positions; // Update joint_positions_

    joint_data_received_ = true; // Mark that joint data has been received

    // // Output reordered joint names and positions
    // std::cout << "Reordered joint names: ";
    // for (const auto& name : joint_names_) {
    //     std::cout << name << " ";
    // }
    // std::cout << std::endl;

    // std::cout << "Reordered joint positions: ";
    // for (const auto& position : joint_positions_) {
    //     std::cout << position << " ";
    // }
    // std::cout << std::endl;

  }

  // Callback function to handle incoming action data
  void action_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    callback_count_++;
    action_data_ = msg->data;

    // Log received action data
    std::ostringstream oss;

    oss << "action data: [";
    for (size_t i = 0; i < action_data_.size(); ++i) {
        oss << action_data_[i];
        if (i < action_data_.size() - 1) oss << ", ";
    }
    oss << "]";
    RCLCPP_INFO(logger_, "Subscribing openvla action, %s, #%d", oss.str().c_str(), callback_count_);

    // Clear oss if you need to reuse it
    oss.str("");  // Clear contents
    oss.clear();  // Clear error flag

    if (!joint_data_received_) {
        RCLCPP_WARN(logger_, "No joint data available to process action.");
        return; // Skip processing if joint data isn't available
    }

    RCLCPP_INFO(logger_, "Both action and joint data received.");

    // Process action using the latest joint states
    RCLCPP_INFO(logger_, "Processing action using latest joint states...");

    // (Optional) Implement FK, planning, and motion execution here, similar to timer_callback
    process_action_with_joint_data();

    // Reset flags if needed
    joint_data_received_ = false;
  }

  void process_action_with_joint_data()
  {
    // Copy current joint names/positions and action data to separate variables
    std::vector<std::string> current_joint_names = joint_names_;
    std::vector<double> current_joint_positions = joint_positions_;
    std::vector<float> current_action_data = action_data_;

    // Prepare the joint positions for logging
    RCLCPP_INFO(logger_, "Current joint positions: [%f, %f, %f, %f, %f, %f]", 
    joint_positions_[0], joint_positions_[1], joint_positions_[2], 
    joint_positions_[3], joint_positions_[4], joint_positions_[5]);


    RCLCPP_INFO(logger_, "Checking if /compute_fk service is available...");
    if (!fk_client_->wait_for_service(std::chrono::seconds(3))) {
      RCLCPP_ERROR(logger_, "Interrupted while waiting for /compute_fk service.");
      return;
    }

    RCLCPP_INFO(logger_, "Preparing /compute_fk service request...");
    auto request = std::make_shared<moveit_msgs::srv::GetPositionFK::Request>();
    // request->header.frame_id = "base_link";
    request->header.frame_id = "world";
    request->fk_link_names.push_back("right_finger_prox_link");
    request->robot_state.joint_state.name = current_joint_names;
    request->robot_state.joint_state.position = current_joint_positions;

    RCLCPP_INFO(logger_, "Sending request to /compute_fk service...");
    auto future = fk_client_->async_send_request(request);
    RCLCPP_INFO(logger_, "Waiting for /compute_fk response...");
    try {
        auto response = future.get();
        RCLCPP_INFO(logger_, "Received /compute_fk response.");
        if (response->error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
            end_effector_ = {
                response->pose_stamped[0].pose.position.x,
                response->pose_stamped[0].pose.position.y,
                response->pose_stamped[0].pose.position.z,
                response->pose_stamped[0].pose.orientation.x,
                response->pose_stamped[0].pose.orientation.y,
                response->pose_stamped[0].pose.orientation.z,
                response->pose_stamped[0].pose.orientation.w};

            // Check if the computed FK position is correct with "ros2 run tf2_ros tf2_echo world right_finger_prox_link"
            RCLCPP_INFO(logger_, "Computed FK position: [pos x: %f, pos y: %f, pos z: %f, rot x: %f, rot y: %f, rot z: %f, rot w: %f]",
                        end_effector_[0], end_effector_[1], end_effector_[2], end_effector_[3], end_effector_[4], end_effector_[5], end_effector_[6]);

            // Convert RPY (from action data) to Quaternion
            tf2::Quaternion current_action_data_q;
            current_action_data_q.setRPY(current_action_data[3], current_action_data[4], current_action_data[5]);

            RCLCPP_INFO(logger_, "Displacement: [delta pos x: %f, delta pos y: %f, delta pos z: %f, delta rot x: %f, delta rot y: %f, delta rot z: %f, delta rot w: %f]", 
            current_action_data[0], current_action_data[1], current_action_data[2], 
            current_action_data_q.x(), current_action_data_q.y(), current_action_data_q.z(), current_action_data_q.w());


            // Create a new target pose using action data
            geometry_msgs::msg::Pose target_pose;
            target_pose.position.x = end_effector_[0] + current_action_data[0];
            target_pose.position.y = end_effector_[1] + current_action_data[1];
            target_pose.position.z = end_effector_[2] + current_action_data[2];
            target_pose.orientation.x = end_effector_[3] + current_action_data_q.x();
            target_pose.orientation.y = end_effector_[4] + current_action_data_q.y();
            target_pose.orientation.z = end_effector_[5] + current_action_data_q.z();
            target_pose.orientation.w = end_effector_[6] + current_action_data_q.w();
            // target_pose.position.x = end_effector_[0] + current_action_data_[0] - 0.1;
            // target_pose.position.y = end_effector_[1] + current_action_data_[1] - 0.1;
            // target_pose.position.z = end_effector_[2] + current_action_data_[2] - 0.1;
            // target_pose.orientation.x = end_effector_[3] + current_action_data_q_.x() - 0.1;
            // target_pose.orientation.y = end_effector_[4] + current_action_data_q_.y() - 0.1;
            // target_pose.orientation.z = end_effector_[5] + current_action_data_q_.z() - 0.1;
            // target_pose.orientation.w = end_effector_[6] + current_action_data_q_.w() - 0.1;
            // target_pose.position.x = 0.28;
            // target_pose.position.y = -0.2;
            // target_pose.position.z = 0.1;
            // target_pose.orientation.w = 1.0;

            RCLCPP_INFO(logger_, "Target: [pos x: %f, pos y: %f, pos z: %f, rot x: %f, rot y: %f, rot z: %f, rot w: %f]",
                target_pose.position.x, target_pose.position.y, target_pose.position.z, target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z, target_pose.orientation.w);
            // // Plan and execute motion
            // move_group_interface_.setPoseTarget(target_pose, "right_finger_prox_link");
            // move_group_interface_.setPlanningTime(1.0);

            // RCLCPP_INFO(logger_, "Planning motion...");
            // moveit::planning_interface::MoveGroupInterface::Plan plan;
            // if (move_group_interface_.plan(plan)) {
            //     RCLCPP_INFO(logger_, "Planning succeeded. Executing...");
            //     move_group_interface_.execute(plan);
            // } else {
            //     RCLCPP_ERROR(logger_, "Motion planning failed.");
            // }


            // Start inverse kinematics based on target end effector position
            RCLCPP_INFO(logger_, "Checking if /compute_ik service is available...");
            if (!ik_client_->wait_for_service(std::chrono::seconds(3))) {
              RCLCPP_ERROR(logger_, "Interrupted while waiting for /compute_ik service.");
              return;
            }

            RCLCPP_INFO(logger_, "Preparing /compute_ik service request...");
            // Define the target pose for the end effector
            auto request = std::make_shared<moveit_msgs::srv::GetPositionIK::Request>();
            request->ik_request.group_name = "gen3_lite_arm";
            request->ik_request.ik_link_name = "right_finger_prox_link";
            request->ik_request.robot_state.is_diff = false;
            request->ik_request.timeout.sec = 1;  // Timeout for IK calculation
            request->ik_request.avoid_collisions = true;

            // Set the target pose
            request->ik_request.pose_stamped.header.frame_id = "world";
            request->ik_request.pose_stamped.pose.position.x = target_pose.position.x;
            request->ik_request.pose_stamped.pose.position.y = target_pose.position.y;
            request->ik_request.pose_stamped.pose.position.z = target_pose.position.z;
            request->ik_request.pose_stamped.pose.orientation.x = target_pose.orientation.x;
            request->ik_request.pose_stamped.pose.orientation.y = target_pose.orientation.y;
            request->ik_request.pose_stamped.pose.orientation.z = target_pose.orientation.z;
            request->ik_request.pose_stamped.pose.orientation.w = target_pose.orientation.w;

            RCLCPP_INFO(logger_, "Sending request to /compute_ik service...");
            auto ik_future = ik_client_->async_send_request(request);
            RCLCPP_INFO(logger_, "Waiting for /compute_ik response...");
            try{
                auto response = ik_future.get();
                RCLCPP_INFO(logger_, "Received /compute_ik response.");
                if (response->error_code.val == response->error_code.SUCCESS)
                {
                    RCLCPP_INFO(logger_, "IK solution found!");
                    // // Print the joint names and corresponding positions
                    // for (size_t i = 0; i < response->solution.joint_state.name.size(); ++i)
                    // {
                    //     std::string joint_name = response->solution.joint_state.name[i];
                    //     double joint_position = response->solution.joint_state.position[i];

                    //     RCLCPP_INFO(rclcpp::get_logger("KinovaIK"), "  %s: %f", joint_name.c_str(), joint_position);
                    // }
                    publish_trajectory(response->solution.joint_state.name, response->solution.joint_state.position);
                }
                else
                {
                    RCLCPP_ERROR(logger_, "Failed to find an IK solution. Error code: %d", response->error_code.val);
                }
            }
            catch (const std::exception &e) {
                RCLCPP_ERROR(logger_, "Exception during IK service call: %s", e.what());
            }

        } else {
            RCLCPP_ERROR(logger_, "Failed to compute forward kinematics.");
        }
    }
    catch (const std::exception &e) {
        RCLCPP_ERROR(logger_, "Exception during FK service call: %s", e.what());
    }
  }

  void publish_trajectory(const std::vector<std::string> &joint_names, const std::vector<double> &joint_positions)
  {
    // Check if the positions vector has at least 6 values
    if (joint_positions.size() < 6) {
        RCLCPP_ERROR(logger_, "Not enough positions provided. Expected at least 6 values.");
        return;
    }

    // // Extract the first 6 positions
    // std::vector<double> positions_for_6_joints(positions.begin(), positions.begin() + 6);

    // Copy current joint names/positions and action data to separate variables
    std::vector<std::string> current_joint_names = joint_names;
    std::vector<double> current_joint_positions = joint_positions;


    // // Output joint names and positions
    // std::cout << "joint names: ";
    // for (const auto& name : joint_names_) {
    //     std::cout << name << " ";
    // }
    // std::cout << std::endl;

    // std::cout << "joint positions: ";
    // for (const auto& position : joint_positions_) {
    //     std::cout << position << " ";
    // }
    // std::cout << std::endl;


    // Desired order of joints without "right_finger_bottom_joint"
    std::vector<std::string> desired_order = {
        "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"
    };

    // Create a map to store positions indexed by joint names
    std::unordered_map<std::string, double> name_position_map;
    for (size_t i = 0; i < current_joint_names.size(); ++i) {
        name_position_map[current_joint_names[i]] = current_joint_positions[i];
    }

    // Reorder joint_names_ and joint_positions_ based on desired_order
    std::vector<double> reordered_positions;
    std::vector<std::string> reordered_names;
    for (const auto& name : desired_order) {
        if (name_position_map.find(name) != name_position_map.end()) {
            reordered_positions.push_back(name_position_map[name]);
            reordered_names.push_back(name);
        }
    }

    current_joint_names = reordered_names;      // Update joint_names_
    current_joint_positions = reordered_positions; // Update joint_positions_


    // Create the trajectory message
    trajectory_msgs::msg::JointTrajectory trajectory_msg;
    trajectory_msg.joint_names = current_joint_names;
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = current_joint_positions;
    point.time_from_start.sec = 1;  // Set the time from start as needed

    // Add the point to the trajectory
    trajectory_msg.points.push_back(point);

    // Log the trajectory (for debugging purposes)
    RCLCPP_INFO(logger_, "Trajectory message joint names: ");
    for (const auto& name : trajectory_msg.joint_names) {
        RCLCPP_INFO(logger_, "%s", name.c_str());
    }

    RCLCPP_INFO(logger_, "Trajectory message positions: ");
    for (const auto& pos : point.positions) {
        RCLCPP_INFO(logger_, "%f", pos);
    }

    // Publish the trajectory
    trajectory_publisher_->publish(trajectory_msg);

    RCLCPP_INFO(logger_, "Published joint trajectory.");
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger logger_;
  moveit::planning_interface::MoveGroupInterface move_group_interface_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscription_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_subscription_;
  rclcpp::Client<moveit_msgs::srv::GetPositionFK>::SharedPtr fk_client_;
  rclcpp::Client<moveit_msgs::srv::GetPositionIK>::SharedPtr ik_client_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_publisher_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::CallbackGroup::SharedPtr timer_cb_group_;
  // rclcpp::TimerBase::SharedPtr timer_;
  int callback_count_;
  std::vector<float> action_data_;
  std::array<double, 7> end_effector_;
  std::vector<std::string> joint_names_;
  std::vector<double> joint_positions_;
  // bool action_data_received_;  // Flag to track if action data was received
  bool joint_data_received_;   // Flag to track if joint data was received
  // std::vector<float> current_action_data_; // Declare as a member
  // std::vector<double> current_joint_positions_; // Declare as a member
  // const std::string from_frame = "world";
  // const std::string to_frame = "end_effector";
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  KinovaNoMoveItOpenvla kinova_no_moveit_openvla;
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(kinova_no_moveit_openvla.get_node());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}



