#include <memory>
#include <vector>
#include <sstream>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/srv/get_position_fk.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <algorithm>
#include <unordered_map>


class KinovaMoveItOpenvla
{
public:
  KinovaMoveItOpenvla()
  : node_(std::make_shared<rclcpp::Node>("kinova_moveit_openvla_", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))),
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
        std::bind(&KinovaMoveItOpenvla::action_callback, this, std::placeholders::_1), 
        options_);

    joint_subscription_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", 10, 
        std::bind(&KinovaMoveItOpenvla::joint_callback, this, std::placeholders::_1), 
        options_);

    fk_client_ = node_->create_client<moveit_msgs::srv::GetPositionFK>(
        "compute_fk", rmw_qos_profile_services_default, timer_cb_group_);
  

    // // Timer to handle the FK service and planning
    // timer_ = node_->create_wall_timer(
    //     // std::chrono::seconds(1), 
    //     std::chrono::milliseconds(200), 
    //     std::bind(&KinovaMoveItOpenvlaSim::timer_callback, this),
    //     timer_cb_group_);
      
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
    // // Desired order of joints with "right_finger_bottom_joint"
    // std::vector<std::string> desired_order_full = {
    //     "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "right_finger_bottom_joint"
    // };


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
    // std::vector<std::string> reordered_names;
    // for (const auto& name : desired_order_full) {
    //     if (name_position_map.find(name) != name_position_map.end()) {
    //         reordered_positions.push_back(name_position_map[name]);
    //         reordered_names.push_back(name);
    //     }
    // }

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

            // Plan and execute motion
            move_group_interface_.setPoseTarget(target_pose, "right_finger_prox_link");
            move_group_interface_.setPlanningTime(1.0);

            RCLCPP_INFO(logger_, "Planning motion...");
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            if (move_group_interface_.plan(plan)) {
                RCLCPP_INFO(logger_, "Planning succeeded. Executing...");
                move_group_interface_.execute(plan);
            } else {
                RCLCPP_ERROR(logger_, "Motion planning failed.");
            }
        } else {
            RCLCPP_ERROR(logger_, "Failed to compute forward kinematics.");
        }
    }
    catch (const std::exception &e) {
        RCLCPP_ERROR(logger_, "Exception during FK service call: %s", e.what());
    }
  }



  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger logger_;
  moveit::planning_interface::MoveGroupInterface move_group_interface_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscription_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_subscription_;
  rclcpp::Client<moveit_msgs::srv::GetPositionFK>::SharedPtr fk_client_;
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
  KinovaMoveItOpenvla kinova_moveit_openvla;
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(kinova_moveit_openvla.get_node());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}

// #include <memory>
// #include <vector>
// #include <sstream>
// #include <rclcpp/rclcpp.hpp>
// #include <std_msgs/msg/float32_multi_array.hpp>
// #include <sensor_msgs/msg/joint_state.hpp>
// #include <moveit/move_group_interface/move_group_interface.h>
// #include <geometry_msgs/msg/pose.hpp>
// #include <moveit_msgs/srv/get_position_fk.hpp>

// class KinovaMoveItOpenvla
// {
// public:
//   KinovaMoveItOpenvla()
//   : node_(std::make_shared<rclcpp::Node>("kinova_moveit_openvla", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))),
//     logger_(node_->get_logger()),
//     move_group_interface_(node_, "gen3_lite_arm"),
//     callback_count_(0),
//     joint_data_received_(false)
//   {

//     callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
//     timer_cb_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

//     // Subscriptions
//     rclcpp::SubscriptionOptions options_;
//     options_.callback_group = callback_group_;

//     // Subscriptions
//     subscription_ = node_->create_subscription<std_msgs::msg::Float32MultiArray>(
//         "openvla_action", 10, 
//         std::bind(&KinovaMoveItOpenvla::action_callback, this, std::placeholders::_1), 
//         options_);

//     joint_subscription_ = node_->create_subscription<sensor_msgs::msg::JointState>(
//         "joint_states", 10, 
//         std::bind(&KinovaMoveItOpenvla::joint_callback, this, std::placeholders::_1), 
//         options_);

//     fk_client_ = node_->create_client<moveit_msgs::srv::GetPositionFK>(
//         "compute_fk", rmw_qos_profile_services_default, timer_cb_group_);
  

//     // // Timer to handle the FK service and planning
//     // timer_ = node_->create_wall_timer(
//     //     // std::chrono::seconds(1), 
//     //     std::chrono::milliseconds(200), 
//     //     std::bind(&KinovaMoveItOpenvlaSim::timer_callback, this),
//     //     timer_cb_group_);
      
//     // We print the current planning frame for this group.
//     RCLCPP_INFO(logger_, "Planning frame: %s",
//                   move_group_interface_.getPlanningFrame().c_str());

//     // We can also print the name of the end-effector link for this group.
//     RCLCPP_INFO(logger_, "End effector link: %s",
//               move_group_interface_.getEndEffectorLink().c_str());

//     // We can get a list of all the groups in the robot:
//     RCLCPP_INFO(logger_, "Available Planning Groups:");
//     std::copy(move_group_interface_.getJointModelGroupNames().begin(),
//             move_group_interface_.getJointModelGroupNames().end(),
//             std::ostream_iterator<std::string>(std::cout, ", "));

//   }

//   // Getter for node_ pointer
//   rclcpp::Node::SharedPtr get_node() { return node_; }

// private:
//   std::vector<float> current_action_data_; // Declare as a member
//   std::vector<double> current_joint_positions_; // Declare as a member

//   // Callback function to handle joint states data and update joint_positions_
//   void joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
//   {
//     joint_positions_ = msg->position;
//     joint_data_received_ = true;  // Mark that joint data has been received

//     // // Prepare the joint positions for logging
//     // std::ostringstream oss;
//     // oss << "Subscribing joint state, #" << callback_count_ << ", joint_positions: [";
//     // for (size_t i = 0; i < joint_positions_.size(); ++i) {
//     //     oss << joint_positions_[i];
//     //     if (i < joint_positions_.size() - 1) oss << ", ";
//     // }
//     // oss << "]";

//     // // Log the joint positions with action count
//     // RCLCPP_INFO(logger_, "%s", oss.str().c_str());

//   }

//   // Callback function to handle incoming action data
//   void action_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
//   {
//       callback_count_++;
//       action_data_ = msg->data;

//       // Log received action data
//       std::ostringstream oss;

//       oss << "action data: [";
//       for (size_t i = 0; i < action_data_.size(); ++i) {
//           oss << action_data_[i];
//           if (i < action_data_.size() - 1) oss << ", ";
//       }
//       oss << "]";
//       RCLCPP_INFO(logger_, "Subscribing openvla action, %s, #%d", oss.str().c_str(), callback_count_);

//       // Clear oss if you need to reuse it
//       oss.str("");  // Clear contents
//       oss.clear();  // Clear error flag

//       if (!joint_data_received_) {
//           RCLCPP_WARN(logger_, "No joint data available to process action.");
//           return; // Skip processing if joint data isn't available
//       }

//       RCLCPP_INFO(logger_, "Both action and joint data received.");

//       // Process action using the latest joint states
//       RCLCPP_INFO(logger_, "Processing action using latest joint states...");

//       // (Optional) Implement FK, planning, and motion execution here, similar to timer_callback
//       process_action_with_joint_data();

//       // Reset flags if needed
//       joint_data_received_ = false;
//   }

//   void process_action_with_joint_data()
//   {
//     //   if (!joint_data_received_) {
//     //       RCLCPP_WARN(logger_, "No joint data available to process action.");
//     //       return; // Skip FK computation if joint data is missing
//     //   }

//       // Copy current joint positions and action data to separate variables
//       std::vector<double> current_joint_positions_ = joint_positions_;
//       std::vector<float> current_action_data_ = action_data_;

//       // Prepare the joint positions for logging
//       std::ostringstream oss;
//       oss << "Current joint state: [";
//       for (size_t i = 0; i < current_joint_positions_.size(); ++i) {
//           oss << current_joint_positions_[i];
//           if (i < current_joint_positions_.size() - 1) oss << ", ";
//       }
//       oss << "]";
//       // Log the joint positions with action count
//       RCLCPP_INFO(logger_, "%s", oss.str().c_str());

//       // Clear oss if you need to reuse it
//       oss.str("");  // Clear contents
//       oss.clear();  // Clear error flag

//       // Log received action data
//       //   std::ostringstream oss;
//       oss << "Current action data: [";
//       for (size_t i = 0; i < current_action_data_.size(); ++i) {
//           oss << current_action_data_[i];
//           if (i < current_action_data_.size() - 1) oss << ", ";
//       }
//       oss << "]";
//       RCLCPP_INFO(logger_, "%s", oss.str().c_str());


//       if (!fk_client_->wait_for_service(std::chrono::seconds(3))) {
//           RCLCPP_ERROR(logger_, "Interrupted while waiting for /compute_fk service.");
//           return;
//       }

//       RCLCPP_INFO(logger_, "Checking if /compute_fk service is available...");
//       if (!fk_client_->wait_for_service(std::chrono::seconds(3))) {
//       RCLCPP_ERROR(logger_, "Interrupted while waiting for /compute_fk service.");
//       return;
//       }

//       RCLCPP_INFO(logger_, "Preparing /compute_fk service request...");
//       auto request = std::make_shared<moveit_msgs::srv::GetPositionFK::Request>();
//       // request->header.frame_id = "base_link";
//       request->header.frame_id = "world";
//       // request->fk_link_names.push_back("end_effector_link");
//       request->fk_link_names.push_back("right_finger_prox_link");
//       request->robot_state.joint_state.name = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};
//       request->robot_state.joint_state.position = current_joint_positions_;

//       RCLCPP_INFO(logger_, "Sending request to /compute_fk service...");
//       auto future = fk_client_->async_send_request(request);
//       RCLCPP_INFO(logger_, "Waiting for /compute_fk response...");
//       try {
//           auto response = future.get();
//           RCLCPP_INFO(logger_, "Received /compute_fk response.");
//           if (response->error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
//               end_effector_ = {
//                   response->pose_stamped[0].pose.position.x,
//                   response->pose_stamped[0].pose.position.y,
//                   response->pose_stamped[0].pose.position.z,
//                   response->pose_stamped[0].pose.orientation.x,
//                   response->pose_stamped[0].pose.orientation.y,
//                   response->pose_stamped[0].pose.orientation.z};
//                   // response->pose_stamped[0].pose.orientation.w};

//               RCLCPP_INFO(logger_, "Computed FK position: [pos x: %f, pos y: %f, pos z: %f, rot x: %f, rot y: %f, rot z: %f]",
//                           end_effector_[0], end_effector_[1], end_effector_[2], end_effector_[3], end_effector_[4], end_effector_[5]);

//               // Create a new target pose using action data
//               geometry_msgs::msg::Pose target_pose;
//               // target_pose.position.x = end_effector_[0] + current_action_data_[0];
//               // target_pose.position.y = end_effector_[1] + current_action_data_[1];
//               // target_pose.position.z = end_effector_[2] + current_action_data_[2];
//               // target_pose.orientation.x = end_effector_[3] + current_action_data_[3];
//               // target_pose.orientation.y = end_effector_[4] + current_action_data_[4];
//               // target_pose.orientation.z = end_effector_[5] + current_action_data_[5];
//               // target_pose.position.x = 0.28;
//               // target_pose.position.y = -0.2;
//               // target_pose.position.z = 0.1;
//               target_pose.orientation.w = 1.0;


//               RCLCPP_INFO(logger_, "Target: [pos x: %f, pos y: %f, pos z: %f, rot x: %f, rot y: %f, rot z: %f]",
//                     target_pose.position.x, target_pose.position.y, target_pose.position.z, target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z);

//               // Plan and execute motion
//               move_group_interface_.setPoseTarget(target_pose, "right_finger_prox_link");
//               move_group_interface_.setPlanningTime(5.0);

//               RCLCPP_INFO(logger_, "Planning motion...");
//               moveit::planning_interface::MoveGroupInterface::Plan plan;
//               if (move_group_interface_.plan(plan)) {
//                   RCLCPP_INFO(logger_, "Planning succeeded. Executing...");
//                   move_group_interface_.execute(plan);
//               } else {
//                   RCLCPP_ERROR(logger_, "Motion planning failed.");
//               }
//           } else {
//               RCLCPP_ERROR(logger_, "Failed to compute forward kinematics.");
//           }
//       } catch (const std::exception &e) {
//           RCLCPP_ERROR(logger_, "Exception during FK service call: %s", e.what());
//       }
//   }


//   rclcpp::Node::SharedPtr node_;
//   rclcpp::Logger logger_;
//   moveit::planning_interface::MoveGroupInterface move_group_interface_;
//   rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscription_;
//   rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_subscription_;
//   rclcpp::Client<moveit_msgs::srv::GetPositionFK>::SharedPtr fk_client_;
//   rclcpp::CallbackGroup::SharedPtr callback_group_;
//   rclcpp::CallbackGroup::SharedPtr timer_cb_group_;
// //   rclcpp::TimerBase::SharedPtr timer_;
//   int callback_count_;
//   std::vector<float> action_data_;
//   std::array<double, 6> end_effector_;
//   std::vector<double> joint_positions_;
//   // bool action_data_received_;  // Flag to track if action data was received
//   bool joint_data_received_;   // Flag to track if joint data was received
// };

// int main(int argc, char * argv[])
// {
//   rclcpp::init(argc, argv);
//   KinovaMoveItOpenvla kinova_moveit_openvla;
//   rclcpp::executors::MultiThreadedExecutor executor;
//   executor.add_node(kinova_moveit_openvla.get_node());
//   executor.spin();
//   rclcpp::shutdown();
//   return 0;
// }



// #include <memory>
// #include <vector>
// #include <sstream>
// #include <rclcpp/rclcpp.hpp>
// #include <std_msgs/msg/float32_multi_array.hpp>
// #include <sensor_msgs/msg/joint_state.hpp>
// #include <moveit/move_group_interface/move_group_interface.h>
// #include <geometry_msgs/msg/pose.hpp>
// #include <moveit_msgs/srv/get_position_fk.hpp>

// class KinovaMoveItOpenvla
// {
// public:
//   KinovaMoveItOpenvla()
//   : node_(std::make_shared<rclcpp::Node>("kinova_moveit_openvla", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))),
//     logger_(node_->get_logger()),
//     move_group_interface_(node_, "gen3_lite_arm"),
//     callback_count_(0),
//     action_data_received_(false),
//     joint_data_received_(false)
//   {
//     // callback_group_1_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
//     // callback_group_2_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
//     // callback_group_3_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
//     // timer_cb_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

//     // // Subscriptions
//     // rclcpp::SubscriptionOptions options_1;
//     // options_1.callback_group = callback_group_1_;
//     // rclcpp::SubscriptionOptions options_2;
//     // options_2.callback_group = callback_group_2_;

//     callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
//     timer_cb_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

//     // callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
//     // timer_cb_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

//     // Subscriptions
//     rclcpp::SubscriptionOptions options_;
//     options_.callback_group = callback_group_;

//     // Subscriptions
//     subscription_ = node_->create_subscription<std_msgs::msg::Float32MultiArray>(
//         "openvla_action", 10, 
//         std::bind(&KinovaMoveItOpenvla::action_callback, this, std::placeholders::_1), 
//         options_);

//     joint_subscription_ = node_->create_subscription<sensor_msgs::msg::JointState>(
//         "joint_states", 10, 
//         std::bind(&KinovaMoveItOpenvla::joint_callback, this, std::placeholders::_1), 
//         options_);

//     fk_client_ = node_->create_client<moveit_msgs::srv::GetPositionFK>(
//         "compute_fk", rmw_qos_profile_services_default, callback_group_);
  

//     // Timer to handle the FK service and planning
//     timer_ = node_->create_wall_timer(
//         // std::chrono::seconds(1), 
//         std::chrono::milliseconds(200), 
//         std::bind(&KinovaMoveItOpenvla::timer_callback, this),
//         timer_cb_group_);

//     // // Callback groups
//     // callback_group_1_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
//     // callback_group_2_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
//     // callback_group_3_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

//     // // Subscriptions
//     // rclcpp::SubscriptionOptions options_1;
//     // options_1.callback_group = callback_group_1_;
//     // rclcpp::SubscriptionOptions options_2;
//     // options_2.callback_group = callback_group_2_;

//     // subscription_ = node_->create_subscription<std_msgs::msg::Float32MultiArray>(
//     //     "openvla_action_dummy", 10, 
//     //     std::bind(&KinovaMoveItOpenvlaSim::action_callback, this, std::placeholders::_1), 
//     //     options_1);

//     // joint_subscription_ = node_->create_subscription<sensor_msgs::msg::JointState>(
//     //     "joint_states", 10, 
//     //     std::bind(&KinovaMoveItOpenvlaSim::joint_callback, this, std::placeholders::_1), 
//     //     options_2);

//     // fk_client_ = node_->create_client<moveit_msgs::srv::GetPositionFK>(
//     //     "compute_fk", rmw_qos_profile_services_default, callback_group_3_);

//     // // Timer to handle the FK service and planning
//     // timer_ = node_->create_wall_timer(
//     //     std::chrono::seconds(5), 
//     //     std::bind(&KinovaMoveItOpenvlaSim::timer_callback, this));
//   }

//   // Getter for node_ pointer
//   rclcpp::Node::SharedPtr get_node() { return node_; }

// private:
//   // Callback function to handle incoming action data
//   void action_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
//   {
//     callback_count_++;
//     action_data_ = msg->data;
//     action_data_received_ = true;  // Mark that action data has been received

//     std::ostringstream oss;
//     oss << "Received action data: [";
//     for (size_t i = 0; i < action_data_.size(); ++i) {
//       oss << action_data_[i];
//       if (i < action_data_.size() - 1) oss << ", ";
//     }
//     oss << "]";

//     RCLCPP_INFO(logger_, "Subscribing openvla action, %s, #%d", oss.str().c_str(), callback_count_);
//   }

//   // Callback function to handle joint states data and update joint_positions_
//   void joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
//   {
//     joint_positions_ = msg->position;
//     joint_data_received_ = true;  // Mark that joint data has been received

//     // // Prepare the joint positions for logging
//     // std::ostringstream oss;
//     // oss << "Subscribing joint state, #" << callback_count_ << ", joint_positions: [";
//     // for (size_t i = 0; i < joint_positions_.size(); ++i) {
//     //     oss << joint_positions_[i];
//     //     if (i < joint_positions_.size() - 1) oss << ", ";
//     // }
//     // oss << "]";

//     // // Log the joint positions with action count
//     // RCLCPP_INFO(logger_, "%s", oss.str().c_str());
//   }


//   // Timer callback to check if new action and joint data has been received
//   void timer_callback()
//   {
//     if (action_data_received_ && joint_data_received_) {
//         RCLCPP_INFO(logger_, "Both action and joint data received.");

//         // Perform the actions you need to execute after receiving both data

//         RCLCPP_INFO(logger_, "Checking if /compute_fk service is available...");
//         if (!fk_client_->wait_for_service(std::chrono::seconds(3))) {
//         RCLCPP_ERROR(logger_, "Interrupted while waiting for /compute_fk service.");
//         return;
//         }

//         RCLCPP_INFO(logger_, "Preparing /compute_fk service request...");
//         auto request = std::make_shared<moveit_msgs::srv::GetPositionFK::Request>();
//         request->header.frame_id = "base_link";
//         request->fk_link_names.push_back("end_effector_link");
//         request->robot_state.joint_state.name = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};
//         request->robot_state.joint_state.position = joint_positions_;

//         RCLCPP_INFO(logger_, "Sending request to /compute_fk service...");
//         auto future = fk_client_->async_send_request(request);

//         // // if (rclcpp::spin_until_future_complete(node_, future) == rclcpp::FutureReturnCode::SUCCESS) {
//         // auto response = future.get();
//         // if (response->error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
//         //   end_effector_ = {response->pose_stamped[0].pose.position.x,
//         //                             response->pose_stamped[0].pose.position.y,
//         //                             response->pose_stamped[0].pose.position.z};
//         //   RCLCPP_INFO(logger_, "Current end effector position: [x: %f, y: %f, z: %f]",
//         //               end_effector_[0], end_effector_[1], end_effector_[2]);
//         // } else {
//         //   RCLCPP_ERROR(logger_, "Failed to compute forward kinematics.");
//         // }
//         // // } else {
//         // //   RCLCPP_ERROR(logger_, "Failed to call /compute_fk service.");
//         // // }

//         RCLCPP_INFO(logger_, "Waiting for /compute_fk response...");
//         try {
//             auto response = future.get();
//             RCLCPP_INFO(logger_, "Received /compute_fk response.");
//             if (response->error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
//                 end_effector_ = {
//                     response->pose_stamped[0].pose.position.x,
//                     response->pose_stamped[0].pose.position.y,
//                     response->pose_stamped[0].pose.position.z};
//                 RCLCPP_INFO(logger_, "Current end effector position: [x: %f, y: %f, z: %f]",
//                             end_effector_[0], end_effector_[1], end_effector_[2]);
//             } else {
//                 RCLCPP_ERROR(logger_, "Failed to compute forward kinematics. Error code: %d",
//                              response->error_code.val);
//                 return;
//             }
//         } catch (const std::exception &e) {
//             RCLCPP_ERROR(logger_, "Exception while calling /compute_fk service: %s", e.what());
//             return;
//         }

//         geometry_msgs::msg::Pose target_pose;
//         target_pose.orientation.w = 1.0;
//         target_pose.position.x = end_effector_[0] + action_data_[0];
//         target_pose.position.y = end_effector_[1] + action_data_[1];
//         target_pose.position.z = end_effector_[2] + action_data_[2];

//         RCLCPP_INFO(logger_, "Target pose: x: %f, y: %f, z: %f",
//                     target_pose.position.x, target_pose.position.y, target_pose.position.z);

//         move_group_interface_.setPoseTarget(target_pose);
//         move_group_interface_.setPlanningTime(1.0);

//         RCLCPP_INFO(logger_, "Planning motion...");
//         moveit::planning_interface::MoveGroupInterface::Plan plan;
//         auto success = static_cast<bool>(move_group_interface_.plan(plan));

//         RCLCPP_INFO(logger_, "Planning %s", success ? "succeeded" : "failed");

//         if (success) {
//           RCLCPP_INFO(logger_, "Planning succeeded. Executing...");
//           move_group_interface_.execute(plan);
//         } else {
//           RCLCPP_ERROR(logger_, "Planning failed!");
//           // Fallback to Cartesian planning
//           std::vector<geometry_msgs::msg::Pose> waypoints = {target_pose};
//           moveit_msgs::msg::RobotTrajectory trajectory;
//           double fraction = move_group_interface_.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);

//           if (fraction > 0.0) {
//               RCLCPP_INFO(logger_, "Cartesian path succeeded for %.2f%% of the path", fraction * 100.0);
//               move_group_interface_.execute(trajectory);
//           } else {
//               RCLCPP_ERROR(logger_, "Cartesian planning also failed!");
//           }
//         }

//         // Reset the flags for the next cycle
//         action_data_received_ = false;
//         joint_data_received_ = false;


//     } else {
//       RCLCPP_WARN(logger_, "Waiting for action and joint data...");
//     }
//   }

//   rclcpp::Node::SharedPtr node_;
//   rclcpp::Logger logger_;
//   moveit::planning_interface::MoveGroupInterface move_group_interface_;
//   rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscription_;
//   rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_subscription_;
//   rclcpp::Client<moveit_msgs::srv::GetPositionFK>::SharedPtr fk_client_;
//   rclcpp::CallbackGroup::SharedPtr callback_group_;
//   rclcpp::CallbackGroup::SharedPtr timer_cb_group_;
//   // rclcpp::CallbackGroup::SharedPtr callback_group_1_;
//   // rclcpp::CallbackGroup::SharedPtr callback_group_2_;
//   // rclcpp::CallbackGroup::SharedPtr callback_group_3_;
//   rclcpp::TimerBase::SharedPtr timer_;
//   int callback_count_;
//   std::vector<float> action_data_;
//   std::array<double, 3> end_effector_;
//   std::vector<double> joint_positions_;
//   bool action_data_received_;  // Flag to track if action data was received
//   bool joint_data_received_;   // Flag to track if joint data was received
// };

// int main(int argc, char * argv[])
// {
//   rclcpp::init(argc, argv);
//   KinovaMoveItOpenvla kinova_moveit_openvla;
//   rclcpp::executors::MultiThreadedExecutor executor;
//   executor.add_node(kinova_moveit_openvla.get_node());
//   executor.spin();
//   rclcpp::shutdown();
//   return 0;
// }

