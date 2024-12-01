#include <memory>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

void action_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    // Static variable to count the number of times the callback is invoked
    static int callback_count = 0;
    callback_count++;  // Increment the count

    // Process the received data
    std::vector<float> action_data = msg->data;

    // Convert the vector to a string using std::ostringstream
    std::ostringstream oss;
    oss << "Received action data: [";
    for (size_t i = 0; i < action_data.size(); ++i) {
        oss << action_data[i];
        if (i < action_data.size() - 1) oss << ", ";
    }
    oss << "]";

    // Log the entire vector as a single string
    RCLCPP_INFO(
        rclcpp::get_logger("openvla_test"),
        "Subscribing openvla action, %s, #%d",
        oss.str().c_str(),
        callback_count
    );

    // // Example: Log the received data
    // for (const auto & value : action_data) {
    //     RCLCPP_INFO(rclcpp::get_logger("openvla_test"), "Received value: %f", value);
    // }
}

int main(int argc, char * argv[])
{
    // Initialize ROS and create the Node
    rclcpp::init(argc, argv);
    auto const node = std::make_shared<rclcpp::Node>(
        "openvla_test",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );

    // Create a ROS logger
    auto const logger = rclcpp::get_logger("openvla_test");

    // // Create a MoveIt MoveGroup Interface
    // using moveit::planning_interface::MoveGroupInterface;
    // auto move_group_interface = MoveGroupInterface(node, "manipulator");

    // Create the subscription to the Float32MultiArray topic
    auto subscription = node->create_subscription<std_msgs::msg::Float32MultiArray>(
        "openvla_action", 10, action_callback);

    // Execute any existing code
    // ...

    // Start spinning to process the subscription callback
    rclcpp::spin(node);

    // Shutdown ROS
    rclcpp::shutdown();
    return 0;
}
