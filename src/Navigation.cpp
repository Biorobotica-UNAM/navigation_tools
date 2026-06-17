#include "navigation_tools/Navigation.hpp"

// ============================================================
// Constructor
// ============================================================

Navigation::Navigation(const rclcpp::Node::SharedPtr& node)
: node_(node)
{
    cb_group_clients_ =
        node_->create_callback_group(
            rclcpp::CallbackGroupType::Reentrant);

    // Guys, please. Make sure the code compiles with ROS 2 Humble too. We talked about this!!!
    //const rclcpp::QoS qos = rclcpp::ServicesQoS();
    const rmw_qos_profile_t qos = rmw_qos_profile_services_default;

    go_to_client_ =
        node_->create_client<
            known_locations_interfaces::srv::LocationsGet>(
                "/nav_controller/go_to_location",
                qos,
                cb_group_clients_);

    add_location_client_ =
        node_->create_client<
            known_locations_interfaces::srv::LocationsServer>(
                "/locations_manager/add",
                qos,
                cb_group_clients_);

    save_pose_client_ =
        node_->create_client<
            known_locations_interfaces::srv::SavePoseLocation>(
                "/locations_manager/save",
                qos,
                cb_group_clients_);
}

// ============================================================
// GoTo
// ============================================================

bool Navigation::GoTo(const std::string& location_name,
                      std::chrono::seconds timeout)
{
    return GoToDetailed(location_name, timeout).success;
}

Navigation::NavigationResult
Navigation::GoToDetailed(const std::string& location_name,
                         std::chrono::seconds timeout)
{
    auto request =
        std::make_shared<
            known_locations_interfaces::srv::LocationsGet::Request>();

    request->location_name = location_name;

    NavigationResult result;

    bool completed =
        call_service<
            known_locations_interfaces::srv::LocationsGet>(
                go_to_client_,
                request,
                [&](auto response)
                {
                    result.success = response->success;
                    result.message = response->message;

                    RCLCPP_INFO(
                        node_->get_logger(),
                        "GoTo [%s] %s: %s",
                        location_name.c_str(),
                        result.success ? "SUCCESS" : "FAILED",
                        response->message.c_str());
                },
                timeout,
                "GoTo");

    if (!completed) {
        result.success = false;
        result.message = "GoTo service unavailable, timed out, or threw an exception";
    }

    return result;
}

// ============================================================
// SavePoseLocation
// ============================================================

bool Navigation::SavePoseLocation(
    const std::string&              location_name,
    const geometry_msgs::msg::Pose& pose,
    std::chrono::seconds            timeout)
{
    auto request =
        std::make_shared<
            known_locations_interfaces::srv::SavePoseLocation::Request>();

    request->location_name = location_name;
    request->pose          = pose;

    bool success = false;

    bool completed =
        call_service<
            known_locations_interfaces::srv::SavePoseLocation>(
                save_pose_client_,
                request,
                [&](auto response)
                {
                    success = response->success;

                    RCLCPP_INFO(
                        node_->get_logger(),
                        "SavePoseLocation [%s] %s: %s",
                        location_name.c_str(),
                        success ? "SUCCESS" : "FAILED",
                        response->message.c_str());
                },
                timeout,
                "SavePoseLocation");

    return completed && success;
}

// ============================================================
// AddLocation
// ============================================================

bool Navigation::AddLocation(
    const std::string& location_name,
    std::chrono::seconds timeout)
{
    auto request =
        std::make_shared<
            known_locations_interfaces::srv::LocationsServer::Request>();

    request->location_name = location_name;

    bool success = false;

    bool completed =
        call_service<
            known_locations_interfaces::srv::LocationsServer>(
                add_location_client_,
                request,
                [&](auto response)
                {
                    success = response->success;

                    RCLCPP_INFO(
                        node_->get_logger(),
                        "AddLocation [%s] %s",
                        location_name.c_str(),
                        success ? "SUCCESS" : "FAILED");
                },
                timeout,
                "AddLocation");

    return completed && success;
}
