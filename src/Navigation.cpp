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

    const rclcpp::QoS qos = rclcpp::ServicesQoS();

    go_to_client_ =
        node_->create_client<
            known_locations_interfaces::srv::LocationsGet>(
                "/nav_controller/go_to_location",
                qos,
                cb_group_clients_);

    add_location_client_ =
        node_->create_client<
            known_locations_interfaces::srv::LocationsServer>(
                "/locations_manager/add_location",
                qos,
                cb_group_clients_);
}

// ============================================================
// GoTo
// ============================================================

bool Navigation::GoTo(const std::string& location_name,
                      std::chrono::seconds timeout)
{
    auto request =
        std::make_shared<
            known_locations_interfaces::srv::LocationsGet::Request>();

    request->location_name = location_name;

    bool success = false;

    bool completed =
        call_service<
            known_locations_interfaces::srv::LocationsGet>(
                go_to_client_,
                request,
                [&](auto response)
                {
                    success = response->success;

                    RCLCPP_INFO(
                        node_->get_logger(),
                        "GoTo [%s] %s: %s",
                        location_name.c_str(),
                        success ? "SUCCESS" : "FAILED",
                        response->message.c_str());
                },
                timeout,
                "GoTo");

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
