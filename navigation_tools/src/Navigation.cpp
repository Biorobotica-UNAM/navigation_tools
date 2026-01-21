#include "navigation_tools/Navigation.hpp"

Navigation::Navigation(rclcpp::Node::SharedPtr node)
    : node_(node)
{
    client_goal_ = node_->create_client<known_locations_interfaces::srv::LocationsServer>("/known_location_goal");
}

bool Navigation::GoTo(const std::string& location_name)
{
    auto request = std::make_shared<known_locations_interfaces::srv::LocationsServer::Request>();
    request->location_name = location_name;

    if (!client_goal_->wait_for_service(std::chrono::milliseconds(500)))
    {
        RCLCPP_WARN(node_->get_logger(), "Servicio /known_location_goal no disponible");
        return false;
    }

    auto future = client_goal_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(node_, future, std::chrono::seconds(120)) == rclcpp::FutureReturnCode::SUCCESS)
    {
        auto response = future.get();
        return response->success;
    }

    RCLCPP_WARN(node_->get_logger(), "No se recibió respuesta confiable del servicio");
    return false;
}
