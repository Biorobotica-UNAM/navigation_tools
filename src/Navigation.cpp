#include "navigation_tools/Navigation.hpp"

#include <chrono>
#include <future>

Navigation::Navigation(rclcpp::Node::SharedPtr node)
    : node_(std::move(node))
{
    client_goal_ =
        node_->create_client<known_locations_interfaces::srv::LocationsServer>("/known_location_goal");
}

bool Navigation::GoTo(const std::string& location_name)
{
    using namespace std::chrono_literals;

    auto request =
        std::make_shared<known_locations_interfaces::srv::LocationsServer::Request>();
    request->location_name = location_name;

    if (!client_goal_->wait_for_service(500ms))
    {
        RCLCPP_WARN(node_->get_logger(), "Servicio /known_location_goal no disponible");
        return false;
    }

    // Executor-safe: NO spin dentro de la tool.
    auto prom = std::make_shared<std::promise<bool>>();
    auto fut = prom->get_future();

    client_goal_->async_send_request(
        request,
        [this, prom, location_name](
            std::shared_future<known_locations_interfaces::srv::LocationsServer::Response::SharedPtr> future)
        {
            try
            {
                auto response = future.get();
                prom->set_value(response->success);
            }
            catch (const std::exception& e)
            {
                RCLCPP_WARN(node_->get_logger(),
                            "Excepción en respuesta /known_location_goal (%s): %s",
                            location_name.c_str(), e.what());
                prom->set_value(false);
            }
        });

    if (fut.wait_for(120s) == std::future_status::ready)
        return fut.get();

    RCLCPP_WARN(node_->get_logger(),
                "Timeout esperando respuesta de /known_location_goal (%s)",
                location_name.c_str());
    return false;
}
