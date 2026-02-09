#include "navigation_tools/Navigation.hpp"

using namespace std::chrono_literals;

Navigation::Navigation(const rclcpp::Node::SharedPtr& node)
    : node_(node)
{
    client_goal_ =
        node_->create_client<
            known_locations_interfaces::srv::LocationsServer>(
                "/known_location_goal");
}

// ---------------------------------------------------------
// Wait for service
// ---------------------------------------------------------

bool Navigation::waitForService(
    const rclcpp::ClientBase::SharedPtr& client,
    const std::chrono::milliseconds& timeout) const
{
    return client->wait_for_service(timeout);
}

// ---------------------------------------------------------
// GoTo
// ---------------------------------------------------------

bool Navigation::GoTo(const std::string& location_name)
{
    if (!waitForService(client_goal_, 500ms))
    {
        RCLCPP_WARN(node_->get_logger(),
                    "Servicio /known_location_goal no disponible");
        return false;
    }

    auto request =
        std::make_shared<
            known_locations_interfaces::srv::LocationsServer::Request>();
    request->location_name = location_name;

    std::mutex mtx;
    std::condition_variable cv;
    bool done = false;
    bool success = false;

    client_goal_->async_send_request(
        request,
        [this, &mtx, &cv, &done, &success, location_name](
            rclcpp::Client<
                known_locations_interfaces::srv::LocationsServer>::SharedFuture future)
        {
            try
            {
                auto response = future.get();
                success = response->success;
            }
            catch (const std::exception& e)
            {
                RCLCPP_WARN(node_->get_logger(),
                            "Excepción en /known_location_goal (%s): %s",
                            location_name.c_str(), e.what());
                success = false;
            }

            {
                std::lock_guard<std::mutex> lock(mtx);
                done = true;
            }
            cv.notify_one();
        });

    std::unique_lock<std::mutex> lock(mtx);

    if (!cv.wait_for(lock, 120s, [&]{ return done; }))
    {
        RCLCPP_WARN(node_->get_logger(),
                    "Timeout esperando /known_location_goal (%s)",
                    location_name.c_str());
        return false;
    }

    return success;
}

