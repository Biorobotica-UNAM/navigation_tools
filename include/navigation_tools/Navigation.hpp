#pragma once

/**
 * @brief Facade over ROS2 navigation-related services.
 * 
 * Responsibility:
 *   Provide a clean synchronous interface for high-level navigation
 *   actions such as:
 *     - go to known location
 *     - register current pose as known location
 *
 * Design patterns applied:
 *   Facade          (GoF Structural)
 *     Hides ROS2 service complexity behind intention-revealing methods.
 *
 *   Template Method (GoF Behavioral)
 *     call_service<T>() centralises:
 *       async_send_request → wait_for → get → return
 *
 * Principles applied:
 *   SRP
 *     Navigation communication only.
 *
 *   DRY
 *     Timeout + async waiting logic implemented once.
 *
 *   DIP
 *     Callers depend on Navigation abstraction,
 *     not raw ROS2 clients.
 *
 * BioRobotics Laboratory, Facultad de Ingenieria, UNAM
 */

#include <chrono>
#include <future>
#include <memory>
#include <string>

#include "known_locations_interfaces/srv/locations_get.hpp"
#include "known_locations_interfaces/srv/locations_server.hpp"

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class Navigation
{
public:
    struct NavigationResult
    {
        bool success{false};
        std::string message;
    };

    explicit Navigation(const rclcpp::Node::SharedPtr& node);

    /**
     * @brief Navigate to a known location.
     *
     * @param location_name Known semantic location.
     * @param timeout Maximum waiting time.
     *
     * @return true if navigation succeeded.
     */
    bool GoTo(const std::string& location_name,
              std::chrono::seconds timeout = 300s);

    NavigationResult GoToDetailed(const std::string& location_name,
                                  std::chrono::seconds timeout = 300s);

    /**
     * @brief Save current robot pose as a known location.
     *
     * @param location_name Semantic location name.
     * @param timeout Maximum waiting time.
     *
     * @return true if operation succeeded.
     */
    bool AddLocation(const std::string& location_name,
                     std::chrono::seconds timeout = 5s);

private:

    rclcpp::Node::SharedPtr node_;

    rclcpp::CallbackGroup::SharedPtr cb_group_clients_;

    rclcpp::Client<
        known_locations_interfaces::srv::LocationsGet>::SharedPtr
            go_to_client_;

    rclcpp::Client<
        known_locations_interfaces::srv::LocationsServer>::SharedPtr
            add_location_client_;

    /**
     * @brief Template Method:
     *        async → wait → response extraction.
     *
     * @tparam ServiceT  ROS2 service type.
     * @tparam CallbackT Callable:
     *                   (Response::SharedPtr) → void
     */
    template<typename ServiceT, typename CallbackT>
    bool call_service(
        typename rclcpp::Client<ServiceT>::SharedPtr& client,
        typename ServiceT::Request::SharedPtr request,
        CallbackT on_response,
        std::chrono::seconds timeout,
        const std::string& label);
};

// ============================================================
// Template implementation
// ============================================================

template<typename ServiceT, typename CallbackT>
bool Navigation::call_service(
    typename rclcpp::Client<ServiceT>::SharedPtr& client,
    typename ServiceT::Request::SharedPtr request,
    CallbackT on_response,
    std::chrono::seconds timeout,
    const std::string& label)
{
    if (!client->wait_for_service(1s))
    {
        RCLCPP_ERROR(node_->get_logger(),
                     "%s service not available.",
                     label.c_str());
        return false;
    }

    auto future = client->async_send_request(request);

    if (future.wait_for(timeout) != std::future_status::ready)
    {
        RCLCPP_ERROR(node_->get_logger(),
                     "Timeout calling %s service after %ld s.",
                     label.c_str(),
                     timeout.count());

        return false;
    }

    try
    {
        on_response(future.get());
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(node_->get_logger(),
                     "%s service exception: %s",
                     label.c_str(),
                     e.what());

        return false;
    }

    return true;
}
