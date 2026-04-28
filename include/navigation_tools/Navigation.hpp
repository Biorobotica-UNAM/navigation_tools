#pragma once

#include <string>
#include <chrono>
#include <memory>
#include <mutex>
#include <condition_variable>

#include "rclcpp/rclcpp.hpp"
#include "known_locations_interfaces/srv/locations_get.hpp"
#include "known_locations_interfaces/srv/locations_server.hpp"

class Navigation
{
public:
    explicit Navigation(const rclcpp::Node::SharedPtr& node);

    bool GoTo(const std::string& location_name);
    bool AddLocation(const std::string& location_name);

private:
    bool waitForService(
        const rclcpp::ClientBase::SharedPtr& client,
        const std::chrono::milliseconds& timeout) const;

private:
    rclcpp::Node::SharedPtr node_;

    rclcpp::Client<known_locations_interfaces::srv::LocationsGet>   ::SharedPtr client_goal_;
    rclcpp::Client<known_locations_interfaces::srv::LocationsServer>::SharedPtr client_add_location_;
};

