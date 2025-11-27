#pragma once

#include <string>
#include <memory>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "known_locations_interfaces/srv/locations_server.hpp"

class Navigation
{
private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<known_locations_interfaces::srv::LocationsServer>::SharedPtr client_goal_;

public:
    Navigation(rclcpp::Node::SharedPtr node);

    bool GoTo(const std::string& location_name);
};
