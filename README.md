# navigation_tools
This repository provides a utility class for navigation in ROS 2. 

## Description
The `Navigation` class acts as a client to the folowing service:
* `/nav_controller/go_to_location` [`known_locations_interfaces::srv::LocationsGet`]

Service timeout behavior:
* Waits up to **500 ms** for the service to be available.
* Waits up to **120 seconds** for a response once the request is sent.

The class sends a request to the navigation controller to move the robot to a known location by name.

## Behaviour
1. Waits for the navigation service to be available.
2. Sends the requested location name.
3. Blocks execution until a response is received or timeout occurs.
4. Returns:
   * `true` → Navigation succeeded.
   * `false` → Service unavailable, timeout, exception, or navigation failure.

The class internally handles:
* Asynchronous service calls
* Condition variable synchronization
* Timeout management
* Exception handling

## Interfaces
### Service Client
* `/nav_controller/go_to_location` [`known_locations_interfaces::srv::LocationsGet`]
Request:
* `location_name` [`string`]
Response:
* `success` [`bool`]

## Usage example
Include the header:
```cpp
#include "navigation_tools/Navigation.hpp"
```
Then:
```cpp
auto navigation = std::make_shared<Navigation>(node);

bool result = navigation->GoTo("kitchen");

if (result)
{
  RCLCPP_INFO(node->get_logger(), "Navigation succeeded");
}
else
{
  RCLCPP_WARN(node->get_logger(), "Navigation failed");
}
```
