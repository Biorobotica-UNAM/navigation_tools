#include "navigation_tools/Navigation.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <algorithm>
#include <cmath>
#include <thread>

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

	goal_pub_ =
		node_->create_publisher<
		    geometry_msgs::msg::PoseStamped>(
		        "/nav_control/goal",
		        rclcpp::QoS(10));
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

// ============================================================
// Go
// ============================================================

bool Navigation::Go(const geometry_msgs::msg::PoseStamped& goal)
{
    // 1. Disparamos la meta al tópico para despertar a mvn_pln y simple_move
    goal_pub_->publish(goal);

    RCLCPP_INFO(node_->get_logger(),
                "Navigation goal published via Go: [%.3f, %.3f]. Tracking 30s timeout...",
                goal.pose.position.x, goal.pose.position.y);

    // 2. Inicializamos herramientas locales de control de trayectoria
    auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
    
    // Publicador directo a los motores para poder interceptar el control físico si expira el tiempo
    auto cmd_vel_pub = node_->create_publisher<geometry_msgs::msg::Twist>("/hardware/base/cmd_vel", 10);

    const double kArrivalTolerance = 0.05; // Margen de éxito de llegada de 5 cm
    const auto kTimeoutDuration = std::chrono::seconds(30);
    auto start_time = std::chrono::steady_clock::now();
    
    bool arrived_in_time = false;

    // ── LAZO DE SEGUIMIENTO (Máximo 30 segundos) ──────────────────────────
    while (rclcpp::ok()) {
        // Criterio de corte: Se agotaron los 30 segundos en el reloj
        if (std::chrono::steady_clock::now() - start_time > kTimeoutDuration) {
            break; 
        }

        try {
            // Obtenemos la posición en vivo de Justina en el mapa
            auto tf_robot = tf_buffer->lookupTransform("map", "base_link", tf2::TimePointZero, 40ms);
            double rx = tf_robot.transform.translation.x;
            double ry = tf_robot.transform.translation.y;

            // Medimos la distancia euclidiana restante por Pitágoras
            double dx = goal.pose.position.x - rx;
            double dy = goal.pose.position.y - ry;
            double distance = std::sqrt(dx * dx + dy * dy);

            // Criterio de éxito por cercanía geométrica
            if (distance <= kArrivalTolerance) {
                arrived_in_time = true;
                break;
            }
        } 
        catch (const tf2::TransformException &) {
            // Ignoramos parpadeos o retrasos leves del transform_listener durante la marcha
        }

        std::this_thread::sleep_for(100ms); // Muestreo constante a 10 Hz
    }

    // Caso de Éxito: Llegó a tiempo al objetivo espacial
    if (arrived_in_time) {
        RCLCPP_INFO(node_->get_logger(), "[Navigation] Target reached successfully within 30s limit.");
        return true; 
    }

    // ── CASO DE INTERCEPCIÓN POR TIMEOUT (Maniobra de Emergencia) ──
    RCLCPP_WARN(node_->get_logger(), 
                "[Navigation] 30s Timeout exceeded! Overriding stack, braking and aligning to face target...");

    // 1. Mandamos un Twist en ceros para romper la inercia del controlador anterior
    geometry_msgs::msg::Twist stop_twist;
    cmd_vel_pub->publish(stop_twist);
    std::this_thread::sleep_for(200ms); // Pausa táctica para asentar la base del robot

    // 2. Parámetros de control proporcional para la rotación sobre su propio eje
    const double kAngleTolerance = 0.05; // ~3 grados de precisión angular de frente al objetivo
    const double kKpTheta = 1.3;         // Ganancia proporcional de giro
    const double kMaxAngVel = 0.4;       // Velocidad máxima de rotación regulada para evitar derrapes
    
    auto align_start_time = std::chrono::steady_clock::now();
    const auto kMaxAlignDuration = std::chrono::seconds(6); // Timeout de seguridad del bucle de giro

    // Lazo cerrado de alta frecuencia (20 Hz) para rotar hacia las coordenadas de la meta
    while (rclcpp::ok()) {
        if (std::chrono::steady_clock::now() - align_start_time > kMaxAlignDuration) {
            RCLCPP_ERROR(node_->get_logger(), "[Navigation] Emergency alignment timed out before completion.");
            break;
        }

        try {
            auto tf_robot = tf_buffer->lookupTransform("map", "base_link", tf2::TimePointZero, 40ms);
            double rx = tf_robot.transform.translation.x;
            double ry = tf_robot.transform.translation.y;

            // Calculamos el ángulo absoluto del vector que apunta al objetivo en el plano absoluto
            double dx = goal.pose.position.x - rx;
            double dy = goal.pose.position.y - ry;
            double angle_to_target = std::atan2(dy, dx);

            // Desarmamos el cuaternión de la TF para extraer el Yaw (orientación actual del robot)
            tf2::Quaternion q(
                tf_robot.transform.rotation.x,
                tf_robot.transform.rotation.y,
                tf_robot.transform.rotation.z,
                tf_robot.transform.rotation.w);
            tf2::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);

            // Calculamos el error angular remanente normalizándolo entre [-PI, +PI]
            double angle_error = angle_to_target - yaw;
            angle_error = std::atan2(std::sin(angle_error), std::cos(angle_error));

            // Si ya estamos de frente dentro de la tolerancia, rompemos el ciclo
            if (std::abs(angle_error) <= kAngleTolerance) {
                RCLCPP_INFO(node_->get_logger(), "[Navigation] Alignment completed. Final angular error: %.3f rad", angle_error);
                break;
            }

            // Inyectamos velocidad angular pura en el eje Z
            geometry_msgs::msg::Twist twist;
            twist.angular.z = std::clamp(kKpTheta * angle_error, -kMaxAngVel, kMaxAngVel);
            cmd_vel_pub->publish(twist);
        } 
        catch (const tf2::TransformException &) {
            std::this_thread::sleep_for(20ms);
        }

        std::this_thread::sleep_for(50ms); // 20 Hz
    }

    // 3. Forzamos un último comando de parada absoluto por seguridad
    cmd_vel_pub->publish(stop_twist);

    // Retornamos falso al supervisor para indicarle explícitamente el fallo por tiempo
    return false;
}
