#include <cmath>      // For std::nanf
#include <rclcpp/rclcpp.hpp>
#include "px4_msgs/msg/actuator_motors.hpp"
#include "bridge_interfaces/srv/bridge_step.hpp"


class MinimalServer : public rclcpp::Node
{
public:
    MinimalServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
        : Node("flightstack_server", options)
    {
        service_ = this->create_service<bridge_interfaces::srv::BridgeStep>(
            "bridge_step",
            std::bind(&MinimalServer::handle_request, this, std::placeholders::_1, std::placeholders::_2)
        );
        RCLCPP_INFO(this->get_logger(), "BridgeStep service server is ready (C++)");
    }


private:
    void handle_request(
        const std::shared_ptr<bridge_interfaces::srv::BridgeStep::Request> request,
        std::shared_ptr<bridge_interfaces::srv::BridgeStep::Response> response)
    {
        // Check A: Extract the time packaged inside the message payload from Python
        double payload_time_sec = request->vehicle_odometry.timestamp / 1000000.0;
        RCLCPP_INFO(this->get_logger(), "[C++ Server] Incoming Payload Time: %.4f s", payload_time_sec);

        // Check B: Look at the actual ROS 2 Node's internal clock time
        double node_clock_sec = this->get_clock()->now().seconds();
        RCLCPP_INFO(this->get_logger(), "[C++ Server] Internal Node Clock Time: %.4f s", node_clock_sec);
        
        // LOG: Request arrived
        RCLCPP_INFO(this->get_logger(), "→ Received BridgeStep request");
        
        // LOG: Request data
        RCLCPP_INFO(this->get_logger(), 
            "  Position: [%.2f, %.2f, %.2f]",
            request->vehicle_odometry.position[0],
            request->vehicle_odometry.position[1],
            request->vehicle_odometry.position[2]);
        
        // Set default (disarmed) values to NaN for all 12 controls first
        for (size_t i = 0; i < 12; ++i) {
            response->actuator_motors.control[i] = std::nanf("");
        }

        // Set first 8 controls to some example value (0.2f here)
        for (size_t i = 0; i < 8; ++i) {
            response->actuator_motors.control[i] = 1.0f;
        }

        // Set timestamp fields in microseconds
        response->actuator_motors.timestamp = this->now().nanoseconds() / 1000;
        response->actuator_motors.timestamp_sample = response->actuator_motors.timestamp;

        response->actuator_motors.reversible_flags = 0; // No reversibles by default
        
        // LOG: Response ready
        RCLCPP_INFO(this->get_logger(), "← Sending response with 8 motors @ 0.2f");
    }

    rclcpp::Service<bridge_interfaces::srv::BridgeStep>::SharedPtr service_;
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions node_options;

    node_options.append_parameter_override("use_sim_time", true);

    rclcpp::spin(std::make_shared<MinimalServer>(node_options));
    rclcpp::shutdown();
    return 0;
}
