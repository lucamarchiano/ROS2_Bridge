#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <condition_variable>
#include <mutex>
#include "rosgraph_msgs/msg/clock.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "px4_msgs/msg/actuator_motors.hpp"
#include "flightstack_server/srv/compute_control.hpp"

class SimulationBridgeServer : public rclcpp::Node
{
public:
    SimulationBridgeServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
        : Node("control_server", options)
    {
        // Allocate distinct, mutually exclusive callback groups to prevent self-deadlocking
        callback_group_service_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        callback_group_sub_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        // Set up subscription options for our custom callback thread
        rclcpp::SubscriptionOptions sub_opt;
        sub_opt.callback_group = callback_group_sub_;

        // Set up the system clock master and native flightstack communication interfaces
        pub_clock_ = this->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);
        pub_odometry_ = this->create_publisher<px4_msgs::msg::VehicleOdometry>("/fmu/out/vehicle_odometry", 10);
        
        sub_motors_ = this->create_subscription<px4_msgs::msg::ActuatorMotors>(
            "/fmu/in/actuator_motors", 10,
            std::bind(&SimulationBridgeServer::motor_topic_callback, this, std::placeholders::_1),
            sub_opt);

        // Host the service endpoint that PyChrono calls every physics step.
        // For ROS 2 Jazzy, we pass rclcpp::ServicesQoS() and our callback group directly as positional arguments.
        service_server_ = this->create_service<flightstack_server::srv::ComputeControl>(
            "compute_control",
            std::bind(&SimulationBridgeServer::handle_simulation_step, this, std::placeholders::_1, std::placeholders::_2),
            rclcpp::ServicesQoS(),
            callback_group_service_);

        RCLCPP_INFO(this->get_logger(), "Simulation Lockstep Bridge Proxy is fully operational on ROS 2 Jazzy.");
    }

private:
    void handle_simulation_step(
        const std::shared_ptr<flightstack_server::srv::ComputeControl::Request> request,
        std::shared_ptr<flightstack_server::srv::ComputeControl::Response> response)
    {
        uint64_t current_sim_time_us = request->vehicle_odometry.timestamp;

        // Reset tracking flags and lock the target timestamp for this discrete step
        {
            std::lock_guard<std::mutex> lock(mutex_);
            target_timestamp_us_ = current_sim_time_us;
            motor_data_received_ = false;
        }

        // SAFEGUARD: Publish Odometry state FIRST so the flightstack ingests the data
        px4_msgs::msg::VehicleOdometry odom_msg = request->vehicle_odometry;
        pub_odometry_->publish(odom_msg);

        // SAFEGUARD: Advance the global clock SECOND to trigger the flightstack's internal timer
        rosgraph_msgs::msg::Clock clock_msg;
        clock_msg.clock.sec = current_sim_time_us / 1000000;
        clock_msg.clock.nanosec = (current_sim_time_us % 1000000) * 1000;
        pub_clock_->publish(clock_msg);

        // Block this service thread until the flightstack's timer runs the math and publishes motors back
        std::unique_lock<std::mutex> lock(mutex_);
        bool finished_before_timeout = cv_.wait_for(lock, std::chrono::milliseconds(100), [this]() {
            return motor_data_received_;
        });

        if (finished_before_timeout) {
            response->actuator_motors = latest_motor_msg_;
        } else {
            RCLCPP_WARN(this->get_logger(), "SIL Bridge Timeout! Flightstack missed the synchronization window.");
            for (size_t i = 0; i < 12; ++i) {
                response->actuator_motors.control[i] = std::nanf("");
            }
        }
    }

    void motor_topic_callback(const px4_msgs::msg::ActuatorMotors::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        
        // SAFEGUARD: Drop any stale motor messages belonging to past simulation steps
        if (msg->timestamp >= target_timestamp_us_) {
            latest_motor_msg_ = *msg;
            motor_data_received_ = true;
            cv_.notify_one(); 
        }
    }

    // ROS 2 Comms handles
    rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr pub_clock_;
    rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr pub_odometry_;
    rclcpp::Subscription<px4_msgs::msg::ActuatorMotors>::SharedPtr sub_motors_;
    rclcpp::Service<flightstack_server::srv::ComputeControl>::SharedPtr service_server_;

    rclcpp::CallbackGroup::SharedPtr callback_group_service_;
    rclcpp::CallbackGroup::SharedPtr callback_group_sub_;

    // Thread synchronization resources
    std::mutex mutex_;
    std::condition_variable cv_;
    bool motor_data_received_ = false;
    uint64_t target_timestamp_us_ = 0;
    px4_msgs::msg::ActuatorMotors latest_motor_msg_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    
    // Bridge node must run with use_sim_time = false to act as the clock source
    rclcpp::NodeOptions options;
    options.append_parameter_override("use_sim_time", false);
    
    auto node = std::make_shared<SimulationBridgeServer>(options);
    
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    
    rclcpp::shutdown();
    return 0;
}