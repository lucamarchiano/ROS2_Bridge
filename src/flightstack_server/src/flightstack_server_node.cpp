#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <condition_variable>
#include <mutex>
#include <fstream>
#include <filesystem>
#include <string>
#include <chrono>
#include "rosgraph_msgs/msg/clock.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "px4_msgs/msg/actuator_motors.hpp"
#include "bridge_interfaces/srv/bridge_step.hpp"

class SimulationBridgeServer : public rclcpp::Node
{
public:
    SimulationBridgeServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
        : Node("flightstack_server", options)
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
        service_server_ = this->create_service<bridge_interfaces::srv::BridgeStep>(
            "bridge_step",
            std::bind(&SimulationBridgeServer::handle_simulation_step, this, std::placeholders::_1, std::placeholders::_2),
            rclcpp::ServicesQoS(),
            callback_group_service_);

        RCLCPP_INFO(this->get_logger(), "Simulation Lockstep Bridge Proxy is fully operational on ROS 2 Jazzy.");
    }

private:

    void initialize_latency_logger() {
        // Read the exact log file path exported by Flightstack
        std::ifstream handshake("/tmp/flightstack_log_path.txt");
        
        if (handshake.is_open()) {
            std::string fs_log_path;
            std::getline(handshake, fs_log_path);
            handshake.close();

            if (!fs_log_path.empty()) {
                std::filesystem::path fs_path(fs_log_path);
                
                // Example fs_path: ./src/flightstack/log/20260814/PID/logs/log_20260814_070333.log
                // We use .parent_path().parent_path() to step up to the "PID" directory
                std::filesystem::path base_controller_dir = fs_path.parent_path().parent_path(); 
                
                // Create the new "latency_logs" folder next to "logs" and "gains"
                std::filesystem::path latency_dir = base_controller_dir / "latency_logs";
                if (!std::filesystem::exists(latency_dir)) {
                    std::filesystem::create_directories(latency_dir);
                }

                // Extract the filename (e.g., "log_20260814_070333.log")
                std::string filename = fs_path.filename().string();
                
                // Replace "log_" with "latency_log_"
                size_t pos = filename.find("log_");
                if (pos != std::string::npos) {
                    filename.replace(pos, 4, "latency_log_");
                }
                // Note: It already ends in .log, so we don't need to change the extension!

                // Construct final path
                std::filesystem::path final_path = latency_dir / filename;
                
                latency_logger_.open(final_path.string(), std::ios::out | std::ios::app);
                if (latency_logger_.is_open()) {
                    RCLCPP_INFO(this->get_logger(), "Paired latency logger natively with Flightstack: %s", final_path.string().c_str());
                    latency_logger_ << "Simulation_Time_us,PyChrono_RTT_ms,Cpp_Compute_ms,Network_Overhead_ms\n";
                    latency_logger_initialized_ = true;
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Failed to open latency log file at: %s", final_path.string().c_str());
                }
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "Waiting for Flightstack to export log path...");
        }
    }
    
    void handle_simulation_step(
        const std::shared_ptr<bridge_interfaces::srv::BridgeStep::Request> request,
        std::shared_ptr<bridge_interfaces::srv::BridgeStep::Response> response)
    {
        uint64_t current_sim_time_us = request->vehicle_odometry.timestamp;
        
        double pychrono_rtt_ms = request->previous_step_rtt_ms;

        {
            std::lock_guard<std::mutex> lock(mutex_);
            target_timestamp_us_ = current_sim_time_us;
            motor_data_received_ = false;
        }

        auto wall_start_time = std::chrono::steady_clock::now();

        px4_msgs::msg::VehicleOdometry odom_msg = request->vehicle_odometry;
        pub_odometry_->publish(odom_msg);

        rosgraph_msgs::msg::Clock clock_msg;
        clock_msg.clock.sec = current_sim_time_us / 1000000;
        clock_msg.clock.nanosec = (current_sim_time_us % 1000000) * 1000;
        pub_clock_->publish(clock_msg);

        std::unique_lock<std::mutex> lock(mutex_);
        bool success = cv_.wait_for(lock, std::chrono::milliseconds(1000), [this]() {
            return motor_data_received_;
        });

        // ADD THIS: Stop the Timer & Calculate Latencies
        auto wall_end_time = std::chrono::steady_clock::now();
        double cpp_turnaround_ms = std::chrono::duration<double, std::milli>(wall_end_time - wall_start_time).count();
        double network_overhead_ms = pychrono_rtt_ms - cpp_turnaround_ms;

        // ==========================================
        // DYNAMIC LATENCY LOGGING
        // ==========================================
        if (!latency_logger_initialized_) {
            initialize_latency_logger();
        }

        if (latency_logger_.is_open() && pychrono_rtt_ms > 0.0) {
            latency_logger_ << current_sim_time_us << ","
                            << pychrono_rtt_ms << ","
                            << cpp_turnaround_ms << ","
                            << network_overhead_ms << "\n";
            latency_logger_.flush();
        }

        if (success) {
            if (std::isnan(latest_motor_msg_.control[0])) {
                RCLCPP_ERROR(this->get_logger(), "t=%.3f | FlightStack actively replied with NaNs!", request->vehicle_odometry.timestamp / 1e6);
            }
            
            if (pychrono_rtt_ms > 0.0) {
                RCLCPP_INFO(this->get_logger(), 
                            "PyChrono RTT: %.3f ms | C++ Compute: %.3f ms | Network Overhead: %.3f ms", 
                            pychrono_rtt_ms, cpp_turnaround_ms, network_overhead_ms);
            }

            response->actuator_motors = latest_motor_msg_;
        } else {
            RCLCPP_ERROR(this->get_logger(), "t=%.3f | Bridge Timeout! FlightStack did not reply after %.3f ms.", request->vehicle_odometry.timestamp / 1e6, cpp_turnaround_ms);
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
    rclcpp::Service<bridge_interfaces::srv::BridgeStep>::SharedPtr service_server_;

    rclcpp::CallbackGroup::SharedPtr callback_group_service_;
    rclcpp::CallbackGroup::SharedPtr callback_group_sub_;

    // Thread synchronization resources
    std::mutex mutex_;
    std::condition_variable cv_;
    bool motor_data_received_ = false;
    uint64_t target_timestamp_us_ = 0;
    px4_msgs::msg::ActuatorMotors latest_motor_msg_;
    
    // Logger resources
    std::ofstream latency_logger_;
    bool latency_logger_initialized_ = false;
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