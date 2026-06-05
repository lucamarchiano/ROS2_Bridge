# test_ros2_bridge.py
import numpy as np
import logging
import time as time_module
from px4_msgs.msg import VehicleOdometry
from pychrono_client.bridge import ROS2Bridge, ControlServiceException

logging.basicConfig(level=logging.INFO, format='%(levelname)s: %(message)s')
logger = logging.getLogger(__name__)

def main():
    time = 0.0
    time_sim = 5.0  # Let's test a clean 5-second simulated run
    dt = 0.01       # 100 Hz steps
    timeout_sec = 15.0
    
    ros2_bridge = ROS2Bridge()
    ros2_bridge.Initialize()
    
    step = 0
    try:
        while time < time_sim:
            step += 1
            logger.info(f"--- Step {step} (t={time:.2f}s) ---")
            logger.info(f"[Python Client] Current Sim Time: {time:.4f} s")

            # 1. Update the Clock First
            ros2_bridge.publish_sim_time(time)
            
            # Best Practice: Yield CPU for 1ms to allow the ROS2 network thread 
            # to safely flush the clock message out before sending the service request.
            time_module.sleep(0.001)
            
            # 2. Package Telemetry
            odometry = VehicleOdometry()
            odometry.timestamp = int(time * 1e6)
            odometry.pose_frame = VehicleOdometry.POSE_FRAME_NED
            odometry.velocity_frame = VehicleOdometry.VELOCITY_FRAME_NED
            odometry.position = [float(time), 0.0, -1.0]  # Simulating tracking movement
            odometry.angular_velocity = np.array([0.1, 0.1, 0.1], dtype=np.float32)
            
            # 3. Request Control (STRICT LOCK-STEP BLOCK)
            ros2_bridge.set_odometry_request(odometry=odometry)
            control = ros2_bridge.get_control_response(timeout_sec=timeout_sec)
            
            logger.info(f"✓ Control response computed for Sim Time: {time:.2f}s")
            
            # 4. Advance Physics Step Time
            time += dt
        
        logger.info(f"Simulation Finished! Stats: {ros2_bridge.get_latency_stats()}")
        
    except ControlServiceException as e:
        logger.error(f"Execution failed: {e}")
    finally:
        ros2_bridge.shutdown()

if __name__ == '__main__':
    main()