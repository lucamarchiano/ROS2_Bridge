import numpy as np
import logging

from px4_msgs.msg import VehicleOdometry

from ros2_bridge import ROS2Bridge, ControlServiceException

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(levelname)s: %(message)s'
)
logger = logging.getLogger(__name__)

def main():
    
    time = 0.0
    time_sim = 10.0
    dt = 0.01
    timeout_sec = 15.0
    
    # Initialize bridge
    ros2_bridge = ROS2Bridge()
    ros2_bridge.Initialize()
    
    step = 0
    try:

        while time < time_sim:
            step += 1
            logger.info(f"Step {step} (t={time:.2f}s)")
            
            odometry = VehicleOdometry()
            odometry.pose_frame = VehicleOdometry.POSE_FRAME_NED
            odometry.velocity_frame = VehicleOdometry.VELOCITY_FRAME_NED
            odometry.angular_velocity += np.array([0.1, 0.1, 0.1], dtype=np.float32)
            
            ros2_bridge.setOdometryRequest(odometry=odometry, current_time=time, dt=dt)
            control = ros2_bridge.getControlResponse(timeout_sec=timeout_sec, current_time=time, dt=dt)
            logger.info(f"✓ Control received: {control}")

            time += dt

        logger.info(f"Simulation completed. Status: {ros2_bridge.handler.get_status()}")

    
    except ControlServiceException as e:
        logger.error(f"✗ {e}")

    except KeyboardInterrupt:
        logger.info("Interrupted by user")


if __name__ == '__main__':
    main()
