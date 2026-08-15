# ros2_bridge_sync.py
import numpy as np
import logging
import rclpy
import time as time_module
import threading
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
# from rosgraph_msgs.msg import Clock
from bridge_interfaces.srv import BridgeStep
from px4_msgs.msg import VehicleOdometry

logging.basicConfig(level=logging.INFO, format='%(levelname)s: %(message)s')
logger = logging.getLogger(__name__)

class ControlServiceException(Exception):
    pass

class ROS2ControlNode(Node):
    def __init__(self):
        super().__init__('pychrono_simulator')
        
        self.client_cb_group = MutuallyExclusiveCallbackGroup()

        # Create service client
        self.client = self.create_client(
            BridgeStep, "bridge_step", callback_group=self.client_cb_group
        )
        
        if not self.client.wait_for_service(timeout_sec=10.0):
            raise ControlServiceException("bridge_step service not available")
        logger.info("✓ bridge_step service ready")
    

class ROS2Bridge:
    def __init__(self):
        self.node = None
        self.initialized = False
        self.latency_history = []
        self.executor = None
        self.executor_thread = None
        self.clock_pub = None
        self._odometry = None
        self._request_start_time = 0.0
        self._last_rtt_ms = 0.0
    
    def Initialize(self):
        if not rclpy.ok():
            rclpy.init()
        
        self.node = ROS2ControlNode()
        
        # CREATE EXECUTOR AND START BACKGROUND SPIN THREAD
        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.node)
        
        self.executor_thread = threading.Thread(
            target=self._spin_executor,
            daemon=True
        )
        self.executor_thread.start()
        
        self.initialized = True
        logger.info("✓ ROS2Bridge initialized (with background executor)")

    def _spin_executor(self):
        """Background thread: continuously spin executor to process responses"""
        try:
            self.executor.spin()
        except Exception as e:
            logger.error(f"Executor error: {e}")
    
    def set_odometry_request(self, odometry: VehicleOdometry):
        self._odometry = odometry
        self._request_start_time = time_module.perf_counter()
    
    def get_control_response(self, timeout_sec: float = 15.0) -> np.ndarray:
        if not self.initialized:
            raise RuntimeError("Must call Initialize() first")
    
        future = self.node.client.call_async(BridgeStep.Request(
            vehicle_odometry=self._odometry,
            previous_step_rtt_ms=self._last_rtt_ms
        ))
        
        response_received = threading.Event()
        
        def done_callback(_future):
            response_received.set()
            
        future.add_done_callback(done_callback)
        
        # Main simulation execution blocks here cleanly
        if not response_received.wait(timeout=timeout_sec):
            raise ControlServiceException("Service call timed out")
            
        response = future.result()
        if response is None:
            raise ControlServiceException("Service returned empty response")
            
        request_to_control_ms = (time_module.perf_counter() - self._request_start_time) * 1000
        self.latency_history.append(request_to_control_ms)
        self._last_rtt_ms = float(request_to_control_ms)
        
        return response.actuator_motors
    
    def get_latency_stats(self) -> dict:
        if not self.latency_history:
            return {"error": "No measurements"}
        latencies = np.array(self.latency_history)
        return {
            'count': len(latencies),
            'min_ms': float(latencies.min()),
            'max_ms': float(latencies.max()),
            'mean_ms': float(latencies.mean()),
        }
    
    def shutdown(self):
        if self.executor:
            self.executor.shutdown()
        if self.node:
            self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if self.executor_thread:
            self.executor_thread.join(timeout=1.0)
