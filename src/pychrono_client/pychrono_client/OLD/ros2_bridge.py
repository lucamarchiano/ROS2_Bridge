import numpy as np
import time as time_module
import logging

import pychrono.ros as chronoros

from enum import Enum
from flightstack_server.srv import ComputeControl
from px4_msgs.msg import VehicleOdometry

"""ROS2 bridge to perform SIL/HIL simulation
    - Initialize() sets up the bridge
    - setOdometryRequest() sets the odometry request
    - getControlResponse() provides the control input response"""


# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(levelname)s: %(message)s'
)
logger = logging.getLogger(__name__)


class RequestState(Enum):
    """State machine for service request-response cycle"""
    IDLE = "idle"              # No request pending
    PENDING = "pending"        # Request sent, waiting for response
    RECEIVED = "received"      # Response received and fresh


class ControlServiceException(Exception):
    """Custom exception for domain-specific control service errors:
    - Manager update failures
    - Control response timeout
    These errors should stop the simulation"""
    pass

class ROS2Bridge:
    def __init__(self):
        self.manager = None
        self.handler = None

    def Initialize(self) -> None:
        """Initialize ROS2 bridge. Returns True if successful, False otherwise."""
        try:
            self.manager = chronoros.ChROSPythonManager()
            self.handler = ROS2ControlHandler()
            
            self.manager.RegisterHandler(self.handler)
            self.manager.RegisterHandler(chronoros.ChROSClockHandler())
            self.manager.Initialize()
            
            logger.info("✓ ROS2Bridge initialized successfully")
            
        except ControlServiceException as e:
            logger.error(f"✗ ROS2Bridge initialization failed: {e}")
            raise
  
    def setOdometryRequest(self, odometry, current_time, dt) -> None:
        """Set odometry message and trigger request"""
        self.handler._odometry = odometry
        self.handler._odometry_updated = True
        
        if not self.manager.Update(current_time, dt):  # ← Keep current_time constant!
            raise ControlServiceException("Manager update failed")
        

    def getControlResponse(self, current_time, dt, timeout_sec: float = 15.0) -> np.ndarray:
        """Block until control response available"""
        start_time = time_module.time()
        
        while self.handler._state != RequestState.RECEIVED:
            elapsed = time_module.time() - start_time
            
            if elapsed > timeout_sec:
                raise ControlServiceException(
                    f"Control response not received within {timeout_sec}s"
                )
            
            if not self.manager.Update(current_time, dt):  # ← Keep current_time constant!
                raise ControlServiceException("Manager update failed")
                
            time_module.sleep(0.001)
        
        self.handler._state = RequestState.IDLE
        return self.handler._control_response


class ROS2ControlHandler(chronoros.ChROSHandler):
    """Synchronous ROS2 service bridge for SIL/HIL simulation"""
    
    def __init__(self, update_rate_hz: int = 1000):
        super().__init__(update_rate_hz)
        self.client = None
        self.node = None
        
        self._state = RequestState.IDLE
        self._future = None
        self._odometry = None
        self._control_response = None
        self._request_count = 0
        self._response_count = 0
        self._odometry_updated = False

    def Initialize(self, interface: chronoros.ChROSInterface) -> bool:
        """Initialize ROS2 service client. Returns True if successful, False otherwise.
            Automatically called by ROS manager Initialize() method"""
        try:
            self._initialize_odometry()
            self._initialize_service_client(interface)
            logger.info("✓ ROS2ControlHandler initialized")
            return True
            
        except ControlServiceException as e:
            logger.error(f"✗ Initialization failed: {e}")
            return False

    def _initialize_odometry(self) -> None:
        """Initialize VehicleOdometry message. Returns True if successful."""
        self._odometry = VehicleOdometry()
        self._odometry.pose_frame = VehicleOdometry.POSE_FRAME_NED
        self._odometry.velocity_frame = VehicleOdometry.VELOCITY_FRAME_NED

    def _initialize_service_client(self, interface: chronoros.ChROSInterface) -> None:
        """Create and validate ROS2 service client. Returns True if successful."""
        self.node = interface.GetNode()
        self.client = self.node.create_client(ComputeControl, "compute_control")
        
        timeout_sec = 1.0
        max_attempts = 15
        for attempt in range(max_attempts):
            if self.client.wait_for_service(timeout_sec=timeout_sec):
                logger.info("✓ compute_control service ready")
                return
            logger.info(f"  Waiting for service ({attempt + 1}/{max_attempts})")
        
        raise ControlServiceException("compute_control service not available")
        

    def Tick(self, time: float) -> None:
        """Called by ROS manager to process requests/responses"""
        # Poll response if pending
        if self._state == RequestState.PENDING and self._future is not None:
            self._poll_response()
        
        # Send request if odometry updated
        if self._odometry_updated and self._state == RequestState.IDLE:
            self._dispatch_request()
            self._odometry_updated = False

    def _dispatch_request(self) -> None:
        """Send async service request"""
        if self._odometry is None:
            return
        
        request = ComputeControl.Request()
        request.vehicle_odometry = self._odometry
        
        self._future = self.client.call_async(request)
        self._state = RequestState.PENDING
        self._request_count += 1

    def _poll_response(self) -> None:
        """Check if service response arrived"""
        if not self._future.done():
            return
        
        try:
            response = self._future.result()
            self._control_response = response.actuator_motors
            self._state = RequestState.RECEIVED
            self._response_count += 1
        except Exception as e:
            self._state = RequestState.IDLE
            logger.error(f"Service call failed: {e}") 
        finally:
            self._future = None

    def get_status(self) -> dict:
        return {
            "state": self._state.value,
            "request_count": self._request_count,
            "response_count": self._response_count,
            "latest_control": self._control_response,
        }

    def SupportsIncomingMessages(self) -> bool:
        """Service clients don't consume incoming messages"""
        return False
