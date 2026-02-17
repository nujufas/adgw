#!/usr/bin/env python3
"""
Middleware Protocol - Binary serialization for C++ data structures

Matches the binary layout of:
- VehicleState (192 bytes)
- ControlCommand (variable, minimum ~400 bytes)
"""

import struct
import time
from typing import List, Dict, Any


class VehicleState:
    """
    VehicleState binary layout (192 bytes total):
    - Identity (16 bytes): id, timestamp
    - Pose (48 bytes): x, y, z, heading, pitch, roll
    - Velocity (48 bytes): vx, vy, vz, speed, acceleration, yaw_rate
    - Dimensions (32 bytes): length, width, height, wheelbase
    - Vehicle State (32 bytes): steering_angle, throttle, brake, gear
    - Flags (16 bytes): vehicle_type, control_mode, lights, reserved
    """
    
    SIZE = 192
    FORMAT = '<Qd 6d 6d 4d 4d 4I'  # Little-endian
    
    def __init__(self, data: bytes = None):
        if data:
            self.parse(data)
        else:
            self.id = 0
            self.timestamp = 0.0
            self.x = self.y = self.z = 0.0
            self.heading = self.pitch = self.roll = 0.0
            self.vx = self.vy = self.vz = 0.0
            self.speed = self.acceleration = self.yaw_rate = 0.0
            self.length = self.width = self.height = self.wheelbase = 0.0
            self.steering_angle = self.throttle = self.brake = self.gear = 0.0
            self.vehicle_type = self.control_mode = self.lights = self.reserved = 0
    
    def parse(self, data: bytes) -> None:
        """Parse binary data into VehicleState"""
        if len(data) < self.SIZE:
            raise ValueError(f"Insufficient data: {len(data)} < {self.SIZE}")
        
        values = struct.unpack(self.FORMAT, data[:self.SIZE])
        
        # Identity
        self.id = values[0]
        self.timestamp = values[1]
        
        # Pose
        self.x = values[2]
        self.y = values[3]
        self.z = values[4]
        self.heading = values[5]
        self.pitch = values[6]
        self.roll = values[7]
        
        # Velocity
        self.vx = values[8]
        self.vy = values[9]
        self.vz = values[10]
        self.speed = values[11]
        self.acceleration = values[12]
        self.yaw_rate = values[13]
        
        # Dimensions
        self.length = values[14]
        self.width = values[15]
        self.height = values[16]
        self.wheelbase = values[17]
        
        # Vehicle State
        self.steering_angle = values[18]
        self.throttle = values[19]
        self.brake = values[20]
        self.gear = values[21]
        
        # Flags
        self.vehicle_type = values[22]
        self.control_mode = values[23]
        self.lights = values[24]
        self.reserved = values[25]
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary for easy access"""
        return {
            'id': self.id,
            'timestamp': self.timestamp,
            'x': self.x,
            'y': self.y,
            'z': self.z,
            'heading': self.heading,
            'pitch': self.pitch,
            'roll': self.roll,
            'vx': self.vx,
            'vy': self.vy,
            'vz': self.vz,
            'speed': self.speed,
            'acceleration': self.acceleration,
            'yaw_rate': self.yaw_rate,
            'length': self.length,
            'width': self.width,
            'height': self.height,
            'wheelbase': self.wheelbase,
            'steering_angle': self.steering_angle,
            'throttle': self.throttle,
            'brake': self.brake,
            'gear': self.gear,
            'vehicle_type': self.vehicle_type,
            'control_mode': self.control_mode,
            'lights': self.lights
        }
    
    def __repr__(self) -> str:
        return (f"VehicleState(id={self.id}, x={self.x:.2f}, y={self.y:.2f}, "
                f"speed={self.speed:.2f}, heading={self.heading:.2f})")


class ControlCommand:
    """
    ControlCommand binary layout (minimum ~400 bytes):
    - Header (24 bytes): vehicleId, timestamp, sequenceNumber
    - Mode (4 bytes): enum Mode
    - Speed Control (32 bytes)
    - Acceleration Control (16 bytes)
    - Position Control (48 bytes)
    - Trajectory Control (256 bytes)
    - Steering Control (16 bytes)
    - Throttle/Brake Control (16 bytes)
    - Full State Control (64 bytes)
    """
    
    # Control modes (must match C++ enum)
    MODE_NONE = 0
    MODE_SPEED = 1
    MODE_ACCELERATION = 2
    MODE_POSITION = 3
    MODE_TRAJECTORY = 4
    MODE_STEERING = 5
    MODE_THROTTLE_BRAKE = 6
    MODE_FULL_STATE = 7
    
    def __init__(self):
        self.vehicleId = 0
        self.timestamp = 0.0
        self.sequenceNumber = 0
        self.mode = self.MODE_NONE
        
        # Speed control
        self.targetSpeed = 0.0
        self.speedTolerance = 0.5
        self.maxAcceleration = 3.0
        self.maxDeceleration = 5.0
        
        # Acceleration control
        self.targetAcceleration = 0.0
        self.duration = 0.0
        
        # Position control
        self.pos_x = self.pos_y = self.pos_z = 0.0
        self.pos_heading = self.pos_pitch = self.pos_roll = 0.0
        
        # Trajectory control
        self.numPoints = 0
        self.waypoints = []  # List of (x, y, speed) tuples
        
        # Steering control
        self.steeringAngle = 0.0
        self.steeringRate = 0.0
        
        # Throttle/brake control
        self.throttle = 0.0
        self.brake = 0.0
        
        # Full state control
        self.state_vx = self.state_vy = self.state_yawRate = 0.0
    
    def serialize(self) -> bytes:
        """Serialize to binary format matching C++ struct"""
        # Header (24 bytes)
        data = struct.pack('<Qdq', self.vehicleId, self.timestamp, self.sequenceNumber)
        
        # Mode (4 bytes)
        data += struct.pack('<I', self.mode)
        
        # Speed Control (32 bytes)
        data += struct.pack('<4d', self.targetSpeed, self.speedTolerance,
                           self.maxAcceleration, self.maxDeceleration)
        
        # Acceleration Control (16 bytes)
        data += struct.pack('<2d', self.targetAcceleration, self.duration)
        
        # Position Control (48 bytes)
        data += struct.pack('<6d', self.pos_x, self.pos_y, self.pos_z,
                           self.pos_heading, self.pos_pitch, self.pos_roll)
        
        # Trajectory Control (256 bytes)
        data += struct.pack('<2I', self.numPoints, 0)  # numPoints + padding
        for i in range(10):
            if i < len(self.waypoints):
                x, y, speed = self.waypoints[i]
                data += struct.pack('<3d', x, y, speed)
            else:
                data += struct.pack('<3d', 0.0, 0.0, 0.0)
        
        # Steering Control (16 bytes)
        data += struct.pack('<2d', self.steeringAngle, self.steeringRate)
        
        # Throttle/Brake Control (16 bytes)
        data += struct.pack('<2d', self.throttle, self.brake)
        
        # Full State Control (64 bytes) - simplified, just first 3 fields
        data += struct.pack('<3d', self.state_vx, self.state_vy, self.state_yawRate)
        data += b'\x00' * 40  # Padding for rest of full state
        
        return data
    
    @staticmethod
    def create_speed_command(vehicle_id: int, target_speed: float, 
                            timestamp: float = None) -> 'ControlCommand':
        """Create a speed control command (ACC mode)"""
        cmd = ControlCommand()
        cmd.vehicleId = vehicle_id
        cmd.timestamp = timestamp if timestamp is not None else time.time()
        cmd.sequenceNumber = 0
        cmd.mode = ControlCommand.MODE_SPEED
        cmd.targetSpeed = target_speed
        cmd.speedTolerance = 0.5
        cmd.maxAcceleration = 3.0
        cmd.maxDeceleration = 5.0
        return cmd
    
    @staticmethod
    def create_acceleration_command(vehicle_id: int, target_accel: float,
                                   duration: float = 1.0,
                                   timestamp: float = None) -> 'ControlCommand':
        """Create an acceleration control command"""
        cmd = ControlCommand()
        cmd.vehicleId = vehicle_id
        cmd.timestamp = timestamp if timestamp is not None else time.time()
        cmd.sequenceNumber = 0
        cmd.mode = ControlCommand.MODE_ACCELERATION
        cmd.targetAcceleration = target_accel
        cmd.duration = duration
        return cmd
    
    def __repr__(self) -> str:
        mode_names = ['NONE', 'SPEED', 'ACCELERATION', 'POSITION', 
                     'TRAJECTORY', 'STEERING', 'THROTTLE_BRAKE', 'FULL_STATE']
        mode_name = mode_names[self.mode] if self.mode < len(mode_names) else 'UNKNOWN'
        return (f"ControlCommand(vehicleId={self.vehicleId}, mode={mode_name}, "
                f"targetSpeed={self.targetSpeed:.2f})")


def parse_vehicle_data(data: bytes) -> List[VehicleState]:
    """
    Parse received UDP data containing multiple vehicles
    
    C++ Format: [count:uint32_t][vehicle1][vehicle2]...[vehicleN]
    
    Args:
        data: Binary data from UDP socket
        
    Returns:
        List of VehicleState objects
    """
    vehicles = []
    
    # Check minimum size (4 bytes for count)
    if len(data) < 4:
        return vehicles
    
    # Parse count (first 4 bytes, little-endian uint32)
    count = struct.unpack('<I', data[0:4])[0]
    
    # Verify we have enough data
    expected_size = 4 + (count * VehicleState.SIZE)
    if len(data) < expected_size:
        return vehicles
    
    # Parse vehicles
    offset = 4  # Start after count
    for i in range(count):
        if offset + VehicleState.SIZE <= len(data):
            vehicle = VehicleState(data[offset:offset + VehicleState.SIZE])
            vehicles.append(vehicle)
            offset += VehicleState.SIZE
    
    return vehicles


if __name__ == '__main__':
    # Test serialization
    print("Testing VehicleState parsing...")
    test_data = b'\x00' * VehicleState.SIZE
    v = VehicleState(test_data)
    print(f"  Parsed: {v}")
    print(f"  Size: {VehicleState.SIZE} bytes")
    
    print("\nTesting ControlCommand serialization...")
    cmd = ControlCommand.create_speed_command(vehicle_id=0, target_speed=25.0)
    data = cmd.serialize()
    print(f"  Created: {cmd}")
    print(f"  Serialized size: {len(data)} bytes")
    
    print("\nTesting parse_vehicle_data with count header...")
    # Create test data: [count=2][vehicle1][vehicle2]
    count_bytes = struct.pack('<I', 2)  # 2 vehicles
    vehicle1_bytes = b'\x01\x00\x00\x00\x00\x00\x00\x00' + b'\x00' * (VehicleState.SIZE - 8)  # id=1
    vehicle2_bytes = b'\x02\x00\x00\x00\x00\x00\x00\x00' + b'\x00' * (VehicleState.SIZE - 8)  # id=2
    test_packet = count_bytes + vehicle1_bytes + vehicle2_bytes
    vehicles = parse_vehicle_data(test_packet)
    print(f"  ✓ Parsed {len(vehicles)} vehicles from packet with count header")
    if len(vehicles) == 2:
        print(f"    Vehicle 0: id={vehicles[0].id}")
        print(f"    Vehicle 1: id={vehicles[1].id}")
    
    print("\nProtocol module ready!")
