#!/usr/bin/env python3
"""
Quick test to verify ACC application can be imported and initialized
"""

import sys
sys.path.insert(0, '/home/nuju/repo/addemo/adgw/apps')

from middleware_protocol import VehicleState, ControlCommand, parse_vehicle_data
import struct

print("Testing imports...")
print("  ✓ VehicleState")
print("  ✓ ControlCommand")
print("  ✓ parse_vehicle_data")

print("\nTesting VehicleState creation...")
v = VehicleState()
v.id = 123
v.x = 10.5
v.speed = 20.0
print(f"  ✓ Created: {v}")

print("\nTesting ControlCommand creation...")
cmd = ControlCommand.create_speed_command(0, 25.0)
data = cmd.serialize()
print(f"  ✓ Created: {cmd}")
print(f"  ✓ Serialized: {len(data)} bytes")

print("\nTesting parse_vehicle_data...")
# Create fake vehicle data (2 vehicles)
fake_data = b'\x00' * VehicleState.SIZE * 2
vehicles = parse_vehicle_data(fake_data)
print(f"  ✓ Parsed {len(vehicles)} vehicles")

print("\nAll tests passed! ✓")
