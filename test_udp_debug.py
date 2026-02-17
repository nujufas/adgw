#!/usr/bin/env python3
"""
Debug test - Check what data is available from middleware
"""

import socket
import struct
import select
import time

def test_udp_receive():
    """Test receiving UDP data"""
    
    # Configuration
    multicast_group = '239.255.0.1'
    port = 48198
    
    print("=" * 70)
    print("UDP Multicast Receiver Debug Test")
    print("=" * 70)
    print(f"Multicast group: {multicast_group}")
    print(f"Port: {port}")
    print()
    
    # Create socket
    print("Creating UDP socket...")
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    
    # Enable reuse
    print("Setting SO_REUSEADDR...")
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    
    # Bind to the port
    print(f"Binding to ('', {port})...")
    sock.bind(('', port))
    
    # Join multicast group
    print(f"Joining multicast group {multicast_group}...")
    mreq = struct.pack('4sL', socket.inet_aton(multicast_group), socket.INADDR_ANY)
    sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
    
    # Set timeout
    sock.settimeout(1.0)
    
    print()
    print("=" * 70)
    print("Socket ready - waiting for data...")
    print("(Make sure middleware is running: bazel-bin/examples/simple_demo)")
    print("=" * 70)
    print()
    
    received_count = 0
    start_time = time.time()
    
    try:
        while time.time() - start_time < 5.0:
            try:
                # Try to receive
                data, addr = sock.recvfrom(65536)
                received_count += 1
                
                print(f"[{received_count}] Received {len(data)} bytes from {addr}")
                
                # Parse the data
                if len(data) >= 4:
                    vehicle_count = struct.unpack('<I', data[0:4])[0]
                    print(f"    Vehicle count: {vehicle_count}")
                    print(f"    First 32 bytes (hex): {data[:32].hex()}")
                    
                    # Try to parse first vehicle
                    if len(data) >= 4 + 192 and vehicle_count > 0:
                        vehicle_id = struct.unpack('<Q', data[4:12])[0]
                        timestamp = struct.unpack('<d', data[12:20])[0]
                        x = struct.unpack('<d', data[20:28])[0]
                        y = struct.unpack('<d', data[28:36])[0]
                        print(f"    First vehicle: id={vehicle_id}, timestamp={timestamp:.3f}, x={x:.2f}, y={y:.2f}")
                else:
                    print(f"    Data too small to parse")
                
                print()
                
            except socket.timeout:
                elapsed = time.time() - start_time
                print(f"[{elapsed:.1f}s] No data (timeout)...")
                
    except KeyboardInterrupt:
        print("\nStopped by user")
    
    print()
    print("=" * 70)
    print(f"Test complete: Received {received_count} packets")
    print("=" * 70)
    
    sock.close()
    
    return received_count > 0

if __name__ == '__main__':
    success = test_udp_receive()
    exit(0 if success else 1)
