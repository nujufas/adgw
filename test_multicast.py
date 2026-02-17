#!/usr/bin/env python3
"""
Simple UDP multicast receiver test
"""

import socket
import struct
import time

multicast_group = '239.255.0.1'
port = 48198

# Create socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

# Bind to port
sock.bind(('', port))

# Join multicast group
mreq = struct.pack('4sl', socket.inet_aton(multicast_group), socket.INADDR_ANY)
sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)

# Set timeout
sock.settimeout(2.0)

print(f"Listening on {multicast_group}:{port}")
print("Waiting for data...")

try:
    for i in range(10):
        try:
            data, addr = sock.recvfrom(65536)
            print(f"Received {len(data)} bytes from {addr}")
            
            # Parse count
            if len(data) >= 4:
                count = struct.unpack('<I', data[0:4])[0]
                print(f"  Count: {count} vehicles")
                print(f"  First 20 bytes: {data[:20].hex()}")
        except socket.timeout:
            print(f"  [{i}] No data (timeout)")
except KeyboardInterrupt:
    print("\nStopped")

sock.close()
