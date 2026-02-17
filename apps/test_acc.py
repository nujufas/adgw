#!/usr/bin/env python3
"""
Test ACC (Adaptive Cruise Control) Application

Validates middleware bidirectional communication:
- Receives vehicle perception data via UDP multicast
- Implements simple ACC control logic
- Sends control commands back to middleware

Usage:
    python3 apps/test_acc.py
    
    (Press Ctrl+C to stop)
"""

import socket
import struct
import time
import signal
import sys
from middleware_protocol import parse_vehicle_data, ControlCommand


class TestACCApp:
    """Simple ACC application for testing closed-loop control"""
    
    def __init__(self, 
                 multicast_group='239.255.0.1',
                 receive_port=48198,
                 send_host='127.0.0.1',
                 send_port=53995):
        """
        Initialize ACC application
        
        Args:
            multicast_group: Multicast group for receiving perception data
            receive_port: Port to receive perception data
            send_host: Host to send control commands
            send_port: Port to send control commands
        """
        self.multicast_group = multicast_group
        self.receive_port = receive_port
        self.send_host = send_host
        self.send_port = send_port
        
        self.running = False
        self.frame_count = 0
        self.commands_sent = 0
        self.start_time = 0.0
        
        # ACC parameters
        self.cruise_speed = 20.0        # Target cruise speed [m/s]
        self.time_gap = 2.0              # Desired time gap to lead vehicle [s]
        self.min_distance = 10.0         # Minimum safe distance [m]
        self.max_acceleration = 3.0      # Max acceleration [m/s²]
        self.max_deceleration = 5.0      # Max deceleration [m/s²]
        
        self._setup_sockets()
        self._setup_signal_handlers()
    
    def _setup_sockets(self):
        """Set up UDP sockets for receiving and sending"""
        # Receive socket (multicast)
        self.recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.recv_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.recv_sock.bind(('', self.receive_port))
        
        # Join multicast group
        mreq = struct.pack('4sl', socket.inet_aton(self.multicast_group), 
                          socket.INADDR_ANY)
        self.recv_sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
        
        # Set timeout for graceful shutdown
        self.recv_sock.settimeout(1.0)
        
        # Send socket
        self.send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        print(f"ACC Application Initialized")
        print(f"  Receiving:  UDP {self.multicast_group}:{self.receive_port} (multicast)")
        print(f"  Sending to: UDP {self.send_host}:{self.send_port}")
        print(f"  ACC Config: cruise={self.cruise_speed:.1f} m/s, time_gap={self.time_gap:.1f} s")
        print()
    
    def _setup_signal_handlers(self):
        """Set up signal handlers for graceful shutdown"""
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)
    
    def _signal_handler(self, signum, frame):
        """Handle shutdown signals"""
        print("\n\nShutdown signal received...")
        self.running = False
    
    def run(self):
        """Main application loop"""
        self.running = True
        self.start_time = time.time()
        last_log_time = self.start_time
        
        print("ACC Application Running (Press Ctrl+C to stop)")
        print("=" * 70)
        
        try:
            while self.running:
                try:
                    # 1. Receive perception data
                    data, _ = self.recv_sock.recvfrom(65536)
                    
                    # Filter: Only process vehicle data packets
                    # Vehicle packets have format: [count:4][vehicles:192*N]
                    # Minimum size for 1 vehicle: 4 + 192 = 196 bytes
                    # We expect 2 vehicles: 4 + 192*2 = 388 bytes
                    if len(data) < 196:
                        # Skip scene data or other short packets
                        continue
                    
                    vehicles = parse_vehicle_data(data)
                    
                    if len(vehicles) < 1:
                        continue
                    
                    self.frame_count += 1
                    
                    # 2. Get ego vehicle (assumed to be first)
                    ego = vehicles[0]
                    
                    # 3. Compute ACC control
                    target_speed = self._compute_acc_control(ego, vehicles)
                    
                    # 4. Send control command
                    self._send_control_command(ego.id, target_speed, ego.timestamp)
                    
                    # 5. Periodic logging
                    current_time = time.time()
                    if current_time - last_log_time >= 1.0:
                        self._log_status(ego, vehicles, target_speed, current_time - self.start_time)
                        last_log_time = current_time
                
                except socket.timeout:
                    # Timeout is expected for graceful shutdown
                    continue
                except Exception as e:
                    print(f"Error in main loop: {e}")
                    continue
        
        finally:
            self._shutdown()
    
    def _compute_acc_control(self, ego, vehicles):
        """
        Compute ACC control logic
        
        Simple algorithm:
        1. If no lead vehicle: cruise at target speed
        2. If lead vehicle exists:
           - Calculate following distance and time gap
           - If too close: decelerate
           - If too far: accelerate toward cruise speed
           - Otherwise: match lead speed
        
        Args:
            ego: Ego vehicle state
            vehicles: List of all vehicles
            
        Returns:
            Target speed [m/s]
        """
        target_speed = self.cruise_speed
        
        # Find lead vehicle (assumes vehicles[1] is lead if it exists)
        if len(vehicles) > 1:
            lead = vehicles[1]
            
            # Calculate relative distance
            dx = lead.x - ego.x
            dy = lead.y - ego.y
            distance = (dx**2 + dy**2)**0.5
            
            # Calculate desired distance based on time gap
            desired_distance = max(self.min_distance, ego.speed * self.time_gap)
            
            # Simple proportional control
            distance_error = distance - desired_distance
            
            if distance_error < -5.0:
                # Too close - decelerate aggressively
                target_speed = max(ego.speed - 2.0, lead.speed - 1.0, 5.0)
            elif distance_error < 0.0:
                # Slightly too close - gentle deceleration
                target_speed = max(ego.speed - 0.5, lead.speed, 10.0)
            elif distance_error < 10.0:
                # Good distance - match lead speed
                target_speed = lead.speed
            else:
                # Too far - accelerate toward cruise speed
                target_speed = min(self.cruise_speed, ego.speed + 1.0)
        
        # Clamp to reasonable values
        target_speed = max(0.0, min(self.cruise_speed, target_speed))
        
        return target_speed
    
    def _send_control_command(self, vehicle_id, target_speed, timestamp):
        """Send control command to middleware"""
        cmd = ControlCommand.create_speed_command(
            vehicle_id=vehicle_id,
            target_speed=target_speed,
            timestamp=timestamp
        )
        
        data = cmd.serialize()
        self.send_sock.sendto(data, (self.send_host, self.send_port))
        self.commands_sent += 1
    
    def _log_status(self, ego, vehicles, target_speed, elapsed_time):
        """Log current status"""
        lead_info = ""
        if len(vehicles) > 1:
            lead = vehicles[1]
            dx = lead.x - ego.x
            dy = lead.y - ego.y
            distance = (dx**2 + dy**2)**0.5
            time_gap = distance / max(ego.speed, 0.1)
            lead_info = f"lead_x={lead.x:7.2f} lead_speed={lead.speed:5.2f} dist={distance:6.2f} gap={time_gap:4.2f}s"
        else:
            lead_info = "no_lead_vehicle"
        
        print(f"[{elapsed_time:6.1f}s] Frame {self.frame_count:5d} | "
              f"ego_x={ego.x:7.2f} ego_speed={ego.speed:5.2f} target={target_speed:5.2f} | "
              f"{lead_info}")
    
    def _shutdown(self):
        """Clean shutdown"""
        elapsed = time.time() - self.start_time if self.start_time > 0 else 0
        
        print()
        print("=" * 70)
        print("ACC Application Shutdown")
        print(f"  Total frames:    {self.frame_count}")
        print(f"  Commands sent:   {self.commands_sent}")
        
        if elapsed > 0 and self.frame_count > 0:
            print(f"  Runtime:         {elapsed:.1f} s")
            print(f"  Average rate:    {self.frame_count / elapsed:.1f} Hz")
        
        self.recv_sock.close()
        self.send_sock.close()
        
        print("\nGoodbye!")


def main():
    """Main entry point"""
    # Default configuration matches simple_demo.cpp
    app = TestACCApp(
        multicast_group='239.255.0.1',
        receive_port=48198,
        send_host='127.0.0.1',
        send_port=53995
    )
    
    app.run()


if __name__ == '__main__':
    main()
