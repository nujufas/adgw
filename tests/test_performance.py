#!/usr/bin/env python3
"""
Performance Benchmark Suite for Phase 6

Measures:
- Latency (perception → control → actuation)
- Throughput (messages/sec, bytes/sec)
- CPU and memory usage
- Real-time factor stability
- Extended run stability
"""

import socket
import struct
import time
import os
import sys
import signal
import subprocess
from collections import deque
from dataclasses import dataclass
from typing import List, Tuple, Optional

# Optional: psutil for CPU/memory monitoring
try:
    import psutil
    PSUTIL_AVAILABLE = True
except ImportError:
    PSUTIL_AVAILABLE = False
    print("Warning: psutil not available, CPU/memory monitoring disabled")

sys.path.insert(0, 'apps')
from middleware_protocol import parse_vehicle_data, ControlCommand


@dataclass
class PerformanceMetrics:
    """Performance measurement results"""
    test_duration: float
    frames_received: int
    commands_sent: int
    average_rate: float
    min_rate: float
    max_rate: float
    latency_avg: float
    latency_min: float
    latency_max: float
    cpu_usage_avg: float
    memory_mb: float
    real_time_factor: float
    packet_loss: int


class PerformanceTester:
    """Performance testing harness"""
    
    def __init__(self, multicast_group='239.255.0.1', receive_port=48198,
                 send_host='127.0.0.1', send_port=53995):
        self.multicast_group = multicast_group
        self.receive_port = receive_port
        self.send_host = send_host
        self.send_port = send_port
        
        self.running = False
        self.middleware_proc = None
        
        # Metrics
        self.frames_received = 0
        self.commands_sent = 0
        self.latencies = deque(maxlen=1000)
        self.frame_intervals = deque(maxlen=1000)
        self.last_frame_time = 0
        
        # Setup sockets
        self._setup_sockets()
    
    def _setup_sockets(self):
        """Set up UDP sockets"""
        # Receive socket (multicast)
        self.recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.recv_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.recv_sock.bind(('', self.receive_port))
        
        # Join multicast group
        mreq = struct.pack('4sl', socket.inet_aton(self.multicast_group),
                          socket.INADDR_ANY)
        self.recv_sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
        self.recv_sock.settimeout(0.1)  # 100ms timeout
        
        # Send socket
        self.send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    def start_middleware(self):
        """Start the middleware process"""
        print("Starting middleware...")
        self.middleware_proc = subprocess.Popen(
            ['./bazel-bin/examples/simple_demo'],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL
        )
        time.sleep(2)  # Give it time to initialize
        
        if self.middleware_proc.poll() is not None:
            raise RuntimeError("Middleware failed to start")
        
        print(f"  Middleware PID: {self.middleware_proc.pid}")
    
    def stop_middleware(self):
        """Stop the middleware process"""
        if self.middleware_proc:
            self.middleware_proc.terminate()
            try:
                self.middleware_proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.middleware_proc.kill()
                self.middleware_proc.wait()
    
    def run_test(self, duration: float) -> PerformanceMetrics:
        """Run performance test for specified duration"""
        print(f"\nRunning performance test for {duration:.1f} seconds...")
        print("=" * 70)
        
        self.running = True
        start_time = time.time()
        last_stats_time = start_time
        
        # Get process for CPU/memory monitoring
        if self.middleware_proc and PSUTIL_AVAILABLE:
            try:
                proc = psutil.Process(self.middleware_proc.pid)
            except:
                proc = None
        else:
            proc = None
        
        cpu_samples = []
        memory_samples = []
        
        try:
            while time.time() - start_time < duration:
                # Try to receive data
                try:
                    data, _ = self.recv_sock.recvfrom(65536)
                    
                    # Filter for vehicle data (>= 196 bytes)
                    if len(data) < 196:
                        continue
                    
                    receive_time = time.time()
                    vehicles = parse_vehicle_data(data)
                    
                    if len(vehicles) < 1:
                        continue
                    
                    self.frames_received += 1
                    
                    # Calculate latency (assuming timestamp is in simulation time)
                    ego = vehicles[0]
                    latency = receive_time - ego.timestamp if ego.timestamp > 0 else 0
                    if latency > 0 and latency < 1.0:  # Reasonable range
                        self.latencies.append(latency)
                    
                    # Calculate frame interval
                    if self.last_frame_time > 0:
                        interval = receive_time - self.last_frame_time
                        self.frame_intervals.append(interval)
                    self.last_frame_time = receive_time
                    
                    # Send control command
                    cmd = ControlCommand.create_speed_command(
                        vehicle_id=ego.id,
                        target_speed=20.0,
                        timestamp=ego.timestamp
                    )
                    self.send_sock.sendto(cmd.serialize(),
                                         (self.send_host, self.send_port))
                    self.commands_sent += 1
                    
                except socket.timeout:
                    pass
                
                # Sample CPU/memory every second
                current_time = time.time()
                if proc and current_time - last_stats_time >= 1.0:
                    try:
                        cpu_samples.append(proc.cpu_percent())
                        memory_samples.append(proc.memory_info().rss / 1024 / 1024)  # MB
                    except:
                        pass
                    last_stats_time = current_time
                    
                    # Print interim stats
                    elapsed = current_time - start_time
                    rate = self.frames_received / elapsed
                    print(f"[{elapsed:5.1f}s] Frames: {self.frames_received:5d}, "
                          f"Rate: {rate:6.1f} Hz, Commands: {self.commands_sent:5d}")
        
        except KeyboardInterrupt:
            print("\nTest interrupted by user")
        
        finally:
            self.running = False
        
        # Calculate metrics
        elapsed = time.time() - start_time
        
        # Frame rate statistics
        avg_rate = self.frames_received / elapsed if elapsed > 0 else 0
        if self.frame_intervals:
            instantaneous_rates = [1.0/i for i in self.frame_intervals if i > 0]
            min_rate = min(instantaneous_rates) if instantaneous_rates else 0
            max_rate = max(instantaneous_rates) if instantaneous_rates else 0
        else:
            min_rate = max_rate = 0
        
        # Latency statistics
        if self.latencies:
            latency_avg = sum(self.latencies) / len(self.latencies)
            latency_min = min(self.latencies)
            latency_max = max(self.latencies)
        else:
            latency_avg = latency_min = latency_max = 0
        
        # CPU/memory
        cpu_avg = sum(cpu_samples) / len(cpu_samples) if cpu_samples else 0
        memory_mb = sum(memory_samples) / len(memory_samples) if memory_samples else 0
        
        # Real-time factor (assuming 100 Hz target)
        real_time_factor = avg_rate / 100.0 if avg_rate > 0 else 0
        
        # Packet loss (assuming ~100 Hz for duration)
        expected_frames = int(duration * 100)
        packet_loss = max(0, expected_frames - self.frames_received)
        
        return PerformanceMetrics(
            test_duration=elapsed,
            frames_received=self.frames_received,
            commands_sent=self.commands_sent,
            average_rate=avg_rate,
            min_rate=min_rate,
            max_rate=max_rate,
            latency_avg=latency_avg * 1000,  # Convert to ms
            latency_min=latency_min * 1000,
            latency_max=latency_max * 1000,
            cpu_usage_avg=cpu_avg,
            memory_mb=memory_mb,
            real_time_factor=real_time_factor,
            packet_loss=packet_loss
        )
    
    def cleanup(self):
        """Clean up resources"""
        self.recv_sock.close()
        self.send_sock.close()


def print_metrics(metrics: PerformanceMetrics, test_name: str):
    """Print performance metrics"""
    print()
    print("=" * 70)
    print(f"Performance Test Results: {test_name}")
    print("=" * 70)
    print(f"  Duration:            {metrics.test_duration:.2f} s")
    print(f"  Frames received:     {metrics.frames_received}")
    print(f"  Commands sent:       {metrics.commands_sent}")
    print()
    print(f"  Average rate:        {metrics.average_rate:.2f} Hz")
    print(f"  Min rate:            {metrics.min_rate:.2f} Hz")
    print(f"  Max rate:            {metrics.max_rate:.2f} Hz")
    print()
    print(f"  Latency (avg):       {metrics.latency_avg:.3f} ms")
    print(f"  Latency (min):       {metrics.latency_min:.3f} ms")
    print(f"  Latency (max):       {metrics.latency_max:.3f} ms")
    print()
    print(f"  CPU usage:           {metrics.cpu_usage_avg:.1f} %")
    print(f"  Memory usage:        {metrics.memory_mb:.1f} MB")
    print()
    print(f"  Real-time factor:    {metrics.real_time_factor:.3f}x")
    print(f"  Packet loss:         {metrics.packet_loss} frames")
    print("=" * 70)


def main():
    """Main test runner"""
    print("=" * 70)
    print("Phase 6: Performance Benchmark Suite")
    print("=" * 70)
    
    # Change to project root
    os.chdir('/home/nuju/repo/addemo/adgw')
    
    # Build middleware
    print("\nBuilding middleware...")
    result = subprocess.run(['bazel', 'build', '//examples:simple_demo'],
                          capture_output=True, text=True)
    if result.returncode != 0:
        print(f"Build failed:\n{result.stderr}")
        return 1
    print("✓ Build successful")
    
    # Run tests
    results = []
    
    try:
        # Test 1: Short burst (10 seconds)
        print("\n" + "=" * 70)
        print("Test 1: Short Burst (10 seconds)")
        print("=" * 70)
        tester = PerformanceTester()
        tester.start_middleware()
        metrics1 = tester.run_test(10.0)
        tester.stop_middleware()
        tester.cleanup()
        print_metrics(metrics1, "Short Burst")
        results.append(("Short Burst (10s)", metrics1))
        
        time.sleep(2)
        
        # Test 2: Medium run (30 seconds)
        print("\n" + "=" * 70)
        print("Test 2: Medium Run (30 seconds)")
        print("=" * 70)
        tester = PerformanceTester()
        tester.start_middleware()
        metrics2 = tester.run_test(30.0)
        tester.stop_middleware()
        tester.cleanup()
        print_metrics(metrics2, "Medium Run")
        results.append(("Medium Run (30s)", metrics2))
        
        time.sleep(2)
        
        # Test 3: Extended run (60 seconds)
        print("\n" + "=" * 70)
        print("Test 3: Extended Run (60 seconds)")
        print("=" * 70)
        tester = PerformanceTester()
        tester.start_middleware()
        metrics3 = tester.run_test(60.0)
        tester.stop_middleware()
        tester.cleanup()
        print_metrics(metrics3, "Extended Run")
        results.append(("Extended Run (60s)", metrics3))
        
    except Exception as e:
        print(f"\nError during testing: {e}")
        return 1
    
    # Summary
    print("\n" + "=" * 70)
    print("Summary of All Tests")
    print("=" * 70)
    print(f"{'Test':<20} {'Rate':>10} {'Latency':>10} {'CPU':>8} {'RTF':>8}")
    print("-" * 70)
    for name, metrics in results:
        print(f"{name:<20} {metrics.average_rate:>9.1f}Hz "
              f"{metrics.latency_avg:>9.2f}ms {metrics.cpu_usage_avg:>7.1f}% "
              f"{metrics.real_time_factor:>7.3f}x")
    print("=" * 70)
    
    # Pass/fail criteria
    print("\nPass/Fail Criteria:")
    all_passed = True
    
    for name, metrics in results:
        passed = True
        reasons = []
        
        if metrics.average_rate < 80.0:
            passed = False
            reasons.append(f"Low rate: {metrics.average_rate:.1f} Hz < 80 Hz")
        
        if metrics.latency_avg > 50.0:
            passed = False
            reasons.append(f"High latency: {metrics.latency_avg:.1f} ms > 50 ms")
        
        if metrics.real_time_factor < 0.8:
            passed = False
            reasons.append(f"Low RTF: {metrics.real_time_factor:.3f} < 0.8")
        
        status = "✓ PASS" if passed else "✗ FAIL"
        print(f"  {name:<20} {status}")
        if not passed:
            for reason in reasons:
                print(f"    - {reason}")
            all_passed = False
    
    print()
    if all_passed:
        print("✓ All performance tests PASSED!")
        return 0
    else:
        print("✗ Some performance tests FAILED")
        return 1


if __name__ == '__main__':
    sys.exit(main())
