#!/usr/bin/env python3
"""
Integrated test - Verify closed-loop communication

This script:
1. Starts the middleware in background
2. Starts the ACC application
3. Runs for 10 seconds
4. Stops both and reports results
"""

import subprocess
import time
import sys
import os
import signal

def main():
    print("=" * 70)
    print("Phase 5: Integrated Closed-Loop Test")
    print("=" * 70)
    print()
    
    # Change to project root
    os.chdir('/home/nuju/repo/addemo/adgw')
    
    # Build first
    print("Building middleware...")
    result = subprocess.run(['bazel', 'build', '//examples:simple_demo'],
                          capture_output=True, text=True)
    if result.returncode != 0:
        print(f"Build failed:\n{result.stderr}")
        return 1
    print("✓ Build successful\n")
    
    middleware_proc = None
    acc_proc = None
    
    try:
        # Start middleware
        print("Starting middleware (simple_demo)...")
        middleware_proc = subprocess.Popen(
            ['./bazel-bin/examples/simple_demo'],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1
        )
        
        # Give it time to initialize
        time.sleep(2)
        
        if middleware_proc.poll() is not None:
            print("ERROR: Middleware exited prematurely")
            return 1
        
        print("✓ Middleware started (PID: {})\n".format(middleware_proc.pid))
        
        # Start ACC application
        print("Starting ACC application (test_acc.py)...")
        acc_proc = subprocess.Popen(
            ['python3', 'apps/test_acc.py'],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1
        )
        
        time.sleep(1)
        
        if acc_proc.poll() is not None:
            print("ERROR: ACC application exited prematurely")
            stdout, _ = acc_proc.communicate()
            print(f"ACC output:\n{stdout}")
            return 1
        
        print("✓ ACC application started (PID: {})\n".format(acc_proc.pid))
        
        print("=" * 70)
        print("Both processes running - monitoring for 10 seconds...")
        print("=" * 70)
        print()
        
        # Monitor ACC output for 10 seconds
        start_time = time.time()
        acc_lines = []
        
        while time.time() - start_time < 10.0:
            if acc_proc.poll() is not None:
                print("\nWARNING: ACC application exited")
                break
            
            if middleware_proc.poll() is not None:
                print("\nWARNING: Middleware exited")
                break
            
            # Try to read a line from ACC (non-blocking)
            try:
                # This is a simple approach - in production use select/poll
                line = acc_proc.stdout.readline()
                if line:
                    acc_lines.append(line.strip())
                    if len(acc_lines) <= 15:  # Print first 15 lines
                        print(f"ACC: {line.strip()}")
            except:
                pass
            
            time.sleep(0.1)
        
        print()
        print("=" * 70)
        print("Test Complete - Stopping processes...")
        print("=" * 70)
        print()
        
    finally:
        # Stop processes
        if acc_proc and acc_proc.poll() is None:
            print("Stopping ACC application...")
            acc_proc.send_signal(signal.SIGINT)
            try:
                acc_proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                acc_proc.kill()
            print("✓ ACC stopped")
        
        if middleware_proc and middleware_proc.poll() is None:
            print("Stopping middleware...")
            middleware_proc.send_signal(signal.SIGINT)
            try:
                middleware_proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                middleware_proc.kill()
            print("✓ Middleware stopped")
    
    print()
    print("=" * 70)
    print("Test Results")
    print("=" * 70)
    
    # Count ACC output lines with frame data
    frame_lines = [l for l in acc_lines if 'Frame' in l and 'ego_x' in l]
    
    if len(frame_lines) > 0:
        print(f"✓ ACC received {len(frame_lines)} data frames")
        print(f"✓ Closed-loop communication verified!")
        print()
        print("Sample output:")
        for line in frame_lines[:3]:
            print(f"  {line}")
        if len(frame_lines) > 3:
            print(f"  ... ({len(frame_lines) - 3} more frames)")
    else:
        print("✗ No data frames received by ACC")
        print("  This might indicate a communication issue")
        print()
        print("ACC output:")
        for line in acc_lines[:20]:
            print(f"  {line}")
    
    print()
    print("=" * 70)
    
    return 0 if len(frame_lines) > 0 else 1

if __name__ == '__main__':
    sys.exit(main())
