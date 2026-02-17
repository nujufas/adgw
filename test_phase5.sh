#!/bin/bash
# Test script for Phase 5 - Closed-loop ACC validation
#
# This script runs the middleware and ACC application together
# to validate bidirectional communication

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "======================================================================"
echo "Phase 5: Closed-Loop ACC Test"
echo "======================================================================"
echo ""
echo "This test validates:"
echo "  1. Middleware publishes vehicle data via UDP multicast"
echo "  2. ACC application receives and processes data"
echo "  3. ACC application sends control commands back"
echo "  4. Middleware receives and applies commands to simulator"
echo ""
echo "Press Ctrl+C to stop both processes"
echo ""
echo "======================================================================"
echo ""

# Build middleware first
echo "Building middleware..."
bazel build //examples:simple_demo
echo ""

# Function to cleanup on exit
cleanup() {
    echo ""
    echo "Cleaning up..."
    kill $MIDDLEWARE_PID 2>/dev/null || true
    kill $ACC_PID 2>/dev/null || true
    wait 2>/dev/null || true
    echo "Done"
}

trap cleanup EXIT INT TERM

# Start middleware in background
echo "Starting middleware (simple_demo)..."
bazel-bin/examples/simple_demo &
MIDDLEWARE_PID=$!

# Give middleware time to initialize
sleep 2

# Start ACC application
echo "Starting ACC application (test_acc.py)..."
cd apps
python3 test_acc.py &
ACC_PID=$!

echo ""
echo "Both processes running:"
echo "  Middleware PID: $MIDDLEWARE_PID"
echo "  ACC App PID:    $ACC_PID"
echo ""
echo "Watch the output to verify closed-loop control!"
echo "Press Ctrl+C to stop"
echo ""

# Wait for either process to exit
wait $MIDDLEWARE_PID $ACC_PID
