#!/bin/bash
# Simple manual test for Phase 5
# Run this script and watch the output from both processes

set -e

echo "=========================================="
echo "Phase 5 Manual Test"
echo "=========================================="
echo ""
echo "This will:"
echo "  1. Start middleware in Terminal 1 (foreground)"
echo "  2. You manually start ACC in Terminal 2"
echo ""
echo "To test:"
echo "  Terminal 1: ./test_manual.sh"
echo "  Terminal 2: cd apps && python3 test_acc.py"
echo ""
echo "Press Enter to start middleware..."
read

cd "$(dirname "$0")"

echo "Building..."
bazel build //examples:simple_demo

echo ""
echo "=========================================="
echo "Starting Middleware"
echo "=========================================="
echo ""
echo "In another terminal, run:"
echo "  cd /home/nuju/repo/addemo/adgw/apps"
echo "  python3 test_acc.py"
echo ""
echo "Press Ctrl+C to stop"
echo ""

bazel-bin/examples/simple_demo
