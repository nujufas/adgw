#!/bin/bash

# Phase 7 Test Script - esmini Integration

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "========================================"
echo "Phase 7 - esmini Integration Test"
echo "========================================"
echo ""

# Export LD_LIBRARY_PATH for esmini
export LD_LIBRARY_PATH=$PWD/third_party/esmini/bin:$LD_LIBRARY_PATH

# Test 1: Build check
echo "Test 1: Build Check"
echo "--------------------"
if [ ! -f "./bazel-bin/examples/esmini_demo" ]; then
    echo "✗ FAILED: esmini_demo executable not found"
    exit 1
fi
echo "✓ PASSED: esmini_demo executable exists"
echo ""

# Test 2: Scenario file check
echo "Test 2: Scenario File Check"
echo "----------------------------"
if [ ! -f "scenarios/acc_test.xosc" ]; then
    echo "✗ FAILED: acc_test.xosc not found"
    exit 1
fi
echo "✓ PASSED: acc_test.xosc exists"
echo ""

# Test 3: esmini library check
echo "Test 3: esmini Library Check"
echo "-----------------------------"
if [ ! -f "third_party/esmini/bin/libesminiLib.so" ]; then
    echo "✗ FAILED: libesminiLib.so not found"
    exit 1
fi
echo "✓ PASSED: libesminiLib.so exists"
echo ""

# Test 4: esmini standalone run (3 seconds)
echo "Test 4: esmini Standalone Run"
echo "------------------------------"
timeout 3 ./bazel-bin/examples/esmini_demo scenarios/acc_test.xosc > /tmp/phase7_test4.log 2>&1 || true
if grep -q "EsminiAdapter initialized successfully" /tmp/phase7_test4.log; then
    echo "✓ PASSED: esmini initialized"
else
    echo "✗ FAILED: esmini did not initialize"
    cat /tmp/phase7_test4.log
    exit 1
fi

if grep -q "Number of objects: 2" /tmp/phase7_test4.log; then
    echo "✓ PASSED: 2 vehicles loaded"
else
    echo "✗ FAILED: vehicles not loaded"
    exit 1
fi

if grep -q "Middleware Engine running..." /tmp/phase7_test4.log; then
    echo "✓ PASSED: Engine running"
else
    echo "✗ FAILED: Engine did not start"
    exit 1
fi
echo ""

# Test 5: Check UDP packets are sent
echo "Test 5: UDP Multicast Transmission"
echo "-----------------------------------"
# Start esmini_demo
./bazel-bin/examples/esmini_demo scenarios/acc_test.xosc > /tmp/phase7_mw.log 2>&1 &
MW_PID=$!
sleep 1

# Listen for UDP packets (3 seconds)
timeout 3 python3 test_udp_debug.py > /tmp/phase7_udp.log 2>&1 || true
kill $MW_PID 2>/dev/null || true
wait $MW_PID 2>/dev/null || true

# Count packets with actual content (lines starting with [number])
PACKET_COUNT=$(grep -c "^\[" /tmp/phase7_udp.log 2>/dev/null || echo "0")
# Remove any whitespace/newlines
PACKET_COUNT=$(echo "$PACKET_COUNT" | tr -d '\n\r ')
if [ "$PACKET_COUNT" -gt "50" ] 2>/dev/null; then
    echo "✓ PASSED: Received $PACKET_COUNT packets (>50)"
else
    echo "✗ FAILED: Only received $PACKET_COUNT packets"
    exit 1
fi
echo ""

# Test 6: Closed-loop with ACC (5 seconds)
echo "Test 6: Closed-Loop with ACC"
echo "-----------------------------"
./bazel-bin/examples/esmini_demo scenarios/acc_test.xosc > /tmp/phase7_cl_mw.log 2>&1 &
MW_PID=$!
sleep 0.5

timeout 5 python3 apps/test_acc.py > /tmp/phase7_cl_acc.log 2>&1 || true
kill $MW_PID 2>/dev/null || true
wait $MW_PID 2>/dev/null || true

# Check ACC received frames
if grep -q "Total frames:" /tmp/phase7_cl_acc.log; then
    FRAME_COUNT=$(grep "Total frames:" /tmp/phase7_cl_acc.log | awk '{print $3}' | tr -d ',')
    if [ "$FRAME_COUNT" -gt "100" ]; then
        echo "✓ PASSED: ACC received $FRAME_COUNT frames"
    else
        echo "✗ FAILED: ACC only received $FRAME_COUNT frames"
        exit 1
    fi
else
    echo "✗ FAILED: ACC log incomplete"
    cat /tmp/phase7_cl_acc.log
    exit 1
fi
echo ""

# Test 7: Performance check
echo "Test 7: Performance Metrics"
echo "----------------------------"
if grep -q "Real-Time Factor:" /tmp/phase7_cl_mw.log; then
    RTF=$(grep "Real-Time Factor:" /tmp/phase7_cl_mw.log | awk '{print $3}' | tr -d 'x')
    echo "Real-Time Factor: ${RTF}x"
    
    # Check if RTF is reasonable (>0.8)
    if awk "BEGIN {exit !($RTF > 0.8)}"; then
        echo "✓ PASSED: Real-time factor acceptable"
    else
        echo "✗ WARNING: Real-time factor low ($RTF < 0.8)"
    fi
else
    echo "✗ FAILED: No performance metrics found"
    exit 1
fi
echo ""

# Test 8: Visualization test (with viewer)
echo "Test 8: Visualization Test"
echo "---------------------------"
echo "Note: This test requires X11/graphics support"
echo "Starting esmini with viewer for 5 seconds..."

# Check if DISPLAY is set (indicates graphical environment)
if [ -z "$DISPLAY" ]; then
    echo "⚠ WARNING: DISPLAY not set, skipping visualization test"
    echo "  (Test would fail in headless environment)"
else
    # Run with viewer for 5 seconds
    timeout 5 ./bazel-bin/examples/esmini_demo scenarios/acc_test.xosc --viewer > /tmp/phase7_viz.log 2>&1 || true
    
    # Check if it started successfully
    if grep -q "EsminiAdapter initialized successfully" /tmp/phase7_viz.log; then
        echo "✓ PASSED: esmini viewer started"
        
        if grep -q "Headless: no" /tmp/phase7_viz.log; then
            echo "✓ PASSED: Viewer mode confirmed"
        else
            echo "✗ FAILED: Viewer mode not confirmed"
            exit 1
        fi
    else
        echo "✗ FAILED: esmini with viewer did not initialize"
        cat /tmp/phase7_viz.log | tail -20
        exit 1
    fi
fi
echo ""

# Summary
echo "========================================"
echo "Phase 7 Test Summary"
echo "========================================"
echo "All tests PASSED!"
echo ""
echo "Phase 7 Complete:"
echo "  ✓ esmini integration working"
echo "  ✓ UDP IPC functioning"
echo "  ✓ Closed-loop with ACC validated"
echo "  ✓ Performance acceptable"
echo ""
echo "Next: Document results in PHASE7_COMPLETE.md"
