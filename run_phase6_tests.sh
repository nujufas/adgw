#!/bin/bash
# Phase 6 Test Suite - Simple Integration Tests
set -e

# Change to project root
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "Working directory: $(pwd)"
echo ""

echo "======================================================================"
echo "Phase 6: Integration Testing & Validation"
echo "======================================================================"
echo ""

# Test counter
TESTS_RUN=0
TESTS_PASSED=0
TESTS_FAILED=0

# Helper functions
run_test() {
    local test_name="$1"
    local test_command="$2"
    
    TESTS_RUN=$((TESTS_RUN + 1))
    echo ""
    echo "----------------------------------------------------------------------"
    echo "Test $TESTS_RUN: $test_name"
    echo "----------------------------------------------------------------------"
    
    if eval "$test_command"; then
        echo "✓ PASSED: $test_name"
        TESTS_PASSED=$((TESTS_PASSED + 1))
        return 0
    else
        echo "✗ FAILED: $test_name"
        TESTS_FAILED=$((TESTS_FAILED + 1))
        return 1
    fi
}

# Build first
echo "Building all components..."
bazel build //... || { echo "Build failed"; exit 1; }
echo "✓ Build successful"
echo ""

# Test 1: All unit tests pass
run_test "Unit Tests" "bazel test //tests:test_data_structures //tests:test_mock_simulator //tests:test_udp_adapter //tests:test_middleware_engine --test_output=errors"

# Test 2: Middleware runs for 5 seconds
run_test "Middleware Stability" "timeout 5 bazel-bin/examples/simple_demo > /tmp/mw_test.log 2>&1 || [ \$? -eq 124 ]"

# Test 3: ACC application can start and stop cleanly
run_test "ACC Application Lifecycle" "timeout 2 python3 apps/test_acc.py > /tmp/acc_test.log 2>&1 || [ \$? -eq 124 ]"

# Test 4: Closed-loop integration (middleware + ACC together)
run_test "Closed-Loop Integration" "
    # Start middleware
    bazel-bin/examples/simple_demo > /tmp/mw_cl.log 2>&1 &
    MW_PID=\$!
    sleep 2
    
    # Start ACC for 3 seconds
    timeout 3 python3 apps/test_acc.py > /tmp/acc_cl.log 2>&1 || true
    
    # Stop middleware
    kill \$MW_PID 2>/dev/null
    wait \$MW_PID 2>/dev/null || true
    
    # Check logs for success - ACC should receive frames
    grep -q 'Frame.*ego_speed' /tmp/acc_cl.log
"

# Test 5: Message reception rate
run_test "Message Reception Rate" "
    # Start middleware
    bazel-bin/examples/simple_demo > /dev/null 2>&1 &
    MW_PID=\$!
    sleep 2
    
    # Run ACC for 5 seconds
    timeout 5 python3 apps/test_acc.py > /tmp/acc_rate.log 2>&1 || true
    
    # Stop middleware
    kill \$MW_PID 2>/dev/null || true
    wait 2>/dev/null || true
    
    # Check if we got reasonable frame rate (at least 400 frames in 5 sec = 80 Hz)
    FRAMES=\$(grep 'Total frames:' /tmp/acc_rate.log | awk '{print \$3}' || echo '0')
    echo \"Received \$FRAMES frames\"
    [ -n \"\$FRAMES\" ] && [ \"\$FRAMES\" -gt 400 ]
"

# Test 6: Protocol serialization
run_test "Protocol Serialization" "python3 apps/middleware_protocol.py > /dev/null 2>&1"

# Test 7: Build artifacts exist
run_test "Build Artifacts" "
    [ -f bazel-bin/examples/simple_demo ] && \
    [ -f apps/test_acc.py ] && \
    [ -f apps/middleware_protocol.py ]
"

# Test 8: UDP multicast reception
run_test "UDP Multicast Reception" "
    # Start middleware
    bazel-bin/examples/simple_demo > /dev/null 2>&1 &
    MW_PID=\$!
    sleep 2
    
    # Test receiver for 2 seconds
    timeout 2 python3 test_udp_debug.py > /tmp/udp_test.log 2>&1 || true
    
    # Stop middleware
    kill \$MW_PID 2>/dev/null || true
    wait \$MW_PID 2>/dev/null || true
    
    # Check if we received packets
    grep -q 'Received.*bytes' /tmp/udp_test.log
"

# Summary
echo ""
echo "======================================================================"
echo "Test Summary"
echo "======================================================================"
echo "  Tests run:     $TESTS_RUN"
echo "  Tests passed:  $TESTS_PASSED"
echo "  Tests failed:  $TESTS_FAILED"
echo "======================================================================"

if [ $TESTS_FAILED -eq 0 ]; then
    echo "✓ ALL TESTS PASSED!"
    echo ""
    exit 0
else
    echo "✗ SOME TESTS FAILED"
    echo ""
    exit 1
fi
