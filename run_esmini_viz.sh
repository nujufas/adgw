#!/bin/bash

# Run esmini with visualization
# This script launches the middleware with esmini's 3D viewer

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# Check if scenario file provided
SCENARIO="${1:-scenarios/acc_test.xosc}"

if [ ! -f "$SCENARIO" ]; then
    echo "Error: Scenario file not found: $SCENARIO"
    echo ""
    echo "Usage: $0 [scenario_file.xosc]"
    echo "Example: $0 scenarios/acc_test.xosc"
    exit 1
fi

# Export LD_LIBRARY_PATH for esmini
export LD_LIBRARY_PATH=$PWD/third_party/esmini/bin:$LD_LIBRARY_PATH

# Check if DISPLAY is set
if [ -z "$DISPLAY" ]; then
    echo "Warning: DISPLAY environment variable not set"
    echo "X11 forwarding or local display required for visualization"
    echo ""
    echo "If using SSH, try: ssh -X user@host"
    echo "Or set DISPLAY manually: export DISPLAY=:0"
    exit 1
fi

echo "========================================"
echo "esmini Visualization Demo"
echo "========================================"
echo ""
echo "Two options to run with visualization:"
echo ""
echo "Option 1: Using esmini native binary (recommended for visualization)"
echo "  ./test_esmini_native.sh"
echo ""
echo "Option 2: Using middleware with esmini adapter"
echo "  Starting esmini with middleware integration..."
echo "  Note: SE_Init() may override viewer settings"
echo ""
echo "Scenario: $SCENARIO"
echo "DISPLAY: $DISPLAY"
echo ""
echo "Controls:"
echo "  - Camera follows vehicles automatically"
echo "  - Close window or press Ctrl+C to stop"
echo ""
echo "----------------------------------------"
echo ""

# Run esmini_demo with viewer (no timeout - let it run until user stops)
./bazel-bin/examples/esmini_demo "$SCENARIO" --viewer

echo ""
echo "esmini visualization stopped"
