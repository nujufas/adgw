#!/bin/bash

# Test esmini viewer directly using the esmini binary

set -e

cd /home/nuju/repo/addemo/adgw

export LD_LIBRARY_PATH=$PWD/third_party/esmini/bin:$LD_LIBRARY_PATH

echo "Testing esmini viewer with native binary..."
echo "This should open a 3D visualization window"
echo ""

# Run esmini native binary with the scenario
./third_party/esmini/bin/esmini --osc scenarios/acc_test.xosc --window 60 60 1024 768

echo ""
echo "esmini viewer test complete"
