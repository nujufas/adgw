#!/bin/bash
# Code generation script for ACC SomeIP service
# Generates CommonAPI code from FIDL and deployment files

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

# Paths
FIDL_DIR="$SCRIPT_DIR/fidl"
FDEPL_DIR="$SCRIPT_DIR/fdepl"
OUTPUT_DIR="$SCRIPT_DIR/src-gen"

# Generators
CORE_GEN="$PROJECT_ROOT/third_party/commonapi/bin/commonapi-core-generator-linux-x86_64"
SOMEIP_GEN="$PROJECT_ROOT/third_party/commonapi/bin/commonapi-someip-generator-linux-x86_64"

echo "=== CommonAPI Code Generation for ACC Services ==="
echo ""
echo "Project Root: $PROJECT_ROOT"
echo "Output Directory: $OUTPUT_DIR"
echo ""

# Create output directory
mkdir -p "$OUTPUT_DIR"

# Step 1: Generate core CommonAPI code (proxy/stub base classes)
echo "Step 1: Generating CommonAPI Core code..."
"$CORE_GEN" -sk -d "$OUTPUT_DIR" "$FIDL_DIR/accServices.fidl"

if [ $? -eq 0 ]; then
    echo "✓ Core code generation completed"
else
    echo "✗ Core code generation failed"
    exit 1
fi

echo ""

# Step 2: Generate SomeIP-specific code
echo "Step 2: Generating CommonAPI SomeIP binding..."
"$SOMEIP_GEN" -d "$OUTPUT_DIR" "$FDEPL_DIR/accServices-someip.fdepl"

if [ $? -eq 0 ]; then
    echo "✓ SomeIP code generation completed"
else
    echo "✗ SomeIP code generation failed"
    exit 1
fi

echo ""
echo "=== Code Generation Complete ==="
echo ""
echo "Generated files are in: $OUTPUT_DIR"
echo ""
echo "Generated structure:"
tree -L 3 "$OUTPUT_DIR" 2>/dev/null || find "$OUTPUT_DIR" -type f | head -20

echo ""
echo "Next steps:"
echo "  1. Implement service stub in src/accServer/"
echo "  2. Build with: bazel build //examples/acc_someip_service:accServer"
echo "  3. Run with appropriate VSOMEIP_CONFIGURATION and COMMONAPI_CONFIG"
