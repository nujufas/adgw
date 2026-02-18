#!/bin/bash
# Script to download and extract esmini third-party dependency

set -e  # Exit on error

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
THIRD_PARTY_DIR="${PROJECT_ROOT}/third_party"
ESMINI_DIR="${THIRD_PARTY_DIR}/esmini"

ESMINI_VERSION="v2.59.0"
ESMINI_URL="https://github.com/esmini/esmini/releases/download/${ESMINI_VERSION}/esmini-demo_Linux.zip"
DOWNLOAD_FILE="${THIRD_PARTY_DIR}/esmini-demo_Linux.zip"

echo "================================================"
echo "Setting up esmini ${ESMINI_VERSION}"
echo "================================================"

# Create third_party directory if it doesn't exist
mkdir -p "${THIRD_PARTY_DIR}"

# Check if esmini is already installed
if [ -d "${ESMINI_DIR}/bin" ] && [ -f "${ESMINI_DIR}/bin/esmini" ]; then
    echo "esmini is already installed at ${ESMINI_DIR}"
    echo "To reinstall, remove the directory and run this script again:"
    echo "  rm -rf ${ESMINI_DIR}"
    echo "  ./scripts/setup_esmini.sh"
    exit 0
fi

# Download esmini
echo "Downloading esmini from ${ESMINI_URL}..."
if command -v curl &> /dev/null; then
    curl -L -o "${DOWNLOAD_FILE}" "${ESMINI_URL}"
elif command -v wget &> /dev/null; then
    wget -O "${DOWNLOAD_FILE}" "${ESMINI_URL}"
else
    echo "Error: Neither curl nor wget is available. Please install one of them."
    exit 1
fi

echo "Download complete!"

# Extract esmini
echo "Extracting esmini to ${THIRD_PARTY_DIR}..."
cd "${THIRD_PARTY_DIR}"
unzip -q "${DOWNLOAD_FILE}"

# The extracted directory is named "esmini-demo"
# Rename it to just "esmini"
if [ -d "esmini-demo" ]; then
    # Remove old esmini directory if it exists
    rm -rf esmini
    mv esmini-demo esmini
    echo "Renamed esmini-demo to esmini"
fi

# Clean up
echo "Cleaning up..."
rm -f "${DOWNLOAD_FILE}"

# Verify installation
if [ -f "${ESMINI_DIR}/bin/esmini" ]; then
    echo ""
    echo "================================================"
    echo "✓ esmini successfully installed!"
    echo "================================================"
    echo "Location: ${ESMINI_DIR}"
    echo ""
    echo "You can now build the project:"
    echo "  bazel build //..."
    echo ""
else
    echo ""
    echo "================================================"
    echo "✗ Installation verification failed"
    echo "================================================"
    echo "Expected esmini binary not found at ${ESMINI_DIR}/bin/esmini"
    exit 1
fi
