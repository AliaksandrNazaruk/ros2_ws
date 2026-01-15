#!/bin/bash
# Prevent duplicate nodes before launching
# Run this script before starting any launch file

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(dirname "$SCRIPT_DIR")"

echo "=========================================="
echo "Pre-Launch Duplicate Prevention"
echo "=========================================="
echo

# Check for duplicates
echo "Checking for duplicate nodes..."
python3 "$SCRIPT_DIR/check_duplicate_nodes.py"

# Ask user if they want to clean up
read -p "Do you want to clean up duplicates? (y/N): " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    echo "Cleaning up duplicates..."
    bash "$SCRIPT_DIR/cleanup_duplicate_nodes.sh"
    echo
    echo "✅ Cleanup complete!"
else
    echo "Skipping cleanup. Launch may fail if duplicates exist."
fi

echo
echo "=========================================="
echo "Ready to launch"
echo "=========================================="
