#!/bin/bash
# Cleanup duplicate ROS2 nodes

set -e

echo "=========================================="
echo "ROS2 Duplicate Nodes Cleanup"
echo "=========================================="
echo

# Function to kill processes by name pattern
kill_duplicates() {
    local pattern=$1
    local node_name=$2
    local count=$(ps aux | grep "$pattern" | grep -v grep | wc -l)
    
    if [ "$count" -gt 1 ]; then
        echo "⚠️  Found $count instances of $node_name"
        echo "   Killing all but the first one..."
        
        # Get PIDs, skip the first one
        pids=$(ps aux | grep "$pattern" | grep -v grep | awk '{print $2}' | tail -n +2)
        
        for pid in $pids; do
            echo "   Killing PID: $pid"
            kill -TERM $pid 2>/dev/null || true
            sleep 0.5
            # Force kill if still running
            if kill -0 $pid 2>/dev/null; then
                echo "   Force killing PID: $pid"
                kill -KILL $pid 2>/dev/null || true
            fi
        done
        
        sleep 1
        remaining=$(ps aux | grep "$pattern" | grep -v grep | wc -l)
        echo "   ✅ Remaining instances: $remaining"
    else
        echo "✅ $node_name: OK ($count instance)"
    fi
    echo
}

# Kill duplicate base_controller nodes
echo "Checking base_controller..."
kill_duplicates "base_controller_node" "base_controller"

# Kill duplicate lifecycle_manager nodes
echo "Checking lifecycle_manager_navigation..."
kill_duplicates "lifecycle_manager.*lifecycle_manager_navigation" "lifecycle_manager_navigation"

# Wait a bit for processes to terminate
sleep 2

# Final check
echo "=========================================="
echo "Final Status:"
echo "=========================================="
base_count=$(ps aux | grep "base_controller_node" | grep -v grep | wc -l)
lifecycle_count=$(ps aux | grep "lifecycle_manager.*lifecycle_manager_navigation" | grep -v grep | wc -l)

echo "base_controller instances: $base_count"
echo "lifecycle_manager_navigation instances: $lifecycle_count"
echo

if [ "$base_count" -le 1 ] && [ "$lifecycle_count" -le 1 ]; then
    echo "✅ All duplicates cleaned up!"
else
    echo "⚠️  Some duplicates may still exist"
fi
