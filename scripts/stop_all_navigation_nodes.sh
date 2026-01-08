#!/bin/bash

# Stop all navigation_integrated_node processes
# This script helps clean up duplicate processes

echo "Stopping all navigation_integrated_node processes..."

# Find all processes
PIDS=$(ps aux | grep "navigation_integrated_node" | grep -v grep | awk '{print $2}')

if [ -z "$PIDS" ]; then
    echo "No navigation_integrated_node processes found"
    exit 0
fi

echo "Found processes: $PIDS"

# Stop each process
for PID in $PIDS; do
    echo "Stopping process $PID..."
    kill -TERM $PID 2>/dev/null
    sleep 1
    
    # Force kill if still running
    if ps -p $PID > /dev/null 2>&1; then
        echo "Force killing process $PID..."
        kill -KILL $PID 2>/dev/null
    fi
done

echo "Done. Waiting 2 seconds..."
sleep 2

# Verify
REMAINING=$(ps aux | grep "navigation_integrated_node" | grep -v grep | wc -l)
if [ "$REMAINING" -eq 0 ]; then
    echo "All navigation_integrated_node processes stopped"
else
    echo "Warning: $REMAINING processes still running"
    ps aux | grep "navigation_integrated_node" | grep -v grep
fi

