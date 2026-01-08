#!/bin/bash

# Script to check and fix Cloudflare Tunnel configuration for ros2.techvisioncloud.pl

echo "=== Cloudflare Tunnel Configuration Checker ==="
echo ""

CONFIG_FILE="/etc/cloudflared/config.yml"
BACKUP_FILE="/etc/cloudflared/config.yml.backup.$(date +%Y%m%d_%H%M%S)"

# Check if running as root
if [ "$EUID" -ne 0 ]; then 
    echo "❌ This script must be run as root (use sudo)"
    exit 1
fi

# Check if config file exists
if [ ! -f "$CONFIG_FILE" ]; then
    echo "❌ Config file not found: $CONFIG_FILE"
    exit 1
fi

echo "📋 Current configuration:"
echo ""
sudo cat "$CONFIG_FILE" | grep -A 5 "ros2.techvisioncloud.pl" || echo "⚠️  ros2.techvisioncloud.pl rule not found!"
echo ""

# Check if ros2 rule is first
FIRST_RULE=$(sudo grep -n "hostname:" "$CONFIG_FILE" | head -1 | cut -d: -f2 | tr -d ' ')
if [[ "$FIRST_RULE" == *"ros2.techvisioncloud.pl"* ]]; then
    echo "✅ ros2.techvisioncloud.pl is FIRST in ingress rules"
else
    echo "❌ ros2.techvisioncloud.pl is NOT first in ingress rules"
    echo "   First rule: $FIRST_RULE"
fi

# Check if path field exists
if sudo grep -A 3 "ros2.techvisioncloud.pl" "$CONFIG_FILE" | grep -q "path:"; then
    echo "❌ Found 'path:' field in ros2 rule - this will cause 404 for /api/*"
    echo "   The 'path:' field limits routing to specific paths only"
else
    echo "✅ No 'path:' field found (correct)"
fi

echo ""
echo "=== Testing endpoints ==="
echo ""

# Test local
echo "Local API:"
LOCAL_STATUS=$(curl -s -o /dev/null -w "%{http_code}" http://localhost:8000/api/monitor/health 2>/dev/null)
if [ "$LOCAL_STATUS" = "200" ]; then
    echo "  ✅ http://localhost:8000/api/monitor/health → HTTP $LOCAL_STATUS"
else
    echo "  ❌ http://localhost:8000/api/monitor/health → HTTP $LOCAL_STATUS"
    echo "     Server might not be running!"
fi

# Test remote
echo "Remote API:"
REMOTE_STATUS=$(curl -s -o /dev/null -w "%{http_code}" https://ros2.techvisioncloud.pl/api/monitor/health 2>/dev/null)
if [ "$REMOTE_STATUS" = "200" ]; then
    echo "  ✅ https://ros2.techvisioncloud.pl/api/monitor/health → HTTP $REMOTE_STATUS"
else
    echo "  ❌ https://ros2.techvisioncloud.pl/api/monitor/health → HTTP $REMOTE_STATUS"
    echo "     Cloudflare Tunnel is not routing /api/* paths correctly"
fi

echo ""
echo "=== Recommendations ==="
echo ""

if [ "$REMOTE_STATUS" != "200" ]; then
    echo "To fix the 404 issue:"
    echo ""
    echo "1. Make backup:"
    echo "   sudo cp $CONFIG_FILE $BACKUP_FILE"
    echo ""
    echo "2. Edit config:"
    echo "   sudo nano $CONFIG_FILE"
    echo ""
    echo "3. Ensure ros2 rule is FIRST and has NO 'path:' field:"
    echo "   ingress:"
    echo "     - hostname: ros2.techvisioncloud.pl"
    echo "       service: http://localhost:8000"
    echo ""
    echo "4. Restart cloudflared:"
    echo "   sudo systemctl restart cloudflared"
    echo ""
    echo "5. Wait 10 seconds and test:"
    echo "   curl -I https://ros2.techvisioncloud.pl/api/monitor/health"
    echo ""
    echo "Or use the fix script:"
    echo "   sudo ./scripts/nav2_test_server/FIX_CLOUDFLARE_404.sh"
fi

