#!/bin/bash

# Integration Testing Script
# Tests Symovo API and FastAPI endpoints

set -e

echo "=========================================="
echo "Integration Testing Script"
echo "=========================================="
echo ""

# Colors
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Configuration
FASTAPI_URL="http://localhost:8000"
SYMOVO_ENDPOINT="${SYMOVO_ENDPOINT:-https://192.168.1.100}"
AMR_ID="${AMR_ID:-15}"

echo -e "${YELLOW}Configuration:${NC}"
echo "  FastAPI URL: $FASTAPI_URL"
echo "  Symovo Endpoint: $SYMOVO_ENDPOINT"
echo "  AMR ID: $AMR_ID"
echo ""

# Test FastAPI Health
echo -e "${YELLOW}Testing FastAPI Health...${NC}"
if curl -s -f "$FASTAPI_URL/api/monitor/health" > /dev/null; then
    echo -e "${GREEN}✅ FastAPI is running${NC}"
    curl -s "$FASTAPI_URL/api/monitor/health" | python3 -m json.tool
else
    echo -e "${RED}❌ FastAPI is not running${NC}"
    exit 1
fi
echo ""

# Test FastAPI Status
echo -e "${YELLOW}Testing FastAPI Status...${NC}"
curl -s "$FASTAPI_URL/api/monitor/status" | python3 -m json.tool
echo ""

# Test FastAPI Nodes
echo -e "${YELLOW}Testing FastAPI Nodes...${NC}"
curl -s "$FASTAPI_URL/api/monitor/nodes" | python3 -m json.tool | head -20
echo ""

# Test Symovo API
echo -e "${YELLOW}Testing Symovo API...${NC}"
if [ -f "/home/boris/ros2_ws/scripts/test_symovo_api.py" ]; then
    cd /home/boris/ros2_ws
    python3 scripts/test_symovo_api.py \
        --endpoint "$SYMOVO_ENDPOINT" \
        --amr-id "$AMR_ID" \
        --tls-verify 2>&1 | tail -30
else
    echo -e "${RED}❌ test_symovo_api.py not found${NC}"
fi
echo ""

# Test FastAPI Positions API
echo -e "${YELLOW}Testing FastAPI Positions API...${NC}"
echo "GET /api/positions"
curl -s "$FASTAPI_URL/api/positions" | python3 -m json.tool | head -30
echo ""

echo -e "${GREEN}=========================================="
echo "Integration tests completed"
echo "==========================================${NC}"

