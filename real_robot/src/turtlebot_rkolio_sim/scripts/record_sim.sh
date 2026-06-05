#!/bin/bash
# ═══════════════════════════════════════════════════════════════════════════
# record_sim.sh — Launch simulation and record a rosbag
# Usage: ./record_sim.sh [--world corridor|basement_room|tunnel]
# ═══════════════════════════════════════════════════════════════════════════
set -e

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

WORLD="corridor"

# ── Parse arguments ──────────────────────────────────────────────────────
while [[ $# -gt 0 ]]; do
  case "$1" in
    --world)
      WORLD="$2"
      shift 2
      ;;
    --world=*)
      WORLD="${1#*=}"
      shift
      ;;
    -h|--help)
      echo "Usage: $0 [--world corridor|basement_room|tunnel]"
      exit 0
      ;;
    *)
      echo -e "${RED}Unknown argument: $1${NC}"
      exit 1
      ;;
  esac
done

# ── Validate world name ─────────────────────────────────────────────────
if [[ "$WORLD" != "corridor" && "$WORLD" != "basement_room" && "$WORLD" != "tunnel" ]]; then
  echo -e "${RED}Error: Invalid world '$WORLD'. Must be one of: corridor, basement_room, tunnel${NC}"
  exit 1
fi

# ── Check Docker ─────────────────────────────────────────────────────────
if ! command -v docker &>/dev/null; then
  echo -e "${RED}Error: docker is not installed.${NC}"
  exit 1
fi

if ! docker compose version &>/dev/null; then
  echo -e "${RED}Error: 'docker compose' is not available.${NC}"
  exit 1
fi

if [ -z "$DISPLAY" ]; then
  echo -e "${YELLOW}Warning: \$DISPLAY is not set.${NC}"
fi

xhost +local:docker 2>/dev/null || true

# ── Navigate to repo root ────────────────────────────────────────────────
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../../.." && pwd)"
cd "$REPO_ROOT"

# ── Create recordings directory ──────────────────────────────────────────
mkdir -p recordings

# ── Build image ──────────────────────────────────────────────────────────
echo -e "${GREEN}Building Docker image...${NC}"
docker compose -f docker/docker-compose.yml build

# ── Launch simulation in background ──────────────────────────────────────
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
BAG_NAME="sim_${TIMESTAMP}"

echo -e "${GREEN}Launching simulation: world=$WORLD${NC}"
docker compose -f docker/docker-compose.yml run --rm -d \
  --name rkolio_sim_recording \
  -e WORLD="$WORLD" rkolio-sim \
  ros2 launch turtlebot_rkolio_sim sim_with_rkolio.launch.py \
  world:="$WORLD"

# ── Wait for simulation to initialize ────────────────────────────────────
echo -e "${YELLOW}Waiting 3 seconds for simulation to start...${NC}"
sleep 3

# ── Start recording ─────────────────────────────────────────────────────
echo -e "${GREEN}Starting rosbag recording → /workspace/recordings/${BAG_NAME}${NC}"
docker exec -it rkolio_sim_recording \
  ros2 bag record \
    /velodyne_points \
    /imu/data \
    /rko_lio/odometry \
    /rko_lio/local_map \
    /tf \
    /tf_static \
    -o "/workspace/recordings/${BAG_NAME}"

# ── Cleanup on exit ──────────────────────────────────────────────────────
echo ""
echo -e "${YELLOW}Recording stopped.${NC}"
echo -e "${GREEN}Bag saved to: recordings/${BAG_NAME}${NC}"
echo ""
echo "Stopping simulation container..."
docker stop rkolio_sim_recording 2>/dev/null || true
echo -e "${GREEN}Done.${NC}"
