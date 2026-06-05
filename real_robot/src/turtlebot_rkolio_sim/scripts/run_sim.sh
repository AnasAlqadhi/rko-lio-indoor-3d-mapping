#!/bin/bash
# ═══════════════════════════════════════════════════════════════════════════
# run_sim.sh — Single entry point for the RKO-LIO Gazebo simulation
# Usage: ./run_sim.sh [--world corridor|basement_room|tunnel]
# ═══════════════════════════════════════════════════════════════════════════
set -e

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

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
      echo ""
      echo "Options:"
      echo "  --world   Gazebo world to load (default: corridor)"
      echo "            Choices: corridor, basement_room, tunnel"
      exit 0
      ;;
    *)
      echo -e "${RED}Unknown argument: $1${NC}"
      echo "Usage: $0 [--world corridor|basement_room|tunnel]"
      exit 1
      ;;
  esac
done

# ── Validate world name ─────────────────────────────────────────────────
if [[ "$WORLD" != "corridor" && "$WORLD" != "basement_room" && "$WORLD" != "tunnel" ]]; then
  echo -e "${RED}Error: Invalid world '$WORLD'. Must be one of: corridor, basement_room, tunnel${NC}"
  exit 1
fi

# ── Check Docker is installed ────────────────────────────────────────────
if ! command -v docker &>/dev/null; then
  echo -e "${RED}Error: docker is not installed. Please install Docker first.${NC}"
  exit 1
fi

# ── Check docker compose is available ────────────────────────────────────
if ! docker compose version &>/dev/null; then
  echo -e "${RED}Error: 'docker compose' is not available. Please install Docker Compose V2.${NC}"
  exit 1
fi

# ── Check DISPLAY ────────────────────────────────────────────────────────
if [ -z "$DISPLAY" ]; then
  echo -e "${YELLOW}Warning: \$DISPLAY is not set. GUI applications (Gazebo, RViz) may not work.${NC}"
fi

# ── Allow Docker to access X11 display ───────────────────────────────────
xhost +local:docker 2>/dev/null || true

# ── Navigate to repo root (two levels up from scripts/) ──────────────────
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../../.." && pwd)"
cd "$REPO_ROOT"

# ── Build Docker image ───────────────────────────────────────────────────
echo -e "${GREEN}Building Docker image...${NC}"
docker compose -f docker/docker-compose.yml build

# ── Launch simulation ────────────────────────────────────────────────────
echo -e "${GREEN}Launching simulation: world=$WORLD${NC}"
docker compose -f docker/docker-compose.yml run --rm \
  -e WORLD="$WORLD" rkolio-sim \
  ros2 launch turtlebot_rkolio_sim sim_with_rkolio.launch.py \
  world:="$WORLD"
