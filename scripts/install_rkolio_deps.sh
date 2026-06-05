#!/usr/bin/env bash
# =============================================================================
# install_rkolio_deps.sh
# RKO-LIO dependency installer + workspace builder
# Hardware: Velodyne VLP-16  +  Pixhawk Orange Cube Plus  +  Ubuntu 22.04
# ROS 2 distro: Humble (default) — override with:  ROS_DISTRO=iron ./install_rkolio_deps.sh
# =============================================================================
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-humble}"
WS_DIR="${RKOLIO_WS:-$HOME/tb3_3d_ws}"          # your colcon workspace root
JOBS="${JOBS:-$(nproc)}"

# Terminal colours
GREEN='\033[0;32m'; YELLOW='\033[1;33m'; RED='\033[0;31m'; NC='\033[0m'
info()  { echo -e "${GREEN}[INFO]${NC}  $*"; }
warn()  { echo -e "${YELLOW}[WARN]${NC}  $*"; }
die()   { echo -e "${RED}[ERR]${NC}   $*" >&2; exit 1; }

# ─── Sanity checks ───────────────────────────────────────────────────────────
[[ $EUID -ne 0 ]] || die "Do NOT run this script as root."
command -v ros2 &>/dev/null || die "ROS 2 ${ROS_DISTRO} not found. Source /opt/ros/${ROS_DISTRO}/setup.bash first."
info "Using ROS 2 distro : ${ROS_DISTRO}"
info "Workspace          : ${WS_DIR}"

# ─── 0. System update ────────────────────────────────────────────────────────
info "0/7  Refreshing APT package lists …"
sudo apt-get update -qq

# ─── 1. Core build tools ─────────────────────────────────────────────────────
info "1/7  Installing core build tools …"
sudo apt-get install -y --no-install-recommends \
    build-essential cmake git wget curl \
    python3-pip python3-colcon-common-extensions python3-rosdep \
    python3-vcstool

# ─── 2. ROS 2 base packages ──────────────────────────────────────────────────
info "2/7  Installing ROS 2 packages …"
sudo apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-tf2 \
    ros-${ROS_DISTRO}-tf2-ros \
    ros-${ROS_DISTRO}-tf2-eigen \
    ros-${ROS_DISTRO}-tf2-geometry-msgs \
    ros-${ROS_DISTRO}-sensor-msgs \
    ros-${ROS_DISTRO}-nav-msgs \
    ros-${ROS_DISTRO}-geometry-msgs \
    ros-${ROS_DISTRO}-rosbag2 \
    ros-${ROS_DISTRO}-rosbag2-cpp \
    ros-${ROS_DISTRO}-rosbag2-storage \
    ros-${ROS_DISTRO}-rclcpp-components \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-diagnostics

# ─── 3. Velodyne VLP-16 ROS 2 driver ─────────────────────────────────────────
info "3/7  Installing Velodyne ROS 2 driver packages …"
sudo apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-velodyne \
    ros-${ROS_DISTRO}-velodyne-driver \
    ros-${ROS_DISTRO}-velodyne-laserscan \
    ros-${ROS_DISTRO}-velodyne-msgs \
    ros-${ROS_DISTRO}-velodyne-pointcloud

# ─── 4. MAVROS + MAVROS extras (Pixhawk comms) ───────────────────────────────
info "4/7  Installing MAVROS …"
sudo apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-mavros \
    ros-${ROS_DISTRO}-mavros-msgs \
    ros-${ROS_DISTRO}-mavros-extras

# Install GeographicLib datasets required by MAVROS
if ! python3 -c "import geographic_msgs" &>/dev/null; then
    sudo apt-get install -y ros-${ROS_DISTRO}-geographic-msgs
fi
GEOLIB_SCRIPT="/opt/ros/${ROS_DISTRO}/lib/mavros/install_geographiclib_datasets.sh"
if [[ -f "${GEOLIB_SCRIPT}" ]]; then
    info "  Installing GeographicLib datasets (requires internet) …"
    sudo bash "${GEOLIB_SCRIPT}"
else
    warn "  GeographicLib install script not found at ${GEOLIB_SCRIPT}."
    warn "  Run manually:  sudo /opt/ros/${ROS_DISTRO}/lib/mavros/install_geographiclib_datasets.sh"
fi

# ─── 5. C++ maths / optimisation libraries (RKO-LIO deps) ────────────────────
info "5/7  Installing Eigen3, Sophus, PCL, TBB …"
sudo apt-get install -y --no-install-recommends \
    libeigen3-dev \
    libsophus-dev \
    libpcl-dev \
    libtbb-dev

# Sophus is sometimes missing from apt in older Ubuntu; build from source if needed.
if ! pkg-config --exists sophus 2>/dev/null && ! dpkg -l | grep -q libsophus-dev; then
    warn "  libsophus-dev unavailable via apt — building from source …"
    SOPHUS_BUILD="$(mktemp -d)"
    git clone --depth 1 https://github.com/strasdat/Sophus.git "${SOPHUS_BUILD}"
    cmake -S "${SOPHUS_BUILD}" -B "${SOPHUS_BUILD}/build" \
          -DCMAKE_BUILD_TYPE=Release -DSOPHUS_USE_BASIC_LOGGING=ON
    cmake --build "${SOPHUS_BUILD}/build" -- -j"${JOBS}"
    sudo cmake --install "${SOPHUS_BUILD}/build"
    rm -rf "${SOPHUS_BUILD}"
fi

# ─── 6. rosdep init / update ─────────────────────────────────────────────────
info "6/7  Running rosdep …"
if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
    sudo rosdep init
fi
rosdep update --rosdistro "${ROS_DISTRO}"
rosdep install --from-paths "${WS_DIR}/src" \
               --ignore-src -r -y \
               --rosdistro "${ROS_DISTRO}" || warn "rosdep reported some unresolved deps (may be normal)."

# ─── 7. Build RKO-LIO (+ turtlebot_rkolio_hardware) ─────────────────────────
info "7/7  Building workspace with colcon …"
# shellcheck disable=SC1091
source /opt/ros/${ROS_DISTRO}/setup.bash

cd "${WS_DIR}"
colcon build \
    --symlink-install \
    --parallel-workers "${JOBS}" \
    --packages-select rko_lio turtlebot_rkolio_hardware \
    --cmake-args \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
    --event-handlers console_cohesive+

info "Build complete."
echo ""
echo -e "${GREEN}=== Next steps ===${NC}"
echo "  1. Source the workspace:"
echo "       source ${WS_DIR}/install/setup.bash"
echo ""
echo "  2. Configure the Pixhawk for 200 Hz IMU output"
echo "     (see Pixhawk IMU tuning section in REAL_ROBOT_GUIDE.md)."
echo ""
echo "  3. Launch the full stack:"
echo "       ros2 launch turtlebot_rkolio_hardware vlp16_pixhawk_rkolio.launch.py"
