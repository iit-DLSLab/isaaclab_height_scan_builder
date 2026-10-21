#!/usr/bin/env bash
set -e

# Set USE_SIM_TIME=true when replaying a bag with `ros2 bag play <bag> --clock`.
# All nodes must follow the same clock: otherwise the self filter cannot find the
# leg TFs at the cloud timestamps and the legs end up in the height map.
USE_SIM_TIME="${USE_SIM_TIME:-false}"

URDF="$(ros2 pkg prefix go2_description)/share/go2_description/urdf/go2_description.urdf"

cleanup() {
    kill \
        "${ROBOT_STATE_PID:-}" \
        "${JOINT_STATE_PID:-}" \
        "${LIDAR_TF_PID:-}" \
        2>/dev/null || true
}
trap cleanup EXIT INT TERM
if [[ ! -f "$URDF" ]]; then
    echo "ERRORE: URDF non trovato: $URDF"
    exit 1
fi

echo "URDF utilizzato: $URDF"

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

# LowState -> JointState
python3 "$SCRIPT_DIR/lowstate_to_joint_states.py" \
    --ros-args \
    -p use_sim_time:="$USE_SIM_TIME" &
JOINT_STATE_PID=$!

sleep 1

if ! kill -0 "$JOINT_STATE_PID" 2>/dev/null; then
    echo "ERRORE: joint_state_publisher si è chiuso."
    exit 1
fi

# Robot TF from URDF
ros2 run robot_state_publisher robot_state_publisher "$URDF" \
    --ros-args \
    -p use_sim_time:="$USE_SIM_TIME" &
ROBOT_STATE_PID=$!

sleep 2

# Unitree raw lidar frame -> URDF sensor frame
#
# The URDF already provides:
#   base_link -> radar
#
# This adds:
#   radar -> utlidar_lidar
#
# Therefore TF can resolve:
#   base_link <-> utlidar_lidar
# ros2 run tf2_ros static_transform_publisher \
#     --x 0 --y 0 --z 0 \
#     --roll 0 --pitch 0 --yaw 0 \
#     --frame-id radar \
#     --child-frame-id utlidar_lidar &
# LIDAR_TF_PID=$!
ros2 run tf2_ros static_transform_publisher \
    --x 0.28216005 \
    --y 0.0 \
    --z 0.0 \
    --roll -2.92072011 \
    --pitch -0.14132399 \
    --yaw -1.01052992 \
    --frame-id base_link \
    --child-frame-id utlidar_lidar &
LIDAR_TF_PID=$!

sleep 2

# Verify JointState
if ! timeout 5 ros2 topic echo /joint_states --once >/dev/null; then
    echo "ERRORE: nessun messaggio ricevuto su /joint_states."
    exit 1
fi

echo "Joint states ricevuti correttamente."

# Height-map pipeline
#
# range_offset: the Go2 L1 lidar over-reads every range by ~0.10 m. Without it
# the ground appears ~8 cm below the feet and the leg points fall outside the
# self-filter shapes (estimated from my_bag: ground flatness + leg kinematics).
#
# height_percentile: cell height = 75th percentile of the accumulated points
# instead of the max, so isolated outliers (leg edges, mixed pixels) and lidar
# noise do not lift the cell. On flat ground (my_bag9) the error RMS drops from
# 4.3 cm (max) / 3.5 cm (0.9) to 2.9 cm. Obstacles that fill a cell are not
# affected; only very thin ones shrink (2 cm wide, 8 cm tall plank -> ~6 cm).
# A longer accumulation does not help: clouds are not motion compensated.
ros2 launch isaaclab_height_scan_builder height_scan_builder.launch.py \
    world_frame:=false \
    publish_world_frame:=true \
    base_link:=base_link \
    imu_frame:=imu \
    imu_topic:=/imu \
    lidar_topic1_name:=/utlidar/cloud \
    self_filter:=true \
    output_topic:=/point_filtered \
    range_offset:=0.10 \
    height_percentile:=0.75 \
    use_sim_time:="$USE_SIM_TIME"
