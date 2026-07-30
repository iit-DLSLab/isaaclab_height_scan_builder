#!/usr/bin/env bash
set -e
 
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
 
# Publish joint positions.
# ros2 run joint_state_publisher joint_state_publisher "$URDF" \
#     --ros-args \
#     -p publish_default_positions:=true &
# JOINT_STATE_PID=$!
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
 
python3 "$SCRIPT_DIR/lowstate_to_joint_states.py" &
JOINT_STATE_PID=$!
 
sleep 1
 
if ! kill -0 "$JOINT_STATE_PID" 2>/dev/null; then
    echo "ERRORE: joint_state_publisher si è chiuso."
    exit 1
fi
 
ros2 run robot_state_publisher robot_state_publisher "$URDF" &
ROBOT_STATE_PID=$!
 
# Alias between the sensor name in the URDF and the point cloud topic.
ros2 run tf2_ros static_transform_publisher \
    --x 0 --y 0 --z 0 \
    --roll 0 --pitch 0 --yaw 0 \
    --frame-id radar \
    --child-frame-id utlidar_lidar &
LIDAR_TF_PID=$!
 
sleep 2
 
# Verify that joint_state_publisher is really publishing.
if ! timeout 5 ros2 topic echo /joint_states --once >/dev/null; then
    echo "ERRORE: nessun messaggio ricevuto su /joint_states."
    exit 1
fi
 
echo "Joint states ricevuti correttamente."
 
ros2 launch isaaclab_height_scan_builder height_scan_builder.launch.py \
    world_frame:=false \
    base_link:=base_link \
    lidar_topic1_name:=/utlidar/cloud \
    self_filter:=true \
    output_topic:=/point_filtered \
    use_sim_time:=false