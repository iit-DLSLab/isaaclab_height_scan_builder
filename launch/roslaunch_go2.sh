ros2 launch isaaclab_height_scan_builder height_scan_builder.launch.py \
    world_frame:=false \
    base_link:=base_link \
    lidar_topic1_name:=/utlidar/cloud \
    self_filter:=true \
    output_topic:=/point_filtered \
    use_sim_time:=false