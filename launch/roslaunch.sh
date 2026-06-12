ros2 launch isaaclab_height_scan_builder height_scan_builder.launch.py \
    imu_topic:=/xbotcore/imu/imu_link \
    base_link:=pelvis \
    imu_frame:=imu_link \
    lidar_topic1_name:=/hesai_jt128_front/points \
    lidar_topic2_name:=/hesai_jt128_back/points \
    self_filter:=true \
    filter_config:=kyon_self_filter.yaml \
    output_topic:=/point_filtered \
    use_sim_time:=true