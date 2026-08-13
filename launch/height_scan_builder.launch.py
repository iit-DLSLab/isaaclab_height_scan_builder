import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression, Command
from launch.conditions import IfCondition

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    imu_topic_arg = DeclareLaunchArgument(
        'imu_topic',
        default_value='/xbotcore/imu/imu_link'
    )

    base_link_arg = DeclareLaunchArgument(
        'base_link',
        default_value='base_link'
    )

    imu_frame_arg = DeclareLaunchArgument(
        'imu_frame',
        default_value='imu_link'
    )

    lidar_topic1_name_arg = DeclareLaunchArgument(
        'lidar_topic1_name',
        default_value='/input1'
    )

    output_topic_arg = DeclareLaunchArgument(
        'output_topic',
        default_value='/points_filtered'
    )

    scan_cloud_topic_arg = DeclareLaunchArgument(
        'scan_cloud_topic',
        default_value='height_scan_cloud'
    )

    marker_topic_arg = DeclareLaunchArgument(
        'marker_topic',
        default_value='height_scan_markers'
    )

    publish_markers_arg = DeclareLaunchArgument(
        'publish_markers',
        default_value='true'
    )

    accumulation_time_sec_arg = DeclareLaunchArgument(
        'accumulation_time_sec',
        default_value='0.30',
        description=(
            'Point-cloud accumulation window in seconds; '
            'set to 0.0 to use only the latest cloud'
        )
    )

    outlier_radius_arg = DeclareLaunchArgument(
        'outlier_radius',
        default_value='0.08',
        description=(
            'Radius used to remove isolated points from the accumulated cloud; '
            'set to 0.0 to disable the filter'
        )
    )

    outlier_min_neighbors_arg = DeclareLaunchArgument(
        'outlier_min_neighbors',
        default_value='3',
        description='Minimum number of neighbors required inside outlier_radius'
    )

    enable_map_shift_arg = DeclareLaunchArgument(
        'enable_map_shift',
        default_value='false',
        description='Apply map_shift_x/y/z to every generated heightmap point'
    )

    map_shift_x_arg = DeclareLaunchArgument(
        'map_shift_x',
        default_value='0.0'
    )

    map_shift_y_arg = DeclareLaunchArgument(
        'map_shift_y',
        default_value='0.0'
    )

    map_shift_z_arg = DeclareLaunchArgument(
        'map_shift_z',
        default_value='0.0'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
    )

    # Controls the frame used internally for point-cloud processing.
    # false -> process in base_link
    # true  -> process in world
    world_frame_arg = DeclareLaunchArgument(
        'world_frame',
        default_value='false',
    )

    # Independent from processing frame.
    # Used to publish world -> base_link, mainly for RViz visualization.
    publish_world_frame_arg = DeclareLaunchArgument(
        'publish_world_frame',
        default_value='true',
    )

    self_filter_arg = DeclareLaunchArgument(
        'self_filter',
        default_value='true'
    )

    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value='',
        description='Optional URDF or MuJoCo XML path used by the self-filter'
    )

    model_format_arg = DeclareLaunchArgument(
        'model_format',
        default_value='auto',
        description='Robot model format: auto, urdf, or mujoco'
    )

    frame_prefix_arg = DeclareLaunchArgument(
        'frame_prefix',
        default_value='',
        description='Prefix used by the MuJoCo TF publisher for body frames'
    )


    # ------------------------------------------------------------------
    # Publish world -> base_link from IMU
    #
    # IMPORTANT:
    # This is now controlled by publish_world_frame, NOT world_frame.
    # Therefore the point clouds can stay in base_link while RViz can
    # still use "world" as Fixed Frame.
    # ------------------------------------------------------------------

    world_frame_publisher_node = Node(
        package='isaaclab_height_scan_builder',
        executable='imu_world_frame_publisher_node',
        name='imu_world_frame_publisher',
        output='screen',
        condition=IfCondition(
            LaunchConfiguration('publish_world_frame')
        ),
        parameters=[
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'imu_topic': LaunchConfiguration('imu_topic'),
                'cloud_topic': LaunchConfiguration('lidar_topic1_name'),
                'world_frame': "world",
                'base_link': LaunchConfiguration('base_link'),
                'imu_frame': LaunchConfiguration('imu_frame'),
                'sync_slop': 0.02,
                'queue_size': 10,
            }
        ]
    )


    # ------------------------------------------------------------------
    # Point cloud manager
    # ------------------------------------------------------------------

    point_cloud_manager_node = Node(
        package='isaaclab_height_scan_builder',
        executable='point_cloud_manager_node',
        name='point_cloud_manager',
        output='screen',
        parameters=[
            {
                'topic1': LaunchConfiguration('lidar_topic1_name'),

                'sync_slop': 0.01,

                'output_topic': LaunchConfiguration('output_topic'),

                'filter_frame': LaunchConfiguration('base_link'),

                'target_frame': ParameterValue(
                    PythonExpression([
                        '"world" if "',
                        LaunchConfiguration('world_frame'),
                        '" == "true" else "base_link"'
                    ]),
                    value_type=str,
                ),

                'voxel_leaf_size': 0.05,
                'box_x': 2.0,
                'box_y': 1.5,

                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }
        ],
        arguments=[
            '--ros-args',
            '--log-level', 'tf2_buffer:=error',
        ]
    )


    # ------------------------------------------------------------------
    # Robot self-filter
    # ------------------------------------------------------------------

    self_filter_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('robot_self_filter'),
                'launch',
                'self_filter.launch.py'
            )
        ),
        condition=IfCondition(
            LaunchConfiguration('self_filter')
        ),
        launch_arguments={
            'filter_config': os.path.join(
                get_package_share_directory(
                    'isaaclab_height_scan_builder'
                ),
                'params',
                'go2_self_filter.yaml'
            ),

            'in_pointcloud_topic':
                LaunchConfiguration('output_topic'),

            'out_pointcloud_topic':
                '/filtered',

            'robot_description': Command(
                'cat ' + os.path.join(
                    get_package_share_directory('go2_description'),
                    'urdf',
                    'go2_description.urdf',
                )
            ),

            'model_path': LaunchConfiguration('model_path'),
            'model_format': LaunchConfiguration('model_format'),
            'frame_prefix': LaunchConfiguration('frame_prefix'),

            'lidar_sensor_type': '0',

        }.items(),
    )


    # ------------------------------------------------------------------
    # Height scan builder
    # ------------------------------------------------------------------

    height_scan_builder_node = Node(
        package='isaaclab_height_scan_builder',
        executable='height_scan_builder_node',
        name='height_scan_builder',
        output='screen',
        parameters=[
            {
                'width': 0.8,
                'height': 0.6,
                'resolution': 0.1,
                'offset_x': 0.4,
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'scan_cloud_topic': LaunchConfiguration('scan_cloud_topic'),
                'marker_topic': LaunchConfiguration('marker_topic'),
                'publish_markers': ParameterValue(
                    LaunchConfiguration('publish_markers'),
                    value_type=bool,
                ),
                'accumulation_time_sec': ParameterValue(
                    LaunchConfiguration('accumulation_time_sec'),
                    value_type=float,
                ),
                'outlier_radius': ParameterValue(
                    LaunchConfiguration('outlier_radius'),
                    value_type=float,
                ),
                'outlier_min_neighbors': ParameterValue(
                    LaunchConfiguration('outlier_min_neighbors'),
                    value_type=int,
                ),
                'enable_map_shift': ParameterValue(
                    LaunchConfiguration('enable_map_shift'),
                    value_type=bool,
                ),
                'map_shift_x': ParameterValue(
                    LaunchConfiguration('map_shift_x'),
                    value_type=float,
                ),
                'map_shift_y': ParameterValue(
                    LaunchConfiguration('map_shift_y'),
                    value_type=float,
                ),
                'map_shift_z': ParameterValue(
                    LaunchConfiguration('map_shift_z'),
                    value_type=float,
                ),

                'target_frame': ParameterValue(
                    PythonExpression([
                        '"world" if "',
                        LaunchConfiguration('world_frame'),
                        '" == "true" else "base_link"'
                    ]),
                    value_type=str,
                ),
            }
        ],
        arguments=[
            '--ros-args',
            '--log-level', 'tf2_buffer:=error',
        ]
    )


    return LaunchDescription([
        imu_topic_arg,
        base_link_arg,
        imu_frame_arg,
        lidar_topic1_name_arg,

        output_topic_arg,
        scan_cloud_topic_arg,
        marker_topic_arg,
        publish_markers_arg,
        accumulation_time_sec_arg,
        outlier_radius_arg,
        outlier_min_neighbors_arg,
        enable_map_shift_arg,
        map_shift_x_arg,
        map_shift_y_arg,
        map_shift_z_arg,

        use_sim_time_arg,

        world_frame_arg,
        publish_world_frame_arg,

        self_filter_arg,
        model_path_arg,
        model_format_arg,
        frame_prefix_arg,

        world_frame_publisher_node,
        point_cloud_manager_node,
        height_scan_builder_node,
        self_filter_launch,
    ])
