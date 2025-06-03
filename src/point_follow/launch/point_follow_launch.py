import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetLaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the directory of this package
    this_directory = get_package_share_directory('point_follow')

    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    one_tf_tree_arg = DeclareLaunchArgument(
        'one_tf_tree',
        default_value='false',
        description='on true all tfs are published with a namespace on /tf and /tf_static'
    )
    enforce_prefixes_arg = DeclareLaunchArgument(
        'enforce_prefixes',
        default_value='false',
        description='on true a prefixes are used for a single robot environment'
    )
    use_static_transformations_arg = DeclareLaunchArgument(
        'use_static_transformations',
        default_value='true',
        description='Use static transformations for sensor frames!'
    )
    stage_world_arg = DeclareLaunchArgument(
        'world',
        default_value=TextSubstitution(text='cave'),
        description='World file relative to the project world file, without .world'
    )

    def stage_world_configuration(context):
        file = os.path.join(
            get_package_share_directory('stage_ros2'),
            'world',
            context.launch_configurations['world'] + '.world')
        return [SetLaunchConfiguration('world_file', file)]

    stage_world_configuration_arg = OpaqueFunction(function=stage_world_configuration)

    return LaunchDescription([
        use_sim_time_arg,
        one_tf_tree_arg,
        enforce_prefixes_arg,
        use_static_transformations_arg,
        stage_world_arg,
        stage_world_configuration_arg,

        # Start stage_ros2 node
        Node(
            package='stage_ros2',
            executable='stage_ros2',
            name='stage',
            output='screen',
            parameters=[{
                'one_tf_tree': LaunchConfiguration('one_tf_tree'),
                'enforce_prefixes': LaunchConfiguration('enforce_prefixes'),
                'use_static_transformations': LaunchConfiguration('use_static_transformations'),
                "world_file": LaunchConfiguration('world_file'),
                "gui": True
            }],
        ),

        # Start RViz node
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            output='screen',
            arguments=[
                '-d', os.path.join(this_directory, 'launch', 'point_follow_config.rviz')
            ]
        ),

        # Start map_server node
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            arguments=[
                os.path.join(this_directory, 'world', 'cave_multi.world')
            ]
        ),

        # Static transform publisher
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='link1_broadcaster',
            arguments=['0', '0', '0', '0', '0', '0', '1', 'map', 'odom']
        ),

        # Start AMCL node
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
                {'initial_pose_x': -7.0},
                {'initial_pose_y': -7.0},
                {'initial_pose_a': 45.0},
                {'odom_model_type': 'diff'},
                {'odom_alpha1': 0.2},
                {'odom_alpha2': 0.2},
                {'odom_alpha3': 0.8},
                {'odom_alpha4': 0.2},
                {'odom_alpha5': 0.1},
                {'laser_max_beams': 30},
                {'min_particles': 200},
                {'max_particles': 2000},
                {'kld_err': 0.05},
                {'kld_z': 0.99},
                {'laser_z_hit': 0.5},
                {'laser_z_short': 0.05},
                {'laser_z_max': 0.05},
                {'laser_z_rand': 0.5},
                {'laser_sigma_hit': 0.2},
                {'laser_lambda_short': 0.1},
                {'laser_model_type': 'likelihood_field'},
                {'laser_likelihood_max_dist': 100.0},
                {'laser_likelihood_min_dist': 90.0},
                {'update_min_d': 0.2},
                {'update_min_a': 0.5},
                {'odom_frame_id': 'odom'},
                {'resample_interval': 1},
                {'transform_tolerance': 0.1},
                {'recovery_alpha_slow': 0.0},
                {'recovery_alpha_fast': 0.0}
            ],
            remappings=[('scan', 'laserscan')]
        ),

        # Start custom node
        Node(
            package='point_follow',
            executable='point_approach',
            name='LocationStuff',
            output='screen'
        )
    ])