from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time',  default='true')
    this_directory = get_package_share_directory('stage_ros2')

    namespace = LaunchConfiguration('namespace')
    namespace_arg = DeclareLaunchArgument('namespace', default_value=TextSubstitution(text=''))

    rviz_config = LaunchConfiguration('config')
    rviz_config_arg = DeclareLaunchArgument(
        'config',
        default_value=TextSubstitution(text='empty'),
        description='Use empty, cave or roblab to load a TUW enviroment')
    
    def rviz_launch_configuration(context):
        file = os.path.join(
            this_directory,
            'config/rviz',
            context.launch_configurations['config'] + '.rviz')
        return [SetLaunchConfiguration('config', file)]

    rviz_launch_configuration_arg = OpaqueFunction(function=rviz_launch_configuration)

    return LaunchDescription([
        namespace_arg,
        rviz_config_arg,
        rviz_launch_configuration_arg,
        Node(
            package='rviz2',
            namespace=namespace,
            executable='rviz2',
            name='rviz2',
            arguments=['-d', [rviz_config]],
            parameters=[{
                "use_sim_time": use_sim_time}],
        )
    ])

    return LaunchDescription([
        # Declare use_sim_time parameter
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),

        # Start stage_ros2 node
        Node(
            package='stage_ros2',
            executable='stage_ros2',
            name='stage_ros2',
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            arguments=['$(find world)/cave_multi.world'],
            remappings=[('base_scan', 'laserscan')]
        ),

        # Start RViz node
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            output='screen',
            arguments=['-d$(find point_follow)/launch/point_follow_config.rviz']
        ),

        # Start map_server node
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            arguments=['$(find world)/cave_multi.world']
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
                {'initial_pose_x': -2.23},
                {'initial_pose_y': -13.26},
                {'initial_pose_a': 77.14},
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