from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction
from ament_index_python.packages import get_package_share_directory
import os
import xacro


def generate_launch_description():

    pkg_path = get_package_share_directory('murphy_description')
    urdf_file = os.path.join(pkg_path, 'urdf', 'murphy.urdf.xacro')
    world_file = os.path.join(pkg_path, 'worlds', 'murphy.world')
    robot_desc = xacro.process_file(urdf_file).toxml()

    return LaunchDescription([

        ExecuteProcess(
            cmd=['gz', 'sim', '-r', world_file],
            output='screen'
        ),
        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
                '0', '0', '0',
                '0', '0', '0',
                'base_link',
                'murphy/base_link/lidar'
            ],
            parameters=[{'use_sim_time': True}],
            output='screen'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[
                {'robot_description': robot_desc},
                {'use_sim_time': True}
            ]
        ),

        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-topic', 'robot_description',
                '-name', 'murphy'
            ],
            parameters=[{'use_sim_time': True}],
            output='screen'
        ),

        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['joint_state_broadcaster'],
                    parameters=[{'use_sim_time': True}],
                    output='screen'
                ),
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=[
                        'diff_drive_controller',
                        '--controller-ros-args',
                        '--ros-args --remap /diff_drive_controller/cmd_vel:=/cmd_vel'
                    ],
                    parameters=[{'use_sim_time': True}],
                    output='screen'
                ),
            ]
        ),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
                '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V'
            ],
            parameters=[{'use_sim_time': True}],
            output='screen'
        ),
        
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            parameters=[os.path.join(pkg_path, 'config', 'ekf.yaml')],
            output='screen'
        ),
        
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[os.path.join(pkg_path, 'config', 'slam.yaml')],
            output='screen'
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            parameters=[{'use_sim_time': True}],
            output='screen'
        )
    ])
