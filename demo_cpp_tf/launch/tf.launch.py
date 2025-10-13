from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # rviz_config=get_package_share_directory('hesai_ros_driver')+'/rviz/rviz2.rviz'
    return LaunchDescription([

        # # imu -> lidar
        Node(
             package='tf2_ros',
             executable='static_transform_publisher',
             arguments='0.52 0.0 0.25 1.57 0.0 0 base_link hesai_lidar'.split(' '),
             output='screen'
        ), 
        Node(
             package='tf2_ros',
             executable='static_transform_publisher',
             arguments='-0.55 -0.02 0.25 3.1415926 0.0 0.0 base_link ls_laser'.split(' '),
             output='screen'
        ),
        # base_link -> imu
        Node(
            name='imu_baselink_tf',
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments='0.0 0.0 0.25 0 0.0 0.0 base_link imu_link'.split(' '),
            output='screen',
            respawn=False
    	),
        #base_footprint -> base_link
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     arguments='0.0 0.0 0.45 0.0 0.0 0.0 base_footprint base_link'.split(' '),
        #     output='screen'
        # )

        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     arguments='0.0 0.0 0.0 0.0 0.0 0.0 odom base_link'.split(' '),
        #     output='screen'
        # )
    ])
