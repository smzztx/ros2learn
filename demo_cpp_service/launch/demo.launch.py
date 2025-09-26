import launch
import launch_ros

action_declare_arg_max_speed = launch.actions.DeclareLaunchArgument('launch_max_speed', default_value='2.0', description='max speed of turtle')

def generate_launch_description():
    action_node_turtle_control = launch_ros.actions.Node(
        package='demo_cpp_service',
        executable='turtle_control',
        name='turtle_control',
        output='screen',
        parameters=[{'max_speed': launch.substitutions.LaunchConfiguration('launch_max_speed', default='2.0')}]
    )
    action_node_patrol_client = launch_ros.actions.Node(
        package='demo_cpp_service',
        executable='patrol_client',
        name='patrol_client',
        output='log'
    )
    action_node_turtlesim = launch_ros.actions.Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='turtlesim',
        output='both'
    )
    launch_description = launch.LaunchDescription([
        action_declare_arg_max_speed,
        action_node_turtle_control,
        action_node_patrol_client,
        action_node_turtlesim
    ])
    return launch_description