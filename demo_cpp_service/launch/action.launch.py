import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    action_include_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            [get_package_share_directory('turtlesim'), '/launch/', '/multisim.launch.py']))

    action_excuteprocess = launch.actions.ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/turtlesim1/spawn', 'turtlesim/srv/Spawn', '{x: 1.0, y: 1.0}'])

    action_log_info = launch.actions.LogInfo(msg="Spawned a turtle at (1.0, 1.0)")

    #延迟启动
    action_group = launch.actions.GroupAction([
        launch.actions.TimerAction(period=2.0, actions=[action_log_info]),
        launch.actions.TimerAction(period=3.0, actions=[action_excuteprocess])
    ])

    launch_description = launch.LaunchDescription([
        action_include_launch,
        action_group])
    return launch_description
