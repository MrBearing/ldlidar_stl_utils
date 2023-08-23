from pathlib import Path
from typing import Dict
from typing import List
from typing import Tuple

from ament_index_python import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import Shutdown
from launch.substitutions import Command
from launch.substitutions import FindExecutable
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node


def _load_robot_description(
        xacro_filepath: Path, xacro_options: List[Tuple] = False) -> Dict:
    """Load robot description."""
    if 'xacro' in str(xacro_filepath):
        params = []
        if xacro_options:
            for xacro_option in xacro_options:
                params.append(' {}:='.format(
                    xacro_option[0]))
                params.append(xacro_option[1])
        command = [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            str(xacro_filepath)]
        robot_description_content = Command(command=(command + params))
    else:
        try:
            with open(str(xacro_filepath), 'r') as file:
                robot_description_content = file
        except EnvironmentError:
            exit(1)

    return {'robot_description': robot_description_content}


def generate_launch_description():
    """Launch rviz display."""
    arg_sensor_range_visible = DeclareLaunchArgument(
        'sensor_range_visible',
        default_value=TextSubstitution(text='False'),
        description='true : ldlidar_stl\'s sensor range is visible in rviz')

    sensor_range_visible = LaunchConfiguration(
        'sensor_range_visible', default='False')
    xacro_filepath_ = get_package_share_path(
        'ldlidar_stl_description') / 'urdf' / 'ld19.urdf.xacro'
    robot_description = _load_robot_description(
        xacro_filepath=xacro_filepath_,
        xacro_options={
            'sensor_range_visible': sensor_range_visible,
        }.items())

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='log',
        parameters=[robot_description])
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=[
            '-d', str(get_package_share_path('ldlidar_stl_bringup') /
                      'rviz' / 'ld19_display.rviz'),
            '--ros-args', '--log-level', 'error'
        ])

    ld = LaunchDescription()
    ld.add_action(arg_sensor_range_visible)
    ld.add_action(rsp_node)
    ld.add_action(rviz_node)
    return ld
