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

from launch import LaunchDescription
from launch_ros.actions import Node


'''
Parameter Description:
---
- Set laser scan directon: 
  1. Set counterclockwise, example: {'laser_scan_dir': True}
  2. Set clockwise,        example: {'laser_scan_dir': False}
- Angle crop setting, Mask data within the set angle range:
  1. Enable angle crop fuction:
    1.1. enable angle crop,  example: {'enable_angle_crop_func': True}
    1.2. disable angle crop, example: {'enable_angle_crop_func': False}
  2. Angle cropping interval setting:
  - The distance and intensity data within the set angle range will be set to 0.
  - angle >= 'angle_crop_min' and angle <= 'angle_crop_max' which is [angle_crop_min, angle_crop_max], unit is degress.
    example:
      {'angle_crop_min': 135.0}
      {'angle_crop_max': 225.0}
      which is [135.0, 225.0], angle unit is degress.
'''


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
    # LDROBOT LiDAR publisher node
    ldlidar_node = Node(
        package='ldlidar_stl_ros2',
        executable='ldlidar_stl_ros2_node',
        name='LD19',
        output='screen',
        parameters=[
            {'product_name': 'LDLiDAR_LD19'},
            {'topic_name': 'scan'},
            {'frame_id': 'ld19_base_link'},
            # {'frame_id': 'base_laser'},
            {'port_name': '/dev/ldlidar'},
            {'port_baudrate': 230400},
            {'laser_scan_dir': True},
            {'enable_angle_crop_func': False},
            {'angle_crop_min': 135.0},
            {'angle_crop_max': 225.0}
        ]
    )
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
                      'rviz' / 'ld19.rviz'),
            '--ros-args', '--log-level', 'error'
        ])

    ld = LaunchDescription()
    ld.add_action(arg_sensor_range_visible)
    ld.add_action(rsp_node)
    ld.add_action(rviz_node)
    ld.add_action(ldlidar_node)

    return ld
