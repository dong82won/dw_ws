#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os
from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration # [추가된 Import]

def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_boxbot_simulation = get_package_share_directory('boxbot_simulation')

    # 모델 경로 설정을 위한 준비
    description_package_name = "boxbot_description"
    install_dir = get_package_prefix(description_package_name)

    # 모델 경로들
    gazebo_models_path = os.path.join(pkg_boxbot_simulation, 'models')

    # 환경 변수 추가 (기존 경로 유지 + 새 경로 추가)
    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] += f":{install_dir}/share:{gazebo_models_path}"
    else:
        os.environ['GAZEBO_MODEL_PATH'] = f"{install_dir}/share:{gazebo_models_path}"

    if 'GAZEBO_PLUGIN_PATH' in os.environ:
        os.environ['GAZEBO_PLUGIN_PATH'] += f":{install_dir}/lib"
    else:
        os.environ['GAZEBO_PLUGIN_PATH'] = f"{install_dir}/lib"

    print(f"GAZEBO MODELS PATH: {os.environ['GAZEBO_MODEL_PATH']}")
    print(f"GAZEBO PLUGINS PATH: {os.environ['GAZEBO_PLUGIN_PATH']}")

    # Gazebo 실행
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')),
        launch_arguments={'world': LaunchConfiguration('world')}.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=[os.path.join(pkg_boxbot_simulation, 'worlds', 'box_bot_empty2.world'), ''],
            description='SDF world file'),
            gazebo
    ])