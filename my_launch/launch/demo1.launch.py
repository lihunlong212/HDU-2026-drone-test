import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def _package_launch(package_name: str, filename: str) -> str:
    package_share = FindPackageShare(package=package_name).find(package_name)
    return os.path.join(package_share, "launch", filename)


def generate_launch_description() -> LaunchDescription:
    fly_carto_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(_package_launch("my_carto_pkg", "fly_carto.launch.py"))
    )
    uart_to_stm32_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(_package_launch("uart_to_stm32", "uart_to_stm32.launch.py"))
    )
    laser_array_ground_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            _package_launch("laser_array_pkg", "laser_array_ground.launch.py")
        )
    )
    pillar_detector_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            _package_launch("pillar_detector_pkg", "pillar_detector.launch.py")
        )
    )
    position_pid_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            _package_launch("pid_control_pkg", "position_pid_controller.launch.py")
        )
    )
    route_test_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(_package_launch("activity_control_pkg", "route_test.launch.py"))
    )
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            _package_launch("drone_camera_pkg", "black_circle_camera.launch.py")
        )
    )

    return LaunchDescription(
        [
            fly_carto_launch,
            TimerAction(period=2.0, actions=[uart_to_stm32_launch]),
            TimerAction(period=3.0, actions=[laser_array_ground_launch]),
            TimerAction(period=4.0, actions=[pillar_detector_launch]),
            TimerAction(period=5.0, actions=[camera_launch]),
            TimerAction(period=7.0, actions=[position_pid_controller_launch]),
            TimerAction(period=9.0, actions=[route_test_launch]),
        ]
    )
