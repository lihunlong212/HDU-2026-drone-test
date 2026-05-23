from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    """demo3 专用 PID 启动文件：

    demo3 使用双高度源：
    - 默认用 /laser_array/ground_height 控制巡航/放置/降落高度；
    - 抓取下降时任务节点切到 /laser_array/min_range，用距柱顶距离控制高度。
    """
    pid_kp_z = LaunchConfiguration("pid_kp_z")
    pid_max_linear_velocity = LaunchConfiguration("pid_max_linear_velocity")
    pid_max_vertical_velocity = LaunchConfiguration("pid_max_vertical_velocity")
    visual_kp = LaunchConfiguration("visual_kp")
    visual_kd = LaunchConfiguration("visual_kd")
    visual_pixel_deadzone = LaunchConfiguration("visual_pixel_deadzone")
    visual_max_xy_velocity = LaunchConfiguration("visual_max_xy_velocity")
    visual_data_timeout_sec = LaunchConfiguration("visual_data_timeout_sec")

    pid_params = {
        "control_frequency": 50.0,
        "map_frame": "map",
        "laser_link_frame": "laser_link",

        "kp_xy": 0.8, "ki_xy": 0.0, "kd_xy": 0.2,
        "kp_yaw": 1.0, "ki_yaw": 0.0, "kd_yaw": 0.2,
        "kp_z": ParameterValue(pid_kp_z, value_type=float), "ki_z": 0.0, "kd_z": 0.2,

        "max_linear_velocity": ParameterValue(pid_max_linear_velocity, value_type=float),
        "max_angular_velocity": 30.0,
        "max_vertical_velocity": ParameterValue(pid_max_vertical_velocity, value_type=float),

        # 视觉接管只在 CENTER / RECENTER_MID / 抓取下降时修正 XY。
        # 这里的速度单位是 cm/s；默认 0.4s 内收不到 /fine_data 时，PID 会把 XY 目标速度置 0。
        "visual_kp_x": ParameterValue(visual_kp, value_type=float), "visual_ki_x": 0.0, "visual_kd_x": ParameterValue(visual_kd, value_type=float),
        "visual_kp_y": ParameterValue(visual_kp, value_type=float), "visual_ki_y": 0.0, "visual_kd_y": ParameterValue(visual_kd, value_type=float),
        "visual_pixel_deadzone": ParameterValue(visual_pixel_deadzone, value_type=float),
        "visual_max_xy_velocity": ParameterValue(visual_max_xy_velocity, value_type=float),
        "visual_data_timeout_sec": ParameterValue(visual_data_timeout_sec, value_type=float),
        "dual_height_mode": True,
    }

    return LaunchDescription([
        DeclareLaunchArgument(
            "pid_kp_z", default_value="1.0",
            description="高度 PID 的 P 增益；高度跟随慢可略加，震荡就减小"),
        DeclareLaunchArgument(
            "pid_max_linear_velocity", default_value="33.0",
            description="普通地图巡航 XY 最大速度(cm/s)，不是视觉接管速度"),
        DeclareLaunchArgument(
            "pid_max_vertical_velocity", default_value="30.0",
            description="Z 轴最大速度(cm/s)，抓取下降太猛就调小"),
        DeclareLaunchArgument(
            "visual_kp", default_value="0.03",
            description="视觉接管 XY 比例增益；越大修正越快，也越容易抖"),
        DeclareLaunchArgument(
            "visual_kd", default_value="0.003",
            description="视觉接管 XY 微分增益；用于压抖，噪声大时不要调太高"),
        DeclareLaunchArgument(
            "visual_pixel_deadzone", default_value="10.0",
            description="视觉接管像素死区(px)，误差小于该值时 XY 速度为 0"),
        DeclareLaunchArgument(
            "visual_max_xy_velocity", default_value="6.0",
            description="视觉接管期间 XY 最大速度(cm/s)，抓取阶段建议小一点"),
        DeclareLaunchArgument(
            "visual_data_timeout_sec", default_value="0.4",
            description="视觉 /fine_data 超时时间(s)；超过后 XY 目标速度发 0"),
        Node(
            package="pid_control_pkg",
            executable="position_pid_controller",
            name="position_pid_controller",
            output="screen",
            parameters=[pid_params],
        )
    ])
