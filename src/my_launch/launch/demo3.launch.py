import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """demo3：在 demo2 的基础上，把扫描任务换成"抓取任务"(pillar_pickup_mission)，
    同时：
      1) visual_node 发布方框几何中心到 /fine_data，供 PID 视觉接管使用；
      2) PID 巡航/放置用 /laser_array/ground_height，抓取下降切到
         /laser_array/min_range，用距柱顶距离控制高度。
    本文件不改动 demo2 相关启动。
    """
    show_display = LaunchConfiguration("show_display")
    camera_index = LaunchConfiguration("camera_index")
    camera_width = LaunchConfiguration("camera_width")
    camera_height = LaunchConfiguration("camera_height")
    camera_fps = LaunchConfiguration("camera_fps")
    process_fps = LaunchConfiguration("process_fps")
    rect_center_bias = LaunchConfiguration("rect_center_bias")

    pid_kp_z = LaunchConfiguration("pid_kp_z")
    pid_max_linear_velocity = LaunchConfiguration("pid_max_linear_velocity")
    pid_max_vertical_velocity = LaunchConfiguration("pid_max_vertical_velocity")
    visual_kp = LaunchConfiguration("visual_kp")
    visual_kd = LaunchConfiguration("visual_kd")
    visual_pixel_deadzone = LaunchConfiguration("visual_pixel_deadzone")
    visual_max_xy_velocity = LaunchConfiguration("visual_max_xy_velocity")
    visual_data_timeout_sec = LaunchConfiguration("visual_data_timeout_sec")

    position_tolerance_cm = LaunchConfiguration("position_tolerance_cm")
    height_tolerance_cm = LaunchConfiguration("height_tolerance_cm")
    visual_align1_timeout_sec = LaunchConfiguration("visual_align1_timeout_sec")
    visual_align2_timeout_sec = LaunchConfiguration("visual_align2_timeout_sec")
    visual_pixel_tol_px = LaunchConfiguration("visual_pixel_tol_px")
    visual_align_required_hits = LaunchConfiguration("visual_align_required_hits")
    visual_jump_px = LaunchConfiguration("visual_jump_px")
    visual_stale_sec = LaunchConfiguration("visual_stale_sec")
    pillar_visit_height_cm = LaunchConfiguration("pillar_visit_height_cm")
    land_align_height_cm = LaunchConfiguration("land_align_height_cm")
    land_recenter_drop_cm = LaunchConfiguration("land_recenter_drop_cm")
    grab_align_height_cm = LaunchConfiguration("grab_align_height_cm")
    grab_pick_height_cm = LaunchConfiguration("grab_pick_height_cm")
    grab_height_tolerance_cm = LaunchConfiguration("grab_height_tolerance_cm")
    drop_final_dy_cm = LaunchConfiguration("drop_final_dy_cm")
    drop_final_dx_cm = LaunchConfiguration("drop_final_dx_cm")
    drop_release_clearance_cm = LaunchConfiguration("drop_release_clearance_cm")
    drop_post_release_hover_sec = LaunchConfiguration("drop_post_release_hover_sec")
    arm_extend_sec = LaunchConfiguration("arm_extend_sec")
    hover_grab_sec = LaunchConfiguration("hover_grab_sec")
    pickup_check_observe_sec = LaunchConfiguration("pickup_check_observe_sec")
    pickup_observe_plate_frames_required = LaunchConfiguration("pickup_observe_plate_frames_required")
    pickup_max_attempts = LaunchConfiguration("pickup_max_attempts")
    measure_only_mode = LaunchConfiguration("measure_only_mode")

    # 包路径
    my_carto_pkg_share        = FindPackageShare(package='my_carto_pkg').find('my_carto_pkg')
    uart_to_stm32_pkg_share   = FindPackageShare(package='uart_to_stm32').find('uart_to_stm32')
    pid_control_pkg_share     = FindPackageShare(package='pid_control_pkg').find('pid_control_pkg')
    activity_control_pkg_share= FindPackageShare(package='activity_control_pkg').find('activity_control_pkg')
    pillar_detector_pkg_share = FindPackageShare(package='pillar_detector_pkg').find('pillar_detector_pkg')
    laser_array_pkg_share     = FindPackageShare(package='laser_array_pkg').find('laser_array_pkg')
    # 磁铁硬件由 uart_to_stm32 通过 /electromagnet_control (帧 0x33) 控制。

    fly_carto_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(my_carto_pkg_share, 'launch', 'fly_carto.launch.py')
        )
    )
    uart_to_stm32_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(uart_to_stm32_pkg_share, 'launch', 'uart_to_stm32.launch.py')
        )
    )
    # demo3 专用：PID z 轴支持地面高度/柱顶距离双源切换
    position_pid_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pid_control_pkg_share, 'launch', 'position_pid_controller_ground.launch.py')
        ),
        launch_arguments={
            "pid_kp_z": pid_kp_z,
            "pid_max_linear_velocity": pid_max_linear_velocity,
            "pid_max_vertical_velocity": pid_max_vertical_velocity,
            "visual_kp": visual_kp,
            "visual_kd": visual_kd,
            "visual_pixel_deadzone": visual_pixel_deadzone,
            "visual_max_xy_velocity": visual_max_xy_velocity,
            "visual_data_timeout_sec": visual_data_timeout_sec,
        }.items()
    )

    # demo3 专用：抓取任务节点。
    pillar_pickup_mission_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(activity_control_pkg_share, 'launch', 'pillar_pickup_mission.launch.py')
        ),
        launch_arguments={
            "position_tolerance_cm": position_tolerance_cm,
            "height_tolerance_cm": height_tolerance_cm,
            "visual_align1_timeout_sec": visual_align1_timeout_sec,
            "visual_align2_timeout_sec": visual_align2_timeout_sec,
            "visual_pixel_tol_px": visual_pixel_tol_px,
            "visual_align_required_hits": visual_align_required_hits,
            "visual_jump_px": visual_jump_px,
            "visual_stale_sec": visual_stale_sec,
            "pillar_visit_height_cm": pillar_visit_height_cm,
            "land_align_height_cm": land_align_height_cm,
            "land_recenter_drop_cm": land_recenter_drop_cm,
            "grab_align_height_cm": grab_align_height_cm,
            "grab_pick_height_cm": grab_pick_height_cm,
            "grab_height_tolerance_cm": grab_height_tolerance_cm,
            "drop_final_dy_cm": drop_final_dy_cm,
            "drop_final_dx_cm": drop_final_dx_cm,
            "drop_release_clearance_cm": drop_release_clearance_cm,
            "drop_post_release_hover_sec": drop_post_release_hover_sec,
            "arm_extend_sec": arm_extend_sec,
            "hover_grab_sec": hover_grab_sec,
            "pickup_check_observe_sec": pickup_check_observe_sec,
            "pickup_observe_plate_frames_required": pickup_observe_plate_frames_required,
            "pickup_max_attempts": pickup_max_attempts,
            "measure_only_mode": measure_only_mode,
        }.items()
    )
    pillar_detector_tf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pillar_detector_pkg_share, 'launch', 'pillar_detector_tf.launch.py')
        )
    )

    # visual_node 直接起（覆盖 center_source=square），不走 visual_pkg.launch.py
    visual_node = Node(
        package="visual_pkg",
        executable="visual_node",
        name="visual_node",
        output="screen",
        parameters=[{
            "camera_index": ParameterValue(camera_index, value_type=int),
            "width": ParameterValue(camera_width, value_type=int),
            "height": ParameterValue(camera_height, value_type=int),
            "camera_fps": ParameterValue(camera_fps, value_type=int),
            "process_fps": ParameterValue(process_fps, value_type=float),
            "show_display": ParameterValue(show_display, value_type=bool),
            "apriltag_code": -1,
            "center_source": "square",
            # 中心框优先指数：压掉对齐柱①时挤进画面边角的起停区 A 大方框(误锁→占比/对准跑偏)。
            # 越大越偏向画面正中的框；0=关闭(纯面积，旧行为)。试飞嫌还抢锁就加大(3~4)。
            "rect_center_bias": ParameterValue(rect_center_bias, value_type=float),
        }],
    )

    # 面阵激光地面高度（抗高台版）
    laser_array_ground_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(laser_array_pkg_share, 'launch', 'laser_array_ground.launch.py')
        )
    )

    return LaunchDescription([
        # ===== 相机/视觉检测参数 =====
        DeclareLaunchArgument(
            "show_display", default_value="false",
            description="是否显示 visual_node 调试窗口；比赛运行一般 false"),
        DeclareLaunchArgument(
            "camera_index", default_value="0",
            description="相机设备序号，插了多个摄像头时按实际设备修改"),
        DeclareLaunchArgument(
            "camera_width", default_value="640",
            description="相机采集宽度(px)，需和摄像头支持的分辨率一致"),
        DeclareLaunchArgument(
            "camera_height", default_value="480",
            description="相机采集高度(px)，需和摄像头支持的分辨率一致"),
        DeclareLaunchArgument(
            "camera_fps", default_value="30",
            description="相机采集帧率，过高会增加 CPU 压力"),
        DeclareLaunchArgument(
            "process_fps", default_value="15.0",
            description="视觉算法处理帧率；画面卡顿可降低，响应慢可提高"),
        DeclareLaunchArgument(
            "rect_center_bias", default_value="2.0",
            description="方框选择的居中优先系数；误锁远处大框就调大，0 表示只看面积"),

        # ===== PID / 视觉接管参数 =====
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
            description="PID 侧 /fine_data 超时时间(s)；超过后 XY 目标速度发 0"),

        # ===== 任务视觉对准判定 =====
        DeclareLaunchArgument(
            "position_tolerance_cm", default_value="9.0",
            description="地图航点到达 XY 容差(cm)，越小越准但更容易卡住"),
        DeclareLaunchArgument(
            "height_tolerance_cm", default_value="12.0",
            description="普通地面高度模式到达容差(cm)，巡航/放置/降落使用"),
        DeclareLaunchArgument(
            "visual_align1_timeout_sec", default_value="4.0",
            description="第一次视觉对准最大等待时间(s)，柱子/降落框都使用"),
        DeclareLaunchArgument(
            "visual_align2_timeout_sec", default_value="1.5",
            description="二次复对准最大等待时间(s)，用于更近距离精对"),
        DeclareLaunchArgument(
            "visual_pixel_tol_px", default_value="15",
            description="视觉认为已对准的像素容差(px)，抓取要求连续满足多帧"),
        DeclareLaunchArgument(
            "visual_align_required_hits", default_value="3",
            description="视觉误差进入容差后需要连续满足的帧数，抓取精对默认 3 帧"),
        DeclareLaunchArgument(
            "visual_jump_px", default_value="100",
            description="视觉中心单帧跳变过滤阈值(px)，误检跳太大时丢弃该帧"),
        DeclareLaunchArgument(
            "visual_stale_sec", default_value="0.4",
            description="任务侧视觉数据失效时间(s)，超过后不认为视觉可用"),

        # ===== 高度 / 抓取参数 =====
        DeclareLaunchArgument(
            "pillar_visit_height_cm", default_value="150.0",
            description="第二趟到柱子上方的巡航高度(cm)，使用地面高度控制"),
        DeclareLaunchArgument(
            "land_align_height_cm", default_value="150.0",
            description="降落前视觉对准起停区 B 的高度(cm)，要能看全黑框"),
        DeclareLaunchArgument(
            "land_recenter_drop_cm", default_value="60.0",
            description="降落第一次对准后下探多少厘米再二次对准；0 表示不二次对准"),
        DeclareLaunchArgument(
            "grab_align_height_cm", default_value="30.0",
            description="到柱子航点后，切柱顶距离控制并先下降到距柱顶高度(cm)做精对"),
        DeclareLaunchArgument(
            "grab_pick_height_cm", default_value="10.0",
            description="精对连续满足后，继续下降到距柱顶高度(cm)伸臂吸取"),
        DeclareLaunchArgument(
            "grab_height_tolerance_cm", default_value="3.0",
            description="抓取阶段柱顶距离高度容差(cm)，用于 30cm/10cm 到位判断"),

        # ===== 放置 / 机械臂 / 重试参数 =====
        DeclareLaunchArgument(
            "drop_final_dy_cm", default_value="-2.0",
            description="放置末段 y 偏置(cm)，map +y=画面左；偏左滚落可加到 -6~-7"),
        DeclareLaunchArgument(
            "drop_final_dx_cm", default_value="4.0",
            description="放置末段 x 偏置(cm)，map +x=画面正上方，正值往前补"),
        DeclareLaunchArgument(
            "drop_release_clearance_cm", default_value="10.0",
            description="放置时叠面上方预留高度(cm)，太低易蹭片，太高易飘落"),
        DeclareLaunchArgument(
            "drop_post_release_hover_sec", default_value="1.0",
            description="松磁后原地悬停时间(s)，等铁片稳定后再收臂"),
        DeclareLaunchArgument(
            "arm_extend_sec", default_value="1.2",
            description="放置时伸臂到位等待时间(s)，到位后才松电磁铁"),
        DeclareLaunchArgument(
            "hover_grab_sec", default_value="1.0",
            description="抓取到 10cm 后保持吸磁/伸臂的等待时间(s)"),
        DeclareLaunchArgument(
            "pickup_check_observe_sec", default_value="2.0",
            description="抓完爬升后观察铁片是否还在柱上的时间(s)"),
        DeclareLaunchArgument(
            "pickup_observe_plate_frames_required", default_value="3",
            description="观察期间连续看到铁片多少帧才判定抓取失败并重试"),
        DeclareLaunchArgument(
            "pickup_max_attempts", default_value="3",
            description="单个铁片最多抓取尝试次数"),
        DeclareLaunchArgument(
            "measure_only_mode", default_value="false",
            description="true 时只跑第一趟测高/占比后降落，不进入抓取叠放；正式比赛保持 false"),
        fly_carto_launch,
        uart_to_stm32_launch,
        position_pid_controller_launch,
        pillar_pickup_mission_launch,
        pillar_detector_tf_launch,
        visual_node,
        laser_array_ground_launch,
    ])
