from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # 放置末段 xy 偏置：可在命令行直接覆盖，如 drop_final_dy_cm:=-6.0 drop_final_dx_cm:=4.0
    drop_final_dy_cm = LaunchConfiguration("drop_final_dy_cm")
    drop_final_dx_cm = LaunchConfiguration("drop_final_dx_cm")

    def p(name, value_type=float):
        return ParameterValue(LaunchConfiguration(name), value_type=value_type)

    return LaunchDescription([
        # ===== 常调：到点/高度容差 =====
        DeclareLaunchArgument(
            "position_tolerance_cm", default_value="9.0",
            description="地图航点到达 XY 容差(cm)，越小越准但更容易卡住"),
        DeclareLaunchArgument(
            "height_tolerance_cm", default_value="8.0",
            description="普通地面高度模式到达容差(cm)，巡航/放置/降落使用"),

        # ===== 常调：视觉对准判定 =====
        DeclareLaunchArgument(
            "visual_align1_timeout_sec", default_value="2.0",
            description="第一次视觉对准最大等待时间(s)，柱子/降落框都使用"),
        DeclareLaunchArgument(
            "visual_align2_timeout_sec", default_value="1.5",
            description="二次复对准最大等待时间(s)，用于更近距离精对"),
        DeclareLaunchArgument(
            "visual_pixel_tol_px", default_value="60",
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

        # ===== 常调：航行/降落高度 =====
        DeclareLaunchArgument(
            "pillar_visit_height_cm", default_value="140.0",
            description="第二趟到柱子上方的巡航高度(cm)，使用地面高度控制"),
        DeclareLaunchArgument(
            "land_align_height_cm", default_value="140.0",
            description="降落前视觉对准起停区 B 的高度(cm)，要能看全黑框"),
        DeclareLaunchArgument(
            "land_recenter_drop_cm", default_value="60.0",
            description="降落第一次对准后下探多少厘米再二次对准；0 表示不二次对准"),

        # ===== 常调：抓取高度 =====
        DeclareLaunchArgument(
            "grab_align_height_cm", default_value="38.0",
            description="到柱子航点后，切柱顶距离控制并先下降到距柱顶高度(cm)做精对"),
        DeclareLaunchArgument(
            "grab_pick_height_cm", default_value="13.0",
            description="精对连续满足后，继续下降到距柱顶高度(cm)伸臂吸取"),
        DeclareLaunchArgument(
            "grab_height_tolerance_cm", default_value="4.0",
            description="抓取阶段柱顶距离高度容差(cm)，用于 30cm/10cm 到位判断"),

        # ===== 常调：放置/机械臂时序 =====
        DeclareLaunchArgument(
            "drop_final_dy_cm", default_value="-2.0",
            description="放置末段 y 偏置(cm)，map +y=画面左；偏左滚落可加到 -6~-7"),
        DeclareLaunchArgument(
            "drop_final_dx_cm", default_value="4.0",
            description="放置末段 x 偏置(cm)，map +x=画面正上方，正值往前补"),
        DeclareLaunchArgument(
            "drop_release_clearance_cm", default_value="13.0",
            description="放置时叠面上方预留高度(cm)，太低易蹭片，太高易飘落"),
        DeclareLaunchArgument(
            "drop_post_release_hover_sec", default_value="1.0",
            description="松磁后原地悬停时间(s)，等铁片稳定后再收臂"),
        DeclareLaunchArgument(
            "arm_extend_sec", default_value="1.2",
            description="放置时伸臂到位等待时间(s)，到位后才松电磁铁"),

        # ===== 常调：抓取结果确认 =====
        DeclareLaunchArgument(
            "pickup_check_observe_sec", default_value="1.0",
            description="抓完爬升后观察铁片是否还在柱上的时间(s)"),
        DeclareLaunchArgument(
            "pickup_observe_plate_frames_required", default_value="2",
            description="观察期间连续看到铁片多少帧才判定抓取失败并重试"),
        DeclareLaunchArgument(
            "pickup_max_attempts", default_value="3",
            description="单个铁片最多抓取尝试次数"),

        Node(
            package="activity_control_pkg",
            executable="pillar_pickup_mission",
            name="pillar_pickup_mission",
            output="screen",
            parameters=[{
                "map_frame": "map",
                "laser_link_frame": "laser_link",

                # 到达容差
                "position_tolerance_cm": p("position_tolerance_cm"),
                "yaw_tolerance_deg": 5.0,
                "height_tolerance_cm": p("height_tolerance_cm"),

                # 航线数值
                # SCAN 阶段飞行高度：用于起飞和扫描终点，低空飞行便于柱子检测
                "flight_height_cm": 30.0,
                "land_height_cm": 20.0,
                "scan_end_x_cm": 250.0,
                "landing_x_cm": 250.0,
                "landing_y_cm": -250.0,
                "pillar_visit_height_cm": p("pillar_visit_height_cm"),
                "pillar_wait_timeout_sec": 3.0,

                # 精准降落：飞到对角起停区 B 上方先用视觉对准 B 黑色方框中心，再竖直降落。
                # land_visual_enable=False 则回退纯位置降落（旧行为）。
                # land_align_height_cm 是对准 B 框时的悬停高度（要能看全 ~50cm 框；默认=巡航高度，
                # 此高度 50cm 框居中、对角的 A 框在视野外，rect_center_bias 会优先锁正下方的 B）。
                "land_visual_enable": True,
                "land_align_height_cm": p("land_align_height_cm"),
                # 第一次对准后再下降 land_recenter_drop_cm 到中停高度，二次对准 B 框一次（更近、像素更准）再降到底。
                # 默认 60：从 150 降到 90 再对一次。设 0 则不分段（对准一次后直接降到底）。
                "land_recenter_drop_cm": p("land_recenter_drop_cm"),

                # 视觉对准
                "visual_align1_timeout_sec": p("visual_align1_timeout_sec"),
                "visual_align2_timeout_sec": p("visual_align2_timeout_sec"),
                "visual_pixel_tol_px": p("visual_pixel_tol_px", int),
                "visual_align_required_hits": p("visual_align_required_hits", int),
                "visual_jump_px": p("visual_jump_px", int),
                "visual_stale_sec": p("visual_stale_sec"),

                # 摄像头→机械臂吸取点 xy 偏置（cm，机体系）。用于“下降取物/放置”：
                # 机体按此偏置平移，让机械臂吸取点正对铁片中心再下降。
                # 2026-05-22：机械臂伸直后吸取点与摄像头光轴共线 → 真值 0/0（非占位）。
                # 铁片对到画面中心即可竖直下降取物。若伸臂后有微小残差再在此填小量微调。
                "arm_offset_dx_cm": 0.0,
                "arm_offset_dy_cm": 0.0,

                # 铁片占比采样（判大小，决定叠放顺序）。第一趟 CENTER 期间累积
                # /circle_area_ratio，少于此帧数则判该柱为空柱（无铁片）。
                "plate_min_ratio_frames": 3,

                # 题目发挥要求："找到黑铁片→滞空3s+声光提示"。第一趟 CENTER 占比帧够
                # （= 确认有铁片）即开蜂鸣器/LED(帧0x22)，悬停此时长后关声光再飞下一柱。
                # 空柱（占比不够）不触发声光。
                "survey_signal_hold_sec": 2.0,

                # 下降 / 抓取 / 叠放：抓取和放置下降都切到柱顶/叠面距离控制。
                "grab_align_height_cm": p("grab_align_height_cm"),
                "grab_pick_height_cm": p("grab_pick_height_cm"),
                "grab_height_tolerance_cm": p("grab_height_tolerance_cm"),
                # 放置末段不贴死：目标=距当前柱顶/叠面 drop_release_clearance(cm)。对准一次中心→直接降到此高度→
                # 伸臂→松磁→悬停 drop_post_release_hover_sec→收臂。避免贴近接触摩擦/下压/惯性把片带歪。
                "drop_release_clearance_cm": p("drop_release_clearance_cm"),
                "drop_post_release_hover_sec": p("drop_post_release_hover_sec"),
                # 放置末段额外 y 偏置(cm)：补电磁铁吸取点物理偏置，防铁片偏左(map+y)滚落。
                # 与 grab 同向(负)。可命令行覆盖：drop_final_dy_cm:=-6.0
                "drop_final_dy_cm": ParameterValue(drop_final_dy_cm, value_type=float),
                # 放置末段额外 x 偏置(cm)：map +x=画面正上方，正值往前补。放置专用，抓取不加。
                # 可命令行覆盖：drop_final_dx_cm:=4.0
                "drop_final_dx_cm": ParameterValue(drop_final_dx_cm, value_type=float),
                # 放置释放时序：先伸臂，等机械臂到位(arm_extend_sec)后松磁，再悬停(drop_post_release_hover_sec)收臂。
                "arm_extend_sec": p("arm_extend_sec"),

                # 空柱放置 anchor：视觉确认后记录当前实际位置，后续下降/叠放复用。
                # 首次允许最多约 24cm 修正（覆盖 32cm 大空柱半边长 16cm + 飞行误差），
                # 已有 anchor 后单次只允许小步更新，防止放完后视觉锁到铁片把目标带偏。
                "empty_pillar_side_cm": 32.0,
                "drop_visual_anchor_enable": True,
                "drop_anchor_max_correction_cm": 24.0,
                "drop_anchor_max_update_step_cm": 8.0,
                "drop_visual_circle_veto_sec": 0.6,

                # traverse_only_mode=true → 只跑第一趟读占比，然后降落，不抓取。
                "traverse_only_mode": False,

                # 抓取观察 + 重试：CLIMB_BACK 到位后悬停 N 秒看 /circle_area_ratio 是否仍非 NaN，
                # 仍能识别出铁片即视为抓取失败，最多重试 pickup_max_attempts 次
                "pickup_check_observe_sec": p("pickup_check_observe_sec"),
                # OBSERVE 期间连续 N 帧还能看到真黑圆盘，才判抓取失败；NaN/空柱不累计。
                "pickup_observe_plate_frames_required": p("pickup_observe_plate_frames_required", int),
                "pickup_max_attempts": p("pickup_max_attempts", int),
            }],
        )
    ])
