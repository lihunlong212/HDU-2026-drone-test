#include "activity_control_pkg/pillar_pickup_mission_node.hpp"

#include <algorithm>
#include <angles/angles.h>
#include <chrono>
#include <cmath>
#include <limits>
#include <numeric>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace activity_control_pkg
{

PillarPickupMissionNode::PillarPickupMissionNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("pillar_pickup_mission", options),
  phase_(MissionPhase::SCAN),
  scan_end_idx_(1),
  current_idx_(0),
  pillars_received_(false),
  survey_iter_(0),
  survey_sub_(SurveySub::APPROACH),
  survey_collecting_ratio_(false),
  empty_idx_(0),
  pickup_iter_(0),
  pickup_sub_(PickupSub::APPROACH),
  has_fine_(false),
  last_fine_dx_(0),
  last_fine_dy_(0),
  visual_takeover_active_(false),
  is_hovering_(false),
  has_area_height_(false),
  area_height_cm_(0.0),
  has_pillar_top_distance_(false),
  pillar_top_distance_cm_(0.0),
  mission_complete_sent_(false)
{
  map_frame_        = declare_parameter<std::string>("map_frame",        "map");
  laser_link_frame_ = declare_parameter<std::string>("laser_link_frame", "laser_link");

  pos_tol_cm_    = declare_parameter("position_tolerance_cm",  9.0);
  yaw_tol_deg_   = declare_parameter("yaw_tolerance_deg",      5.0);
  height_tol_cm_ = declare_parameter("height_tolerance_cm",   12.0);

  flight_height_cm_        = declare_parameter("flight_height_cm",        40.0);
  land_height_cm_          = declare_parameter("land_height_cm",           4.0);
  scan_end_x_cm_           = declare_parameter("scan_end_x_cm",          250.0);
  landing_x_cm_            = declare_parameter("landing_x_cm",           250.0);
  landing_y_cm_            = declare_parameter("landing_y_cm",          -250.0);
  pillar_visit_height_cm_  = declare_parameter("pillar_visit_height_cm", 150.0);
  pillar_wait_timeout_sec_ = declare_parameter("pillar_wait_timeout_sec", 3.0);

  // 精准降落：飞到 B 上方先视觉对准 B 黑框中心再竖直降落。对准高度默认=巡航高度(已验证能看全框)。
  land_visual_enable_      = declare_parameter("land_visual_enable",      true);
  land_align_height_cm_    = declare_parameter("land_align_height_cm",    pillar_visit_height_cm_);
  land_recenter_drop_cm_   = declare_parameter("land_recenter_drop_cm",   60.0);

  visual_align1_timeout_sec_  = declare_parameter("visual_align1_timeout_sec", 4.0);
  visual_align2_timeout_sec_  = declare_parameter("visual_align2_timeout_sec", 1.5);
  visual_pixel_tol_           = declare_parameter("visual_pixel_tol_px",      15);
  visual_align_required_hits_ = declare_parameter("visual_align_required_hits", 2);
  visual_jump_px_             = declare_parameter("visual_jump_px",          100);
  visual_stale_sec_           = declare_parameter("visual_stale_sec",        0.4);
  arm_offset_dx_cm_ = declare_parameter("arm_offset_dx_cm", CAM_TO_ARM_DX_CM);
  arm_offset_dy_cm_ = declare_parameter("arm_offset_dy_cm", CAM_TO_ARM_DY_CM);

  plate_min_ratio_frames_ = declare_parameter("plate_min_ratio_frames", 3);

  survey_signal_hold_sec_ = declare_parameter("survey_signal_hold_sec", 3.0);

  grab_align_height_cm_         = declare_parameter("grab_align_height_cm",         38.0);
  grab_height_tol_cm_           = declare_parameter("grab_height_tolerance_cm",      4.0);
  grab_descend_delta_cm_        = declare_parameter("grab_descend_delta_cm",        26.0);
  grab_hold_sec_                = declare_parameter("grab_hold_sec",                 1.0);
  grab_final_timeout_sec_       = declare_parameter("grab_final_timeout_sec",        5.0);
  grab_check_height_cm_         = declare_parameter("grab_check_height_cm",         40.0);
  drop_align_height_cm_        = declare_parameter("drop_align_height_cm",        38.0);
  drop_release_clearance_cm_   = declare_parameter("drop_release_clearance_cm",   25.0); // 放置投递高度
  drop_final_dy_cm_   = declare_parameter("drop_final_dy_cm",   -2.0);  // 放置末段 y 偏置（补吸取点偏置，防偏左滚落；偏左明显可加到 -6~-7）
  drop_final_dx_cm_   = declare_parameter("drop_final_dx_cm",    4.0);  // 放置末段 x 偏置（map +x=画面正上方，正值往前补）
  arm_extend_sec_     = declare_parameter("arm_extend_sec",      1.2);  // 放置伸臂到位耗时
  drop_magnet_delay_sec_ = declare_parameter("drop_magnet_delay_sec", 1.0);
  empty_pillar_side_cm_           = declare_parameter("empty_pillar_side_cm", 32.0);
  drop_visual_anchor_enable_      = declare_parameter("drop_visual_anchor_enable",      true);
  drop_anchor_max_correction_cm_  = declare_parameter(
    "drop_anchor_max_correction_cm", empty_pillar_side_cm_ * 0.5 + 8.0);
  drop_anchor_max_update_step_cm_ = declare_parameter("drop_anchor_max_update_step_cm", 8.0);
  drop_visual_circle_veto_sec_    = declare_parameter("drop_visual_circle_veto_sec",    0.6);
  traverse_only_mode_ = declare_parameter("traverse_only_mode",  false);
  target_republish_period_sec_ = declare_parameter("target_republish_period_sec", 1.0);

  pickup_check_observe_sec_ = declare_parameter("pickup_check_observe_sec", 2.0);
  pickup_max_attempts_      = declare_parameter("pickup_max_attempts",      2);
  pickup_observe_plate_frames_required_ =
    declare_parameter("pickup_observe_plate_frames_required", 3);

  // ── SCAN 航点 ──
  waypoints_ = {
    PickupWaypoint{0.0,            0.0, flight_height_cm_, 0.0, 0.0, "takeoff"},
    PickupWaypoint{scan_end_x_cm_, 0.0, flight_height_cm_, 0.0, 0.0, "scan_end"},
  };
  scan_end_idx_ = 1;

  tf_buffer_   = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  auto durable_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
  target_pub_            = create_publisher<std_msgs::msg::Float32MultiArray>("/target_position", durable_qos);
  enable_pub_            = create_publisher<std_msgs::msg::Bool>("/pillar_detect_enable", durable_qos);
  visual_takeover_pub_   = create_publisher<std_msgs::msg::Bool>("/visual_takeover", durable_qos);
  visual_takeover_active_pub_ =
    create_publisher<std_msgs::msg::Bool>("/visual_takeover_active", durable_qos);
  active_controller_pub_ = create_publisher<std_msgs::msg::UInt8>("/active_controller", durable_qos);
  route_choice_pub_      = create_publisher<std_msgs::msg::UInt8>("/route_choice", rclcpp::QoS(10).reliable());
  servo_pub_             = create_publisher<std_msgs::msg::UInt8>("/servo_control", rclcpp::QoS(10).reliable());
  electromagnet_pub_     = create_publisher<std_msgs::msg::UInt8>("/electromagnet_control", rclcpp::QoS(10).reliable());
  height_control_mode_pub_ =
    create_publisher<std_msgs::msg::UInt8>("/height_control_mode", durable_qos);
  buzzer_led_pub_        = create_publisher<std_msgs::msg::UInt8>("/buzzer_led_control", rclcpp::QoS(10).reliable());
  mission_complete_pub_  = create_publisher<std_msgs::msg::Empty>("/mission_complete", rclcpp::QoS(10).reliable());
  pickup_done_pub_       = create_publisher<std_msgs::msg::Empty>("/pickup_done", rclcpp::QoS(10).reliable());
  pickup_failed_pub_     = create_publisher<std_msgs::msg::Empty>("/pickup_failed", rclcpp::QoS(10).reliable());

  area_height_sub_ = create_subscription<std_msgs::msg::Int16>(
    "/laser_array/ground_height", rclcpp::QoS(10),
    std::bind(&PillarPickupMissionNode::areaHeightCallback, this, std::placeholders::_1));
  pillar_top_distance_sub_ = create_subscription<std_msgs::msg::Float32>(
    "/laser_array/min_range", rclcpp::QoS(10),
    std::bind(&PillarPickupMissionNode::pillarTopDistanceCallback, this, std::placeholders::_1));
  fine_data_sub_ = create_subscription<std_msgs::msg::Int32MultiArray>(
    "/fine_data", rclcpp::QoS(10),
    std::bind(&PillarPickupMissionNode::fineDataCallback, this, std::placeholders::_1));
  circle_ratio_sub_ = create_subscription<std_msgs::msg::Float32>(
    "/circle_area_ratio", rclcpp::QoS(10),
    std::bind(&PillarPickupMissionNode::circleRatioCallback, this, std::placeholders::_1));

  auto pillars_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
  pillars_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
    "/detected_pillars", pillars_qos,
    std::bind(&PillarPickupMissionNode::pillarsCallback, this, std::placeholders::_1));

  publishEnable(false);
  publishVisualTakeover(false);
  publishHeightControlMode(false);
  publishBuzzerLed(false);   // 上电关闭声光
  republish_enabled_ = true;
  publishTarget(waypoints_[0]);

  monitor_timer_ = create_wall_timer(
    std::chrono::milliseconds(50),
    std::bind(&PillarPickupMissionNode::monitorTimerCallback, this));

  if (target_republish_period_sec_ > 0.0) {
    target_republish_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(target_republish_period_sec_)),
      [this]() {
        std::lock_guard<std::mutex> lock(mutex_);
        if (phase_ == MissionPhase::DONE || phase_ == MissionPhase::WAIT_PILLARS) { return; }
        if (!republish_enabled_) { return; }
        publishTarget(last_target_, false);
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3000,
          "重发当前目标 [%s] 以保证 PID 订阅侧不会漏收", last_target_.tag);
      });
  }

  // 打开 uart_to_stm32 的 /target_velocity 转发：需先收到受支持的 /route_choice。
  route_choice_kick_count_ = 0;
  route_choice_kick_timer_ = create_wall_timer(
    std::chrono::milliseconds(500),
    [this]() {
      std_msgs::msg::UInt8 m;
      m.data = 1;
      route_choice_pub_->publish(m);
      ++route_choice_kick_count_;
      if (route_choice_kick_count_ == 1) {
        RCLCPP_INFO(get_logger(), "已发布 /route_choice=1，启用 uart_to_stm32 目标速度转发");
      }
      if (route_choice_kick_count_ >= 10) {
        route_choice_kick_timer_->cancel();
      }
    });

  RCLCPP_INFO(get_logger(),
    "PICKUP 任务启动（两趟）: 起飞→扫描→[第一趟读铁片占比]→按占比大到小→[第二趟抓取叠放]→降落(%.0f,%.0f)",
    landing_x_cm_, landing_y_cm_);
  RCLCPP_INFO(get_logger(),
    "任务模式: %s", traverse_only_mode_
      ? "只跑第一趟(占比)然后降落，不抓取——试飞用"
      : "full（第一趟占比 + 第二趟抓取叠放）");
  RCLCPP_INFO(get_logger(),
    "高度控制: 巡航/降落用地面高度，抓取/放置下降用 /laser_array/min_range 距柱顶/叠面距离");
}

// ─────────── 回调 ───────────

void PillarPickupMissionNode::areaHeightCallback(const std_msgs::msg::Int16::SharedPtr msg)
{
  area_height_cm_ = static_cast<double>(msg->data);
  has_area_height_ = true;
}

void PillarPickupMissionNode::pillarTopDistanceCallback(const std_msgs::msg::Float32::SharedPtr msg)
{
  if (!std::isfinite(msg->data) || msg->data <= 0.0f) {
    return;
  }
  pillar_top_distance_cm_ = static_cast<double>(msg->data) * 100.0;
  has_pillar_top_distance_ = true;
}

void PillarPickupMissionNode::pillarsCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  detected_pillars_cm_.clear();
  if (msg->data.size() % 2 != 0) {
    RCLCPP_WARN(get_logger(), "/detected_pillars 数据长度异常: %zu", msg->data.size());
    pillars_received_ = true;
    return;
  }
  for (std::size_t k = 0; k + 1 < msg->data.size(); k += 2) {
    detected_pillars_cm_.emplace_back(
      static_cast<double>(msg->data[k]) * 100.0,
      static_cast<double>(msg->data[k + 1]) * 100.0);
  }
  pillars_received_ = true;
  RCLCPP_INFO(get_logger(), "收到 /detected_pillars: %zu 个柱子", detected_pillars_cm_.size());
}

void PillarPickupMissionNode::fineDataCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
{
  if (msg->data.size() < 2) { return; }
  recordFineData(static_cast<int>(msg->data[0]), static_cast<int>(msg->data[1]));
}

void PillarPickupMissionNode::circleRatioCallback(const std_msgs::msg::Float32::SharedPtr msg)
{
  if (std::isnan(msg->data)) {
    if (phase_ == MissionPhase::PICKUP && pickup_sub_ == PickupSub::OBSERVE_GRAB) {
      pickup_observed_plate_frames_ = 0;
    }
    return;
  }
  last_circle_ratio_ = static_cast<double>(msg->data);
  has_circle_ratio_  = true;
  last_circle_valid_time_ = now();
  // 第一趟 ALIGN 期间采集占比样本（判铁片大小）
  if (survey_collecting_ratio_) {
    plate_ratio_samples_.push_back(static_cast<double>(msg->data));
  }
  // 第二趟 OBSERVE 期间连续多帧认出真圆 = 铁片还在原位 = 抓取失败。
  if (phase_ == MissionPhase::PICKUP && pickup_sub_ == PickupSub::OBSERVE_GRAB) {
    ++pickup_observed_plate_frames_;
    if (pickup_observed_plate_frames_ >= std::max(1, pickup_observe_plate_frames_required_)) {
      pickup_observed_plate_ = true;
    }
  }
}

// ─────────── 发布工具 ───────────

void PillarPickupMissionNode::publishTarget(const PickupWaypoint & wp, bool verbose)
{
  last_target_ = wp;
  std_msgs::msg::Float32MultiArray msg;
  msg.data.resize(4);
  msg.data[0] = static_cast<float>(wp.x_cm);
  msg.data[1] = static_cast<float>(wp.y_cm);
  msg.data[2] = static_cast<float>(wp.z_cm);
  msg.data[3] = static_cast<float>(wp.yaw_deg);
  target_pub_->publish(msg);
  publishActiveController(2);
  if (verbose) {
    RCLCPP_INFO(get_logger(),
      "发布目标 [%s]: x=%.1f y=%.1f z=%.1f yaw=%.1f",
      wp.tag, wp.x_cm, wp.y_cm, wp.z_cm, wp.yaw_deg);
  }
}

void PillarPickupMissionNode::publishEnable(bool on)
{
  std_msgs::msg::Bool m;
  m.data = on;
  enable_pub_->publish(m);
}

void PillarPickupMissionNode::publishVisualTakeover(bool on)
{
  std_msgs::msg::Bool m;
  m.data = on;
  visual_takeover_pub_->publish(m);
  visual_takeover_active_pub_->publish(m);
  visual_takeover_active_ = on;
  RCLCPP_INFO(get_logger(), "/visual_takeover = %s, /visual_takeover_active = %s",
    on ? "true" : "false", on ? "true" : "false");
}

void PillarPickupMissionNode::publishActiveController(uint8_t mode)
{
  std_msgs::msg::UInt8 m;
  m.data = mode;
  active_controller_pub_->publish(m);
}

void PillarPickupMissionNode::publishServo(bool extended)
{
  std_msgs::msg::UInt8 m;
  m.data = extended ? 0x01 : 0x00;
  servo_pub_->publish(m);
  RCLCPP_INFO(get_logger(), "→ /servo_control = 0x%02X (机械臂 %s)",
    static_cast<unsigned>(m.data), extended ? "伸出" : "收起");
}

void PillarPickupMissionNode::publishMagnet(bool on)
{
  std_msgs::msg::UInt8 m;
  m.data = on ? 0x01 : 0x00;
  electromagnet_pub_->publish(m);
  RCLCPP_INFO(get_logger(), "→ /electromagnet_control = 0x%02X (%s)",
    static_cast<unsigned>(m.data), on ? "吸" : "松");
}

void PillarPickupMissionNode::publishHeightControlMode(bool pillar_top)
{
  std_msgs::msg::UInt8 m;
  m.data = pillar_top ? 0x01 : 0x00;
  height_control_mode_pub_->publish(m);
  RCLCPP_INFO(get_logger(), "→ /height_control_mode = 0x%02X (%s)",
    static_cast<unsigned>(m.data), pillar_top ? "柱顶距离" : "地面高度");
}

void PillarPickupMissionNode::publishBuzzerLed(bool on)
{
  std_msgs::msg::UInt8 m;
  m.data = on ? 0x01 : 0x00;
  buzzer_led_pub_->publish(m);
  RCLCPP_INFO(get_logger(), "→ /buzzer_led_control = 0x%02X (声光 %s)",
    static_cast<unsigned>(m.data), on ? "开" : "关");
}

// ─────────── 姿态 / 到位 ───────────

bool PillarPickupMissionNode::getCurrentPose(double & x_cm, double & y_cm, double & yaw_deg)
{
  try {
    geometry_msgs::msg::TransformStamped tf = tf_buffer_->lookupTransform(
      map_frame_, laser_link_frame_, tf2::TimePointZero);
    x_cm = meterToCm(tf.transform.translation.x);
    y_cm = meterToCm(tf.transform.translation.y);
    tf2::Quaternion q;
    tf2::fromMsg(tf.transform.rotation, q);
    double roll = 0.0, pitch = 0.0, yaw = 0.0;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    yaw_deg = radToDeg(yaw);
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "TF %s <- %s 失败: %s", map_frame_.c_str(), laser_link_frame_.c_str(), ex.what());
    return false;
  }
}

bool PillarPickupMissionNode::isReached(const PickupWaypoint & wp, double x_cm, double y_cm,
                                        double z_cm, double yaw_deg) const
{
  const double dxy = std::hypot(wp.x_cm - x_cm, wp.y_cm - y_cm);
  const double dz  = std::fabs(wp.z_cm - z_cm);
  const double dyaw = std::fabs(normalizeAngleDeg(wp.yaw_deg - yaw_deg));

  const bool xy_ok  = dxy <= pos_tol_cm_;
  if (phase_ == MissionPhase::PICKUP && !descend_is_drop_ &&
      pickup_sub_ == PickupSub::DESCEND_FINAL) {
    // 抓取最终下降用地面高度相对下降，不吃高度容差：必须到目标读数或更低。
    return z_cm <= wp.z_cm;
  }
  if (phase_ == MissionPhase::PICKUP && !descend_is_drop_ &&
      pickup_sub_ == PickupSub::CLIMB_BACK) {
    // 抓完后只爬到距柱顶观察高度，不回巡航高度。
    return z_cm >= wp.z_cm;
  }
  const bool grab_height_phase =
    phase_ == MissionPhase::PICKUP &&
    (pickup_sub_ == PickupSub::DESCEND_MID ||
     pickup_sub_ == PickupSub::RECENTER_MID ||
     pickup_sub_ == PickupSub::DESCEND_FINAL ||
     pickup_sub_ == PickupSub::HOVER_GRAB ||
     pickup_sub_ == PickupSub::CLIMB_BACK);
  const bool z_ok   = dz  <= (grab_height_phase ? grab_height_tol_cm_ : height_tol_cm_);
  const bool yaw_ok = dyaw <= yaw_tol_deg_;

  if (grab_height_phase) {
    return z_ok;
  }

  if (wp.z_cm > 20.0) {
    return z_ok && xy_ok;
  }
  return z_ok && xy_ok && yaw_ok;
}

// ─────────── 视觉对准 ───────────

void PillarPickupMissionNode::recordFineData(int dx_px, int dy_px)
{
  last_fine_dx_ = dx_px;
  last_fine_dy_ = dy_px;
  has_fine_ = true;
  last_fine_time_ = now();
  fine_hist_.emplace_back(dx_px, dy_px);
  while (fine_hist_.size() > static_cast<std::size_t>(std::max(1, visual_align_required_hits_))) {
    fine_hist_.pop_front();
  }
}

bool PillarPickupMissionNode::isVisuallyAligned() const
{
  if (static_cast<int>(fine_hist_.size()) < visual_align_required_hits_) { return false; }
  for (const auto & p : fine_hist_) {
    if (std::abs(p.first)  > visual_pixel_tol_) { return false; }
    if (std::abs(p.second) > visual_pixel_tol_) { return false; }
  }
  return true;
}

void PillarPickupMissionNode::getDropTargetXY(double & x_cm, double & y_cm) const
{
  x_cm = has_drop_anchor_ ? drop_anchor_x_cm_ : empty_pillar_x_cm_;
  y_cm = has_drop_anchor_ ? drop_anchor_y_cm_ : empty_pillar_y_cm_;
}

bool PillarPickupMissionNode::dropVisionCircleVeto() const
{
  if (stack_count_ <= 0 || !has_circle_ratio_ || drop_visual_circle_veto_sec_ <= 0.0) {
    return false;
  }
  return (now() - last_circle_valid_time_).seconds() <= drop_visual_circle_veto_sec_;
}

void PillarPickupMissionNode::updateDropAnchorFromVision(
  double x_cm, double y_cm, const char * reason)
{
  if (!drop_visual_anchor_enable_) { return; }
  if (!isVisuallyAligned()) {
    RCLCPP_WARN(get_logger(),
      "[drop anchor] %s: 视觉未稳定对齐，保留%s (%.1f,%.1f)",
      reason, has_drop_anchor_ ? "已有 anchor" : "点云空柱坐标",
      has_drop_anchor_ ? drop_anchor_x_cm_ : empty_pillar_x_cm_,
      has_drop_anchor_ ? drop_anchor_y_cm_ : empty_pillar_y_cm_);
    return;
  }
  if (dropVisionCircleVeto()) {
    RCLCPP_WARN(get_logger(),
      "[drop anchor] %s: 已叠放后近期检测到圆盘(ratio=%.3f)，疑似锁到铁片，拒绝更新",
      reason, last_circle_ratio_);
    return;
  }

  const double nominal_delta = std::hypot(x_cm - empty_pillar_x_cm_, y_cm - empty_pillar_y_cm_);
  if (nominal_delta > drop_anchor_max_correction_cm_) {
    RCLCPP_WARN(get_logger(),
      "[drop anchor] %s: 视觉修正 %.1fcm > %.1fcm，拒绝更新，回退%s",
      reason, nominal_delta, drop_anchor_max_correction_cm_,
      has_drop_anchor_ ? "已有 anchor" : "点云空柱坐标");
    return;
  }

  if (has_drop_anchor_) {
    const double update_step = std::hypot(x_cm - drop_anchor_x_cm_, y_cm - drop_anchor_y_cm_);
    if (update_step > drop_anchor_max_update_step_cm_) {
      RCLCPP_WARN(get_logger(),
        "[drop anchor] %s: 单次更新 %.1fcm > %.1fcm，拒绝更新，避免后续视觉带偏",
        reason, update_step, drop_anchor_max_update_step_cm_);
      return;
    }
  }

  drop_anchor_x_cm_ = x_cm;
  drop_anchor_y_cm_ = y_cm;
  has_drop_anchor_ = true;
  RCLCPP_INFO(get_logger(),
    "[drop anchor] %s: 视觉确认空柱中心 -> anchor=(%.1f,%.1f)，相对点云修正 %.1fcm",
    reason, drop_anchor_x_cm_, drop_anchor_y_cm_, nominal_delta);
}

bool PillarPickupMissionNode::finalizePlateRatio(double & out_ratio) const
{
  if (static_cast<int>(plate_ratio_samples_.size()) < plate_min_ratio_frames_) {
    return false;
  }
  auto v = plate_ratio_samples_;
  std::sort(v.begin(), v.end());
  out_ratio = v[v.size() / 2];   // 中位数，抗抖动
  return true;
}

void PillarPickupMissionNode::applyArmOffsetToTarget(
  double & x_cm, double & y_cm, double yaw_deg) const
{
  const double c = std::cos(degToRad(yaw_deg));
  const double s = std::sin(degToRad(yaw_deg));
  const double ox = arm_offset_dx_cm_;
  const double oy = arm_offset_dy_cm_;
  x_cm += c * ox - s * oy;
  y_cm += s * ox + c * oy;
}

// ─────────── 排序工具 ───────────

std::vector<std::size_t> PillarPickupMissionNode::greedyOrderIdx(
  const std::vector<std::pair<double, double>> & pts,
  double start_x_cm, double start_y_cm) const
{
  std::vector<std::size_t> remaining(pts.size());
  for (std::size_t i = 0; i < pts.size(); ++i) { remaining[i] = i; }
  std::vector<std::size_t> ordered;
  ordered.reserve(pts.size());
  double cx = start_x_cm, cy = start_y_cm;
  while (!remaining.empty()) {
    std::size_t best_pos = 0;
    double best_d = std::numeric_limits<double>::max();
    for (std::size_t i = 0; i < remaining.size(); ++i) {
      const auto & p = pts[remaining[i]];
      const double d = std::hypot(p.first - cx, p.second - cy);
      if (d < best_d) { best_d = d; best_pos = i; }
    }
    const std::size_t idx = remaining[best_pos];
    ordered.push_back(idx);
    cx = pts[idx].first;
    cy = pts[idx].second;
    remaining.erase(remaining.begin() + static_cast<std::ptrdiff_t>(best_pos));
  }
  return ordered;
}

std::size_t PillarPickupMissionNode::nearestToLanding() const
{
  std::size_t best = 0;
  double best_d2 = std::numeric_limits<double>::infinity();
  for (std::size_t k = 0; k < detected_pillars_cm_.size(); ++k) {
    const double dx = detected_pillars_cm_[k].first  - landing_x_cm_;
    const double dy = detected_pillars_cm_[k].second - landing_y_cm_;
    const double d2 = dx * dx + dy * dy;
    if (d2 < best_d2) { best_d2 = d2; best = k; }
  }
  return best;
}

// ─────────── LAND 起始 ───────────

void PillarPickupMissionNode::startLanding()
{
  phase_ = MissionPhase::LAND;
  has_land_anchor_ = false;
  publishHeightControlMode(false);
  publishVisualTakeover(false);

  if (land_visual_enable_) {
    // 精准降落：飞到 B 上方 → 视觉对准 B 黑框中心 → 竖直降落（见 enterLandSub/stepLanding）。
    RCLCPP_INFO(get_logger(),
      "进入精准降落，先飞到对角起停区 B (%.0f, %.0f) 上方视觉对准黑框",
      landing_x_cm_, landing_y_cm_);
    enterLandSub(LandSub::APPROACH);
    return;
  }

  // 纯位置降落（旧行为，land_visual_enable=false 时）
  publishVisualTakeover(false);
  republish_enabled_ = true;
  current_idx_ = waypoints_.size();
  waypoints_.push_back(PickupWaypoint{landing_x_cm_, landing_y_cm_, pillar_visit_height_cm_, 0.0, 0.0, "land_approach"});
  waypoints_.push_back(PickupWaypoint{landing_x_cm_, landing_y_cm_, flight_height_cm_,       0.0, 0.0, "land_hover"});
  waypoints_.push_back(PickupWaypoint{landing_x_cm_, landing_y_cm_, land_height_cm_,         0.0, 0.0, "land"});
  publishTarget(waypoints_[current_idx_]);
  RCLCPP_INFO(get_logger(), "进入降落阶段，目标对角起停区 (%.0f, %.0f)", landing_x_cm_, landing_y_cm_);
}

// 降落子状态进入：设目标点 + 视觉接管开关
void PillarPickupMissionNode::enterLandSub(LandSub s)
{
  land_sub_ = s;
  sub_enter_time_ = now();
  fine_hist_.clear();

  // 对准成功后用 anchor（机体实际位姿）；否则回退名义降落点 B。
  const double ax = has_land_anchor_ ? land_anchor_x_cm_ : landing_x_cm_;
  const double ay = has_land_anchor_ ? land_anchor_y_cm_ : landing_y_cm_;

  switch (s) {
    case LandSub::APPROACH: {
      sub_target_ = PickupWaypoint{landing_x_cm_, landing_y_cm_, land_align_height_cm_, 0.0, 0.0, "land_approach"};
      republish_enabled_ = true;
      publishVisualTakeover(false);
      publishTarget(sub_target_);
      break;
    }
    case LandSub::CENTER: {
      sub_target_ = PickupWaypoint{landing_x_cm_, landing_y_cm_, land_align_height_cm_, 0.0, 0.0, "land_center"};
      publishTarget(sub_target_);
      republish_enabled_ = false;          // 视觉接管期间不重发位置目标
      publishVisualTakeover(true);         // 对准 B 黑框中心
      break;
    }
    case LandSub::DESCEND_MID: {
      publishVisualTakeover(false);
      const double mid_z = std::max(land_height_cm_, land_align_height_cm_ - land_recenter_drop_cm_);
      sub_target_ = PickupWaypoint{ax, ay, mid_z, 0.0, 0.0, "land_descend_mid"};
      republish_enabled_ = true;           // 位置保持在 anchor 正上方，下降到中停高度
      publishTarget(sub_target_);
      break;
    }
    case LandSub::RECENTER: {
      const double mid_z = std::max(land_height_cm_, land_align_height_cm_ - land_recenter_drop_cm_);
      sub_target_ = PickupWaypoint{ax, ay, mid_z, 0.0, 0.0, "land_recenter"};
      publishTarget(sub_target_);
      republish_enabled_ = false;          // 视觉接管期间不重发位置目标
      publishVisualTakeover(true);         // 中停高度再对准一次 B 框
      break;
    }
    case LandSub::DESCEND: {
      publishVisualTakeover(false);
      sub_target_ = PickupWaypoint{ax, ay, land_height_cm_, 0.0, 0.0, "land_descend"};
      republish_enabled_ = true;           // 全程位置保持在 anchor 正上方竖直下降
      publishTarget(sub_target_);
      break;
    }
  }
  RCLCPP_INFO(get_logger(), "[降落] 进入子阶段 %d，目标=(%.1f,%.1f,%.1f)",
    static_cast<int>(s), sub_target_.x_cm, sub_target_.y_cm, sub_target_.z_cm);
}

void PillarPickupMissionNode::stepLanding(double x_cm, double y_cm, double z_cm)
{
  switch (land_sub_) {
    case LandSub::APPROACH:
      if (isReached(sub_target_, x_cm, y_cm, z_cm, 0.0)) {
        enterLandSub(LandSub::CENTER);
      }
      break;
    case LandSub::CENTER: {
      const bool aligned = isVisuallyAligned();
      const bool timeout = (now() - sub_enter_time_).seconds() > visual_align1_timeout_sec_;
      if (aligned || timeout) {
        if (aligned) {
          // 视觉对上时机体就在 B 框中心正上方，记其实际位姿为降落 anchor。
          land_anchor_x_cm_ = x_cm;
          land_anchor_y_cm_ = y_cm;
          has_land_anchor_ = true;
          RCLCPP_INFO(get_logger(),
            "[降落] 一次对准 B 框中心 -> anchor=(%.1f,%.1f)，相对名义降落点偏 %.1fcm",
            x_cm, y_cm, std::hypot(x_cm - landing_x_cm_, y_cm - landing_y_cm_));
        } else {
          RCLCPP_WARN(get_logger(),
            "[降落] 一次对准 B 框超时(%.1fs)，回退名义降落点 (%.1f,%.1f)",
            visual_align1_timeout_sec_, landing_x_cm_, landing_y_cm_);
        }
        // 下降一段后再对齐一次：land_recenter_drop>0 且中停高度还在地面之上才分段，否则直接降到底。
        const double mid_z = land_align_height_cm_ - land_recenter_drop_cm_;
        if (land_recenter_drop_cm_ > 0.0 && mid_z > land_height_cm_) {
          enterLandSub(LandSub::DESCEND_MID);
        } else {
          enterLandSub(LandSub::DESCEND);
        }
      }
      break;
    }
    case LandSub::DESCEND_MID:
      if (isReached(sub_target_, x_cm, y_cm, z_cm, 0.0)) {
        enterLandSub(LandSub::RECENTER);   // 到中停高度再视觉对准一次
      }
      break;
    case LandSub::RECENTER: {
      const bool aligned = isVisuallyAligned();
      const bool timeout = (now() - sub_enter_time_).seconds() > visual_align2_timeout_sec_;
      if (aligned || timeout) {
        if (aligned) {
          // 中停高度更近、像素更准，刷新 anchor。
          land_anchor_x_cm_ = x_cm;
          land_anchor_y_cm_ = y_cm;
          has_land_anchor_ = true;
          RCLCPP_INFO(get_logger(),
            "[降落] 二次对准 B 框中心 -> anchor=(%.1f,%.1f)，相对名义降落点偏 %.1fcm",
            x_cm, y_cm, std::hypot(x_cm - landing_x_cm_, y_cm - landing_y_cm_));
        } else {
          RCLCPP_WARN(get_logger(),
            "[降落] 二次对准 B 框超时(%.1fs)，沿用上次 anchor (%.1f,%.1f)",
            visual_align2_timeout_sec_, land_anchor_x_cm_, land_anchor_y_cm_);
        }
        enterLandSub(LandSub::DESCEND);
      }
      break;
    }
    case LandSub::DESCEND:
      if (isReached(sub_target_, x_cm, y_cm, z_cm, 0.0)) {
        publishVisualTakeover(false);
        phase_ = MissionPhase::DONE;
        if (!mission_complete_sent_) {
          std_msgs::msg::Empty e;
          mission_complete_pub_->publish(e);
          mission_complete_sent_ = true;
        }
        publishActiveController(3);
        RCLCPP_INFO(get_logger(), "精准降落完成，任务结束。");
      }
      break;
  }
}

// ─────────── WAIT_PILLARS ───────────

void PillarPickupMissionNode::stepWaitPillars(double x_cm, double y_cm)
{
  const double elapsed = (now() - wait_pillars_start_time_).seconds();
  if (pillars_received_) {
    if (detected_pillars_cm_.empty()) {
      RCLCPP_WARN(get_logger(), "未检出任何柱子，直接降落。");
      startLanding();
      return;
    }
    empty_idx_ = nearestToLanding();
    survey_order_ = greedyOrderIdx(detected_pillars_cm_, x_cm, y_cm);
    survey_results_.assign(detected_pillars_cm_.size(), PillarSurvey{});
    for (std::size_t i = 0; i < detected_pillars_cm_.size(); ++i) {
      survey_results_[i].x_cm = detected_pillars_cm_[i].first;
      survey_results_[i].y_cm = detected_pillars_cm_[i].second;
    }
    RCLCPP_INFO(get_logger(),
      "第一趟规划: %zu 根柱子，空柱(放置目标)=#%zu @ (%.1f,%.1f)",
      survey_order_.size(), empty_idx_,
      survey_results_[empty_idx_].x_cm, survey_results_[empty_idx_].y_cm);
    phase_ = MissionPhase::SURVEY;
    survey_iter_ = 0;
    enterSurveySub(SurveySub::APPROACH);
    return;
  }
  if (elapsed > pillar_wait_timeout_sec_) {
    RCLCPP_WARN(get_logger(), "等待 /detected_pillars 超时 %.1fs，直接降落", elapsed);
    startLanding();
  }
}

// ─────────── SCAN / LAND 航点推进 ───────────

void PillarPickupMissionNode::stepWaypoints(double x_cm, double y_cm, double z_cm, double yaw_deg)
{
  if (current_idx_ >= waypoints_.size()) {
    phase_ = MissionPhase::DONE;
    if (!mission_complete_sent_) {
      std_msgs::msg::Empty e;
      mission_complete_pub_->publish(e);
      mission_complete_sent_ = true;
    }
    publishActiveController(3);
    return;
  }

  const auto & wp = waypoints_[current_idx_];

  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
    "航点 %zu [%s] 目标=(%.1f,%.1f,%.1f) 当前=(%.1f,%.1f,%.1f)",
    current_idx_, wp.tag, wp.x_cm, wp.y_cm, wp.z_cm, x_cm, y_cm, z_cm);

  if (!isReached(wp, x_cm, y_cm, z_cm, yaw_deg)) {
    is_hovering_ = false;
    return;
  }

  if (wp.hover_sec > 0.0) {
    if (!is_hovering_) {
      is_hovering_ = true;
      hover_start_time_ = now();
    }
    if ((now() - hover_start_time_).seconds() < wp.hover_sec) { return; }
  }

  // SCAN 阶段事件：到起飞点开检测，到扫描终点关检测并等聚类
  if (phase_ == MissionPhase::SCAN) {
    if (current_idx_ == 0) {
      publishEnable(true);
    } else if (current_idx_ == scan_end_idx_) {
      publishEnable(false);
      pillars_received_ = false;
      detected_pillars_cm_.clear();
      phase_ = MissionPhase::WAIT_PILLARS;
      wait_pillars_start_time_ = now();
      RCLCPP_INFO(get_logger(), "扫描终点到达，等待 /detected_pillars ...");
      return;
    }
  }

  RCLCPP_INFO(get_logger(), "航点 %zu [%s] 完成。", current_idx_, wp.tag);
  ++current_idx_;
  is_hovering_ = false;
  if (current_idx_ < waypoints_.size()) {
    publishTarget(waypoints_[current_idx_]);
  } else {
    phase_ = MissionPhase::DONE;
    if (!mission_complete_sent_) {
      std_msgs::msg::Empty e;
      mission_complete_pub_->publish(e);
      mission_complete_sent_ = true;
    }
    publishActiveController(3);
    RCLCPP_INFO(get_logger(), "任务全部完成。");
  }
}

// ─────────── 第一趟：SURVEY ───────────

void PillarPickupMissionNode::enterSurveySub(SurveySub s)
{
  survey_sub_ = s;
  sub_enter_time_ = now();
  fine_hist_.clear();

  const std::size_t pi = survey_order_[survey_iter_];
  const double px = survey_results_[pi].x_cm;
  const double py = survey_results_[pi].y_cm;

  switch (s) {
    case SurveySub::APPROACH: {
      sub_target_ = PickupWaypoint{px, py, pillar_visit_height_cm_, 0.0, 0.0, "survey_approach"};
      republish_enabled_ = true;
      publishVisualTakeover(false);
      publishTarget(sub_target_);
      break;
    }
    case SurveySub::CENTER: {
      sub_target_ = PickupWaypoint{px, py, pillar_visit_height_cm_, 0.0, 0.0, "survey_center"};
      publishTarget(sub_target_);
      plate_ratio_samples_.clear();
      survey_collecting_ratio_ = true;   // 第一趟只采占比，不测高
      survey_signal_active_ = false;     // 本柱声光尚未触发
      republish_enabled_ = false;        // 视觉接管期间不重发位置目标
      publishVisualTakeover(true);
      break;
    }
  }
  RCLCPP_INFO(get_logger(), "[survey 柱 %zu/%zu] 进入子阶段 %d",
    survey_iter_ + 1, survey_order_.size(), static_cast<int>(s));
}

void PillarPickupMissionNode::stepSurvey(double x_cm, double y_cm, double z_cm)
{
  const std::size_t pi = survey_order_[survey_iter_];

  switch (survey_sub_) {
    case SurveySub::APPROACH: {
      if (isReached(sub_target_, x_cm, y_cm, z_cm, 0.0)) {
        enterSurveySub(SurveySub::CENTER);
      }
      break;
    }
    case SurveySub::CENTER: {
      const double elapsed = (now() - sub_enter_time_).seconds();
      const bool aligned = isVisuallyAligned();
      const bool ratio_ok =
        static_cast<int>(plate_ratio_samples_.size()) >= plate_min_ratio_frames_;

      // 找到铁片：对准 + 占比帧够 → 开声光，起算滞空（题目"找到滞空3s+声光提示"）
      if (aligned && ratio_ok && !survey_signal_active_) {
        survey_signal_active_ = true;
        survey_signal_start_ = now();
        publishBuzzerLed(true);
        RCLCPP_INFO(get_logger(), "[survey 柱 %zu] 找到铁片 → 开声光，滞空 %.1fs",
          pi, survey_signal_hold_sec_);
      }

      // 退出条件：找到铁片且声光滞空满 hold；或始终没找到(空柱)到对准超时
      const bool signal_done = survey_signal_active_ &&
        (now() - survey_signal_start_).seconds() >= survey_signal_hold_sec_;
      const bool give_up = !survey_signal_active_ && elapsed > visual_align1_timeout_sec_;

      if (signal_done || give_up) {
        if (survey_signal_active_) {
          publishBuzzerLed(false);   // 滞空满 → 关声光
          survey_signal_active_ = false;
        }
        survey_collecting_ratio_ = false;
        publishVisualTakeover(false);

        double ratio = 0.0;
        if (finalizePlateRatio(ratio)) {
          survey_results_[pi].plate_ratio = ratio;
          survey_results_[pi].has_plate = true;
        } else {
          survey_results_[pi].has_plate = false;   // 没采到圆 = 该柱无铁片
        }

        RCLCPP_INFO(get_logger(),
          "[survey 柱 %zu] 铁片占比=%.3f(%s) 用时%.1fs → 下一柱",
          pi,
          survey_results_[pi].plate_ratio, survey_results_[pi].has_plate ? "有片" : "空柱",
          elapsed);

        ++survey_iter_;
        if (survey_iter_ >= survey_order_.size()) { finishSurvey(); }
        else { enterSurveySub(SurveySub::APPROACH); }
      }
      break;
    }
  }
}

void PillarPickupMissionNode::finishSurvey()
{
  // 空柱(放置目标)位置；放置高度不再靠柱高，下降时用 /laser_array/min_range 距柱顶/叠面距离。
  empty_pillar_x_cm_ = survey_results_[empty_idx_].x_cm;
  empty_pillar_y_cm_ = survey_results_[empty_idx_].y_cm;
  has_drop_anchor_ = false;
  drop_anchor_x_cm_ = empty_pillar_x_cm_;
  drop_anchor_y_cm_ = empty_pillar_y_cm_;

  planPickupOrder();

  RCLCPP_INFO(get_logger(), "第一趟完成。待抓铁片 %zu 片（大→小顺序）:", pickup_order_.size());
  for (std::size_t k = 0; k < pickup_order_.size(); ++k) {
    const auto & s = survey_results_[pickup_order_[k]];
    RCLCPP_INFO(get_logger(), "  #%zu 柱%zu @ (%.1f,%.1f) 占比=%.3f",
      k + 1, pickup_order_[k], s.x_cm, s.y_cm, s.plate_ratio);
  }

  if (traverse_only_mode_ || pickup_order_.empty()) {
    if (pickup_order_.empty()) {
      RCLCPP_WARN(get_logger(), "无可抓铁片，直接降落。");
    } else {
      RCLCPP_INFO(get_logger(),
        "只跑第一趟模式（占比已完成）：跳过抓取，直接降落。");
    }
    startLanding();
    return;
  }

  stack_count_ = 0;
  phase_ = MissionPhase::PICKUP;
  pickup_iter_ = 0;
  enterPickupSub(PickupSub::APPROACH);
}

void PillarPickupMissionNode::planPickupOrder()
{
  pickup_order_.clear();
  for (std::size_t i = 0; i < survey_results_.size(); ++i) {
    if (i == empty_idx_) { continue; }            // 空柱是放置目标，不抓
    if (!survey_results_[i].has_plate) { continue; }
    pickup_order_.push_back(i);
  }
  // 占比大的先抓 → 先放 → 垫在底层（大下小上）
  std::sort(pickup_order_.begin(), pickup_order_.end(),
    [this](std::size_t a, std::size_t b) {
      return survey_results_[a].plate_ratio > survey_results_[b].plate_ratio;
    });
}

// ─────────── 第二趟：PICKUP ───────────

void PillarPickupMissionNode::startDropDescend()
{
  // 放置：全程视觉接管。先切柱顶/叠面距离下降到 drop_align_height_cm 精对，
  // 对准连续满足后再下降到 drop_release_clearance 投递。
  descend_is_drop_ = true;
  enterPickupSub(PickupSub::DESCEND_MID);
}

void PillarPickupMissionNode::enterPickupSub(PickupSub s)
{
  pickup_sub_ = s;
  sub_enter_time_ = now();
  fine_hist_.clear();

  const std::size_t si = pickup_order_[pickup_iter_];
  const double px = survey_results_[si].x_cm;
  const double py = survey_results_[si].y_cm;

  switch (s) {
    case PickupSub::APPROACH: {
      sub_target_ = PickupWaypoint{px, py, pillar_visit_height_cm_, 0.0, 0.0, "approach"};
      republish_enabled_ = true;
      publishHeightControlMode(false);
      publishVisualTakeover(false);
      publishTarget(sub_target_);
      pickup_attempts_ = 0;
      break;
    }
    case PickupSub::CENTER: {
      sub_target_ = PickupWaypoint{px, py, pillar_visit_height_cm_, 0.0, 0.0, "center"};
      publishTarget(sub_target_);
      republish_enabled_ = false;
      publishVisualTakeover(true);
      break;
    }
    case PickupSub::DESCEND_MID: {
      publishHeightControlMode(true);
      publishVisualTakeover(true);
      double tx = px, ty = py;
      if (descend_is_drop_) {
        getDropTargetXY(tx, ty);
      }
      applyArmOffsetToTarget(tx, ty, 0.0);
      sub_target_ = PickupWaypoint{
        tx, ty,
        descend_is_drop_ ? drop_align_height_cm_ : grab_align_height_cm_,
        0.0, 0.0,
        descend_is_drop_ ? "drop_descend_to_38cm" : "descend_to_38cm"};
      republish_enabled_ = true;
      publishTarget(sub_target_);
      break;
    }
    case PickupSub::RECENTER_MID: {
      publishHeightControlMode(true);
      // 在距柱顶/叠面预对准高度处精确对准，z 保持当前柱顶距离目标
      double tx = px, ty = py;
      if (descend_is_drop_) {
        getDropTargetXY(tx, ty);
      }
      sub_target_ = PickupWaypoint{
        tx, ty,
        descend_is_drop_ ? drop_align_height_cm_ : grab_align_height_cm_,
        0.0, 0.0,
        descend_is_drop_ ? "drop_recenter_at_38cm" : "recenter_at_38cm"};
      publishTarget(sub_target_);
      republish_enabled_ = false;
      publishVisualTakeover(true);
      break;
    }
    case PickupSub::DESCEND_FINAL: {
      if (!descend_is_drop_) {
        if (!has_area_height_) {
          RCLCPP_WARN(get_logger(),
            "[铁片 %zu/%zu] 抓取最终下降开始时还没有 /laser_array/ground_height，使用 0cm 作为保护目标",
            pickup_iter_ + 1, pickup_order_.size());
        }
        grab_ground_start_height_cm_ = has_area_height_ ? area_height_cm_ : 0.0;
        grab_ground_target_height_cm_ =
          std::max(0.0, grab_ground_start_height_cm_ - grab_descend_delta_cm_);
        grab_final_start_time_ = now();
        publishHeightControlMode(false);
        publishVisualTakeover(true);
        // 预对准通过后，最终下降途中提前吸磁并伸臂。
        // 机械臂伸出会影响 min_range，所以抓取最终下降改用地面高度相对下降。
        publishMagnet(true);
        publishServo(true);
        RCLCPP_INFO(get_logger(),
          "[铁片 %zu/%zu] 抓取最终下降: ground_start=%.1fcm, delta=%.1fcm, target=%.1fcm",
          pickup_iter_ + 1, pickup_order_.size(),
          grab_ground_start_height_cm_, grab_descend_delta_cm_, grab_ground_target_height_cm_);
      }
      // 放置末段用距柱顶/叠面距离；抓取末段用地面高度相对下降。
      const double z_final = descend_is_drop_
        ? drop_release_clearance_cm_
        : grab_ground_target_height_cm_;
      double tx = px, ty = py;
      if (descend_is_drop_) {
        publishHeightControlMode(true);
        publishVisualTakeover(true);
        getDropTargetXY(tx, ty);
      }
      applyArmOffsetToTarget(tx, ty, 0.0);
      if (descend_is_drop_) {
        ty += drop_final_dy_cm_;
        tx += drop_final_dx_cm_;   // 放置专用 x 偏置（map +x=画面正上方）；抓取不加
      }
      sub_target_ = PickupWaypoint{tx, ty, z_final, 0.0, 0.0,
        descend_is_drop_ ? "drop_descend_final" : "descend_final"};
      republish_enabled_ = true;
      publishTarget(sub_target_);
      break;
    }
    case PickupSub::HOVER_GRAB: {
      sub_hover_start_ = now();
      republish_enabled_ = false;
      publishHeightControlMode(false);
      publishVisualTakeover(true);
      RCLCPP_INFO(get_logger(),
        "[铁片 %zu/%zu] HOVER_GRAB: 已到地面高度目标 %.1fcm，停留 %.1fs",
        pickup_iter_ + 1, pickup_order_.size(), grab_ground_target_height_cm_, grab_hold_sec_);
      break;
    }
    case PickupSub::CLIMB_BACK: {
      publishHeightControlMode(true);
      publishVisualTakeover(false);
      sub_target_ = PickupWaypoint{px, py, grab_check_height_cm_, 0.0, 0.0, "climb_to_check_height"};
      republish_enabled_ = true;
      publishTarget(sub_target_);
      break;
    }
    case PickupSub::OBSERVE_GRAB: {
      publishVisualTakeover(false);
      pickup_observed_plate_ = false;
      pickup_observed_plate_frames_ = 0;
      republish_enabled_ = true;
      RCLCPP_INFO(get_logger(),
        "[铁片 %zu/%zu] OBSERVE_GRAB: 观察 %.1fs (尝试 %d/%d)",
        pickup_iter_ + 1, pickup_order_.size(),
        pickup_check_observe_sec_, pickup_attempts_, pickup_max_attempts_);
      break;
    }
    case PickupSub::GOTO_DROP: {
      publishHeightControlMode(false);
      publishVisualTakeover(false);
      double tx = 0.0, ty = 0.0;
      getDropTargetXY(tx, ty);
      sub_target_ = PickupWaypoint{
        tx, ty,
        pillar_visit_height_cm_, 0.0, 0.0, "goto_drop"};
      republish_enabled_ = true;
      publishTarget(sub_target_);
      break;
    }
    case PickupSub::CENTER_DROP: {
      double tx = 0.0, ty = 0.0;
      getDropTargetXY(tx, ty);
      sub_target_ = PickupWaypoint{
        tx, ty,
        pillar_visit_height_cm_, 0.0, 0.0, "center_drop"};
      publishTarget(sub_target_);
      republish_enabled_ = false;
      publishVisualTakeover(true);       // 对准空柱黑色边框中心
      break;
    }
    case PickupSub::HOVER_DROP: {
      sub_hover_start_ = now();
      drop_released_ = false;
      republish_enabled_ = false;
      RCLCPP_INFO(get_logger(),
        "[pickup %zu/%zu] HOVER_DROP: reached %.1fcm, servo down now, magnet off after %.1fs, retract after %.1fs (stack=%d)",
        pickup_iter_ + 1, pickup_order_.size(), drop_release_clearance_cm_,
        drop_magnet_delay_sec_, std::max(arm_extend_sec_, drop_magnet_delay_sec_), stack_count_);
      /*
      RCLCPP_INFO(get_logger(),
        "[铁片 %zu/%zu] HOVER_DROP: 叠面上方%.1fcm → 伸臂 %.1fs → 松磁 → 悬停 %.1fs（已叠 %d 层）",
        pickup_iter_ + 1, pickup_order_.size(), drop_release_clearance_cm_,
        arm_extend_sec_, stack_count_);
      */
      publishServo(true);
      break;
    }
    case PickupSub::CLIMB_AFTER_DROP: {
      double tx = 0.0, ty = 0.0;
      getDropTargetXY(tx, ty);
      sub_target_ = PickupWaypoint{
        tx, ty,
        pillar_visit_height_cm_, 0.0, 0.0, "climb_after_drop"};
      republish_enabled_ = true;
      publishTarget(sub_target_);
      break;
    }
  }
  RCLCPP_INFO(get_logger(), "[铁片 %zu/%zu] 进入子阶段 %d",
    pickup_iter_ + 1, pickup_order_.size(), static_cast<int>(s));
}

void PillarPickupMissionNode::stepPickup(double x_cm, double y_cm, double z_cm)
{
  // fine_data 看门狗：仅日志提示，不强行关 takeover（让 PID 内部保持）
  if (visual_takeover_active_ && has_fine_) {
    const double age = (now() - last_fine_time_).seconds();
    if (age > visual_stale_sec_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
        "fine_data stale %.2fs, 视觉暂不可用", age);
    }
  }

  switch (pickup_sub_) {
    case PickupSub::APPROACH:
      if (isReached(sub_target_, x_cm, y_cm, z_cm, 0.0)) {
        enterPickupSub(PickupSub::CENTER);
      }
      break;
    case PickupSub::CENTER:
      enterPickupSub(PickupSub::DESCEND_MID);
      break;
    case PickupSub::DESCEND_MID:
      if (isReached(sub_target_, x_cm, y_cm, z_cm, 0.0)) {
        enterPickupSub(PickupSub::RECENTER_MID);   // 到预对准高度先精确对准
      }
      break;
    case PickupSub::RECENTER_MID: {
      // 预对准高度处要求视觉误差连续 visual_align_required_hits 帧小于容错，再进入最终下降。
      if (isVisuallyAligned()) {
        if (descend_is_drop_) {
          updateDropAnchorFromVision(x_cm, y_cm, "drop_align_38cm");
        }
        enterPickupSub(PickupSub::DESCEND_FINAL);
      }
      break;
    }
    case PickupSub::DESCEND_FINAL:
      if (!descend_is_drop_ && grab_final_timeout_sec_ > 0.0 &&
          (now() - grab_final_start_time_).seconds() >= grab_final_timeout_sec_) {
        RCLCPP_WARN(get_logger(),
          "[pickup %zu/%zu] grab final timeout %.1fs, retract servo and climb to check",
          pickup_iter_ + 1, pickup_order_.size(), grab_final_timeout_sec_);
        publishServo(false);
        ++pickup_attempts_;
        enterPickupSub(PickupSub::CLIMB_BACK);
        break;
      }
      if (isReached(sub_target_, x_cm, y_cm, z_cm, 0.0)) {
        if (descend_is_drop_) {
          enterPickupSub(PickupSub::HOVER_DROP);
        } else {
          enterPickupSub(PickupSub::HOVER_GRAB);
        }
      }
      break;
    case PickupSub::HOVER_GRAB:
      if ((grab_final_timeout_sec_ > 0.0 &&
           (now() - grab_final_start_time_).seconds() >= grab_final_timeout_sec_) ||
          (now() - sub_hover_start_).seconds() >= grab_hold_sec_) {
        if (grab_final_timeout_sec_ > 0.0 &&
            (now() - grab_final_start_time_).seconds() >= grab_final_timeout_sec_) {
          RCLCPP_WARN(get_logger(),
            "[pickup %zu/%zu] grab final timeout %.1fs during hold, retract servo and climb to check",
            pickup_iter_ + 1, pickup_order_.size(), grab_final_timeout_sec_);
        }
        publishServo(false);   // 电磁铁保持吸住，收臂后只爬到观察高度
        ++pickup_attempts_;
        enterPickupSub(PickupSub::CLIMB_BACK);
      }
      break;
    case PickupSub::CLIMB_BACK:
      if (isReached(sub_target_, x_cm, y_cm, z_cm, 0.0)) {
        enterPickupSub(PickupSub::OBSERVE_GRAB);
      }
      break;
    case PickupSub::OBSERVE_GRAB: {
      const double elapsed = (now() - sub_enter_time_).seconds();
      if (elapsed < pickup_check_observe_sec_) { break; }

      if (!pickup_observed_plate_) {
        RCLCPP_INFO(get_logger(),
          "[铁片 %zu/%zu] 抓取成功（尝试 %d/%d），发布 /pickup_done",
          pickup_iter_ + 1, pickup_order_.size(), pickup_attempts_, pickup_max_attempts_);
        std_msgs::msg::Empty e;
        pickup_done_pub_->publish(e);
        enterPickupSub(PickupSub::GOTO_DROP);
        break;
      }

      // 抓取失败：电磁铁保持通电，决定重试还是放弃
      if (pickup_attempts_ < pickup_max_attempts_) {
        RCLCPP_WARN(get_logger(),
          "[铁片 %zu/%zu] 抓取失败（尝试 %d/%d），重新下降再试",
          pickup_iter_ + 1, pickup_order_.size(), pickup_attempts_, pickup_max_attempts_);
        enterPickupSub(PickupSub::CENTER);
      } else {
        // 本片 3 次都失败：松磁收臂，发 /pickup_failed，然后【跳下一片】，
        // 不直接降落。所有片各自试满 pickup_max_attempts 次仍失败，才进降落。
        RCLCPP_ERROR(get_logger(),
          "[铁片 %zu/%zu] 本片 %d 次抓取均失败，发布 /pickup_failed，尝试下一片",
          pickup_iter_ + 1, pickup_order_.size(), pickup_attempts_);
        publishMagnet(false);
        publishServo(false);
        std_msgs::msg::Empty e;
        pickup_failed_pub_->publish(e);
        ++pickup_iter_;
        if (pickup_iter_ >= pickup_order_.size()) {
          RCLCPP_ERROR(get_logger(),
            "全部 %zu 片均抓取失败，进入降落。", pickup_order_.size());
          mission_complete_sent_ = true;   // 失败收尾，不发完成信号
          startLanding();
        } else {
          pickup_attempts_ = 0;            // 下一片重置尝试计数
          enterPickupSub(PickupSub::APPROACH);
        }
      }
      break;
    }
    case PickupSub::GOTO_DROP:
      if (isReached(sub_target_, x_cm, y_cm, z_cm, 0.0)) {
        enterPickupSub(PickupSub::CENTER_DROP);
      }
      break;
    case PickupSub::CENTER_DROP: {
      startDropDescend();
      break;
    }
    case PickupSub::HOVER_DROP: {
      const double elapsed = (now() - sub_hover_start_).seconds();
      if (!drop_released_ && elapsed >= drop_magnet_delay_sec_) {
        publishMagnet(false);
        drop_released_ = true;
        RCLCPP_INFO(get_logger(),
          "[铁片 %zu/%zu] 放置伸臂到位，松磁释放", pickup_iter_ + 1, pickup_order_.size());
      }
      if (elapsed >= std::max(arm_extend_sec_, drop_magnet_delay_sec_)) {
        publishServo(false);
        ++stack_count_;   // 这一片已叠上
        descend_is_drop_ = false;
        enterPickupSub(PickupSub::CLIMB_AFTER_DROP);
      }
      break;
    }
    case PickupSub::CLIMB_AFTER_DROP:
      if (isReached(sub_target_, x_cm, y_cm, z_cm, 0.0)) {
        ++pickup_iter_;
        if (pickup_iter_ >= pickup_order_.size()) {
          RCLCPP_INFO(get_logger(), "所有铁片已搬运叠放完成，进入降落。");
          startLanding();
        } else {
          pickup_attempts_ = 0;            // 下一片重置尝试计数
          enterPickupSub(PickupSub::APPROACH);
        }
      }
      break;
  }
}

// ─────────── 主循环 ───────────

void PillarPickupMissionNode::monitorTimerCallback()
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (phase_ == MissionPhase::DONE) {
    publishActiveController(3);
    return;
  }

  double x_cm = 0.0, y_cm = 0.0, yaw_deg = 0.0;
  if (!getCurrentPose(x_cm, y_cm, yaw_deg)) { return; }
  const bool pickup_uses_pillar_height =
    phase_ == MissionPhase::PICKUP &&
    (pickup_sub_ == PickupSub::DESCEND_MID ||
     pickup_sub_ == PickupSub::RECENTER_MID ||
     (pickup_sub_ == PickupSub::DESCEND_FINAL && descend_is_drop_) ||
     pickup_sub_ == PickupSub::CLIMB_BACK ||
     pickup_sub_ == PickupSub::HOVER_DROP);
  const double z_cm = (pickup_uses_pillar_height && has_pillar_top_distance_)
    ? pillar_top_distance_cm_
    : (has_area_height_ ? area_height_cm_ : 0.0);

  switch (phase_) {
    case MissionPhase::WAIT_PILLARS:
      stepWaitPillars(x_cm, y_cm);
      break;
    case MissionPhase::SURVEY:
      stepSurvey(x_cm, y_cm, z_cm);
      break;
    case MissionPhase::PICKUP:
      stepPickup(x_cm, y_cm, z_cm);
      break;
    case MissionPhase::SCAN:
      stepWaypoints(x_cm, y_cm, z_cm, yaw_deg);
      break;
    case MissionPhase::LAND:
      if (land_visual_enable_) {
        stepLanding(x_cm, y_cm, z_cm);          // 视觉对准 B 框精准降落
      } else {
        stepWaypoints(x_cm, y_cm, z_cm, yaw_deg);  // 纯位置降落（旧行为）
      }
      break;
    default:
      break;
  }
}

// ─────────── 工具 ───────────

double PillarPickupMissionNode::radToDeg(double v)
{
  return v * 180.0 / M_PI;
}

double PillarPickupMissionNode::normalizeAngleDeg(double angle_deg) const
{
  const double n = angles::normalize_angle(angles::from_degrees(angle_deg));
  return angles::to_degrees(n);
}

}  // namespace activity_control_pkg

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<activity_control_pkg::PillarPickupMissionNode>());
  rclcpp::shutdown();
  return 0;
}
