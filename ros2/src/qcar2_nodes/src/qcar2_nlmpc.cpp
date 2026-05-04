#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <numeric>
#include <stdexcept>
#include <string>
#include <vector>

#ifdef QCAR2_USE_ACADO
extern "C"
{
#include "acado_common.h"
#include "acado_auxiliary_functions.h"
}
#endif

class QCar2NLMPCNode : public rclcpp::Node
{
public:
  QCar2NLMPCNode() : Node("qcar2_nlmpc")
  {
    declare_parameters();
    load_parameters();

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_topic_, rclcpp::QoS(10));
    reference_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      reference_topic_, rclcpp::QoS(1).transient_local().reliable(),
      std::bind(&QCar2NLMPCNode::reference_callback, this, std::placeholders::_1));

#ifdef QCAR2_USE_ACADO
    acado_initializeSolver();
    initialize_acado_memory();
    RCLCPP_INFO(this->get_logger(), "ACADO solver initialized. ACADO_N=%d", ACADO_N);
#else
    RCLCPP_ERROR(
      this->get_logger(),
      "qcar2_nlmpc was built without QCAR2_USE_ACADO. Generate acado_qcar2_nlmpc first and rebuild.");
#endif

    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(control_period_),
      std::bind(&QCar2NLMPCNode::control_timer_callback, this));

    last_control_time_ = this->now();
    publish_stop_command();
  }

private:
  struct ReferencePoint
  {
    double t{0.0};
    double x{0.0};
    double y{0.0};
    double theta{0.0};
    double varphi{0.0};
    double v{0.0};
    double omega_s{0.0};
    int segment{0};
  };

  struct State
  {
    double x{0.0};
    double y{0.0};
    double theta{0.0};
    double varphi{0.0};
  };

  enum class ControllerState
  {
    WAITING_FOR_REFERENCE,
    TRACKING,
    GOAL_REACHED,
    ERROR_STOP
  };

  static constexpr double kEps = 1e-9;

  void declare_parameters()
  {
    this->declare_parameter<std::string>("frame_id", "map");
    this->declare_parameter<std::string>("base_frame", "base_link");
    this->declare_parameter<std::string>("reference_topic", "/qcar2/reference_trajectory");
    this->declare_parameter<std::string>("cmd_topic", "/qcar2_cmd");

    this->declare_parameter<double>("control_period", 0.10);
    this->declare_parameter<int>("Npred", 12);
    this->declare_parameter<double>("L", 0.25725);

    this->declare_parameter<double>("v_min", -0.20);
    this->declare_parameter<double>("v_max", 0.45);
    this->declare_parameter<double>("omega_s_max", 2.0);
    this->declare_parameter<double>("varphi_max", 0.5236);

    this->declare_parameter<std::vector<double>>("Q", std::vector<double>{80.0, 80.0, 8.0, 2.0});
    this->declare_parameter<std::vector<double>>("R", std::vector<double>{5.0, 1.0});
    this->declare_parameter<std::vector<double>>("P", std::vector<double>{200.0, 200.0, 20.0, 5.0});

    this->declare_parameter<double>("goal_xy_tolerance", 0.08);
    this->declare_parameter<double>("goal_yaw_tolerance", 0.20);
    this->declare_parameter<int>("goal_index_margin", 10);
    this->declare_parameter<double>("max_tracking_error", 0.80);
    this->declare_parameter<double>("reference_timeout", 2.0);
    this->declare_parameter<double>("tf_timeout", 0.20);

    this->declare_parameter<bool>("return_steering_to_zero_on_stop", true);
    this->declare_parameter<bool>("print_solver_timing", true);
  }

  void load_parameters()
  {
    frame_id_ = this->get_parameter("frame_id").as_string();
    base_frame_ = this->get_parameter("base_frame").as_string();
    reference_topic_ = this->get_parameter("reference_topic").as_string();
    cmd_topic_ = this->get_parameter("cmd_topic").as_string();

    control_period_ = this->get_parameter("control_period").as_double();
    n_pred_param_ = static_cast<int>(this->get_parameter("Npred").as_int());
    L_ = this->get_parameter("L").as_double();

    v_min_ = this->get_parameter("v_min").as_double();
    v_max_ = this->get_parameter("v_max").as_double();
    omega_s_max_ = this->get_parameter("omega_s_max").as_double();
    varphi_max_ = this->get_parameter("varphi_max").as_double();

    Q_ = this->get_parameter("Q").as_double_array();
    R_ = this->get_parameter("R").as_double_array();
    P_ = this->get_parameter("P").as_double_array();

    goal_xy_tolerance_ = this->get_parameter("goal_xy_tolerance").as_double();
    goal_yaw_tolerance_ = this->get_parameter("goal_yaw_tolerance").as_double();
    goal_index_margin_ = static_cast<int>(this->get_parameter("goal_index_margin").as_int());
    max_tracking_error_ = this->get_parameter("max_tracking_error").as_double();
    reference_timeout_ = this->get_parameter("reference_timeout").as_double();
    tf_timeout_ = this->get_parameter("tf_timeout").as_double();

    return_steering_to_zero_on_stop_ = this->get_parameter("return_steering_to_zero_on_stop").as_bool();
    print_solver_timing_ = this->get_parameter("print_solver_timing").as_bool();

    if (control_period_ <= 0.0 || L_ <= 0.0) {
      throw std::runtime_error("control_period and L must be > 0.");
    }
    if (v_min_ >= v_max_) {
      throw std::runtime_error("v_min must be smaller than v_max.");
    }
    if (omega_s_max_ <= 0.0 || varphi_max_ <= 0.0) {
      throw std::runtime_error("omega_s_max and varphi_max must be > 0.");
    }
    if (Q_.size() != 4 || R_.size() != 2 || P_.size() != 4) {
      throw std::runtime_error("Q must have 4 values, R must have 2 values, P must have 4 values.");
    }
    if (goal_xy_tolerance_ <= 0.0 || goal_yaw_tolerance_ <= 0.0 || max_tracking_error_ <= 0.0) {
      throw std::runtime_error("Goal tolerances and max_tracking_error must be > 0.");
    }

#ifdef QCAR2_USE_ACADO
    if (n_pred_param_ != ACADO_N) {
      RCLCPP_WARN(
        this->get_logger(),
        "YAML Npred=%d but generated ACADO_N=%d. The node will use ACADO_N.",
        n_pred_param_, ACADO_N);
    }
#endif
  }

  static double normalize_angle(double angle)
  {
    return std::atan2(std::sin(angle), std::cos(angle));
  }

  static double yaw_from_quaternion(double qx, double qy, double qz, double qw)
  {
    const double siny_cosp = 2.0 * (qw * qz + qx * qy);
    const double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
    return std::atan2(siny_cosp, cosy_cosp);
  }

  static double interpolate_angle(double a, double b, double ratio)
  {
    return a + ratio * normalize_angle(b - a);
  }

  static double clamp(double value, double low, double high)
  {
    return std::min(std::max(value, low), high);
  }

  void reference_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    constexpr std::size_t fields = 8U;
    if (msg->data.size() < 2U * fields || msg->data.size() % fields != 0U) {
      RCLCPP_WARN(
        this->get_logger(),
        "Invalid reference trajectory message. Expected flattened points [t,x,y,theta,varphi,v,omega_s,segment].");
      return;
    }

    std::vector<ReferencePoint> new_ref;
    new_ref.reserve(msg->data.size() / fields);
    for (std::size_t i = 0; i < msg->data.size(); i += fields) {
      ReferencePoint p;
      p.t = msg->data[i + 0U];
      p.x = msg->data[i + 1U];
      p.y = msg->data[i + 2U];
      p.theta = msg->data[i + 3U];
      p.varphi = msg->data[i + 4U];
      p.v = msg->data[i + 5U];
      p.omega_s = msg->data[i + 6U];
      p.segment = static_cast<int>(std::llround(msg->data[i + 7U]));
      new_ref.push_back(p);
    }

    const uint64_t signature = compute_reference_signature(new_ref);
    if (state_ == ControllerState::GOAL_REACHED && signature == completed_reference_signature_) {
      return;
    }
    if (signature == active_reference_signature_ && !reference_.empty()) {
      last_reference_time_ = this->now();
      return;
    }

    reference_ = std::move(new_ref);
    active_reference_signature_ = signature;
    completed_reference_signature_ = 0U;
    closest_index_ = 0U;
    last_reference_time_ = this->now();
    state_ = ControllerState::TRACKING;
    acado_is_initialized_ = false;

    RCLCPP_INFO(
      this->get_logger(),
      "Received new reference trajectory with %zu points. Tracking started.",
      reference_.size());
  }

  uint64_t compute_reference_signature(const std::vector<ReferencePoint> & ref) const
  {
    if (ref.empty()) {
      return 0U;
    }

    auto quant = [](double value) -> int64_t {
      return static_cast<int64_t>(std::llround(value * 10000.0));
    };

    uint64_t h = 1469598103934665603ULL;
    auto mix = [&h](int64_t v) {
      uint64_t x = static_cast<uint64_t>(v);
      h ^= x;
      h *= 1099511628211ULL;
    };

    mix(static_cast<int64_t>(ref.size()));
    const ReferencePoint & a = ref.front();
    const ReferencePoint & b = ref.back();
    mix(quant(a.x)); mix(quant(a.y)); mix(quant(a.theta));
    mix(quant(b.x)); mix(quant(b.y)); mix(quant(b.theta));
    mix(quant(ref.back().t));
    return h;
  }

  bool get_current_state(State & state)
  {
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_->lookupTransform(
        frame_id_, base_frame_, tf2::TimePointZero,
        std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(tf_timeout_)));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "Cannot lookup TF %s -> %s: %s", frame_id_.c_str(), base_frame_.c_str(), ex.what());
      return false;
    }

    state.x = tf.transform.translation.x;
    state.y = tf.transform.translation.y;
    state.theta = yaw_from_quaternion(
      tf.transform.rotation.x, tf.transform.rotation.y,
      tf.transform.rotation.z, tf.transform.rotation.w);
    state.varphi = varphi_estimate_;
    return true;
  }

  void update_varphi_estimate()
  {
    const rclcpp::Time now = this->now();
    double dt = (now - last_control_time_).seconds();
    if (dt <= 0.0 || dt > 1.0) {
      dt = control_period_;
    }
    last_control_time_ = now;

    varphi_estimate_ = clamp(
      varphi_estimate_ + dt * last_omega_s_cmd_,
      -varphi_max_, varphi_max_);
  }

  std::size_t find_closest_reference_index(const State & state)
  {
    if (reference_.empty()) {
      return 0U;
    }

    const std::size_t start = (closest_index_ > 20U) ? closest_index_ - 20U : 0U;
    const std::size_t end = reference_.size();

    double best_dist2 = std::numeric_limits<double>::infinity();
    std::size_t best = closest_index_;
    for (std::size_t i = start; i < end; ++i) {
      const double dx = state.x - reference_[i].x;
      const double dy = state.y - reference_[i].y;
      const double d2 = dx * dx + dy * dy;
      if (d2 < best_dist2) {
        best_dist2 = d2;
        best = i;
      }
    }

    closest_index_ = std::max(closest_index_, best);
    return closest_index_;
  }

  ReferencePoint interpolate_reference_at_time(double t) const
  {
    if (reference_.empty()) {
      return {};
    }
    if (t <= reference_.front().t) {
      return reference_.front();
    }
    if (t >= reference_.back().t) {
      return reference_.back();
    }

    auto it = std::lower_bound(
      reference_.begin(), reference_.end(), t,
      [](const ReferencePoint & p, double value) {return p.t < value;});

    if (it == reference_.begin()) {
      return *it;
    }

    const ReferencePoint & b = *it;
    const ReferencePoint & a = *(it - 1);
    const double dt = std::max(b.t - a.t, kEps);
    const double r = clamp((t - a.t) / dt, 0.0, 1.0);

    ReferencePoint out;
    out.t = t;
    out.x = a.x + r * (b.x - a.x);
    out.y = a.y + r * (b.y - a.y);
    out.theta = interpolate_angle(a.theta, b.theta, r);
    out.varphi = a.varphi + r * (b.varphi - a.varphi);
    out.v = a.v + r * (b.v - a.v);
    out.omega_s = a.omega_s + r * (b.omega_s - a.omega_s);
    out.segment = a.segment;
    return out;
  }

  bool is_goal_reached(const State & state, const std::size_t closest_idx) const
  {
    if (reference_.empty()) {
      return false;
    }

    const auto & goal = reference_.back();
    const double e_xy = std::hypot(state.x - goal.x, state.y - goal.y);
    const double e_yaw = std::abs(normalize_angle(state.theta - goal.theta));

    const int last_allowed = static_cast<int>(reference_.size()) - 1 - goal_index_margin_;
    const bool near_end = static_cast<int>(closest_idx) >= std::max(0, last_allowed);

    return near_end && e_xy <= goal_xy_tolerance_ && e_yaw <= goal_yaw_tolerance_;
  }

  void publish_command(double v_cmd, double varphi_cmd)
  {
    geometry_msgs::msg::Twist msg;
    msg.linear.x = clamp(v_cmd, v_min_, v_max_);
    msg.angular.z = clamp(varphi_cmd, -varphi_max_, varphi_max_);
    cmd_pub_->publish(msg);
  }

  void publish_stop_command()
  {
    last_omega_s_cmd_ = 0.0;
    if (return_steering_to_zero_on_stop_) {
      varphi_estimate_ = 0.0;
      publish_command(0.0, 0.0);
    } else {
      publish_command(0.0, varphi_estimate_);
    }
  }

  void control_timer_callback()
  {
    update_varphi_estimate();

    if (state_ == ControllerState::WAITING_FOR_REFERENCE || reference_.empty()) {
      publish_stop_command();
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Waiting for a reference trajectory on %s...", reference_topic_.c_str());
      return;
    }

    const double ref_age = (this->now() - last_reference_time_).seconds();
    if (ref_age > reference_timeout_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "Reference trajectory timeout %.2f s. Stopping.", ref_age);
      publish_stop_command();
      return;
    }

    if (state_ == ControllerState::GOAL_REACHED) {
      publish_stop_command();
      return;
    }

    State current;
    if (!get_current_state(current)) {
      publish_stop_command();
      return;
    }

    const std::size_t closest_idx = find_closest_reference_index(current);
    const ReferencePoint closest_ref = reference_[closest_idx];
    const double tracking_error = std::hypot(current.x - closest_ref.x, current.y - closest_ref.y);

    if (tracking_error > max_tracking_error_) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Tracking error %.3f m exceeded limit %.3f m. Emergency stop. Send a new reference/goal to restart.",
        tracking_error, max_tracking_error_);
      state_ = ControllerState::ERROR_STOP;
      publish_stop_command();
      return;
    }

    if (state_ == ControllerState::ERROR_STOP) {
      publish_stop_command();
      return;
    }

    if (is_goal_reached(current, closest_idx)) {
      state_ = ControllerState::GOAL_REACHED;
      completed_reference_signature_ = active_reference_signature_;
      publish_stop_command();
      RCLCPP_INFO(
        this->get_logger(),
        "[QCar2 NLMPC] Goal reached. Controller stopped. Waiting for the next goal/reference...");
      return;
    }

#ifdef QCAR2_USE_ACADO
    std::vector<ReferencePoint> horizon;
    horizon.reserve(static_cast<std::size_t>(ACADO_N + 1));
    const double t0 = reference_[closest_idx].t;
    for (int i = 0; i <= ACADO_N; ++i) {
      horizon.push_back(interpolate_reference_at_time(t0 + static_cast<double>(i) * control_period_));
    }

    const auto solve_start = std::chrono::steady_clock::now();
    double v_cmd = 0.0;
    double omega_s_cmd = 0.0;
    const bool ok = solve_acado(current, horizon, v_cmd, omega_s_cmd);
    const auto solve_end = std::chrono::steady_clock::now();

    if (!ok) {
      RCLCPP_WARN(this->get_logger(), "ACADO failed. Stopping the vehicle.");
      publish_stop_command();
      return;
    }

    v_cmd = clamp(v_cmd, v_min_, v_max_);
    omega_s_cmd = clamp(omega_s_cmd, -omega_s_max_, omega_s_max_);
    last_omega_s_cmd_ = omega_s_cmd;

    const double varphi_cmd = clamp(
      varphi_estimate_ + control_period_ * omega_s_cmd,
      -varphi_max_, varphi_max_);
    publish_command(v_cmd, varphi_cmd);

    if (print_solver_timing_) {
      const double ms = std::chrono::duration<double, std::milli>(solve_end - solve_start).count();
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "closest=%zu e=%.3f v=%.3f omega_s=%.3f varphi_cmd=%.3f solve=%.2f ms",
        closest_idx, tracking_error, v_cmd, omega_s_cmd, varphi_cmd, ms);
    }
#else
    publish_stop_command();
#endif
  }

#ifdef QCAR2_USE_ACADO
  void initialize_acado_memory()
  {
    for (int i = 0; i < ACADO_NX * (ACADO_N + 1); ++i) {
      acadoVariables.x[i] = 0.0;
    }
    for (int i = 0; i < ACADO_NU * ACADO_N; ++i) {
      acadoVariables.u[i] = 0.0;
    }

#ifdef ACADO_NY
    for (int i = 0; i < ACADO_NY * ACADO_N; ++i) {
      acadoVariables.y[i] = 0.0;
    }
#endif
#ifdef ACADO_NYN
    for (int i = 0; i < ACADO_NYN; ++i) {
      acadoVariables.yN[i] = 0.0;
    }
#endif
    set_acado_weights();
  }

  void set_acado_weights()
  {
#ifdef ACADO_NY
    for (int i = 0; i < ACADO_NY * ACADO_NY; ++i) {
      acadoVariables.W[i] = 0.0;
    }
    acadoVariables.W[0 * ACADO_NY + 0] = Q_[0];
    acadoVariables.W[1 * ACADO_NY + 1] = Q_[1];
    acadoVariables.W[2 * ACADO_NY + 2] = Q_[2];
    acadoVariables.W[3 * ACADO_NY + 3] = Q_[3];
    acadoVariables.W[4 * ACADO_NY + 4] = R_[0];
    acadoVariables.W[5 * ACADO_NY + 5] = R_[1];
#endif
#ifdef ACADO_NYN
    for (int i = 0; i < ACADO_NYN * ACADO_NYN; ++i) {
      acadoVariables.WN[i] = 0.0;
    }
    acadoVariables.WN[0 * ACADO_NYN + 0] = P_[0];
    acadoVariables.WN[1 * ACADO_NYN + 1] = P_[1];
    acadoVariables.WN[2 * ACADO_NYN + 2] = P_[2];
    acadoVariables.WN[3 * ACADO_NYN + 3] = P_[3];
#endif
  }

  bool solve_acado(
    const State & current,
    const std::vector<ReferencePoint> & horizon,
    double & v_cmd,
    double & omega_s_cmd)
  {
    if (horizon.size() < static_cast<std::size_t>(ACADO_N + 1)) {
      return false;
    }

    acadoVariables.x0[0] = current.x;
    acadoVariables.x0[1] = current.y;
    acadoVariables.x0[2] = current.theta;
    acadoVariables.x0[3] = current.varphi;

    set_acado_weights();

    if (!acado_is_initialized_) {
      for (int i = 0; i <= ACADO_N; ++i) {
        const auto & r = horizon[static_cast<std::size_t>(i)];
        acadoVariables.x[i * ACADO_NX + 0] = r.x;
        acadoVariables.x[i * ACADO_NX + 1] = r.y;
        acadoVariables.x[i * ACADO_NX + 2] = r.theta;
        acadoVariables.x[i * ACADO_NX + 3] = clamp(r.varphi, -varphi_max_, varphi_max_);
      }
      for (int i = 0; i < ACADO_N; ++i) {
        const auto & r = horizon[static_cast<std::size_t>(i)];
        acadoVariables.u[i * ACADO_NU + 0] = clamp(r.v, v_min_, v_max_);
        acadoVariables.u[i * ACADO_NU + 1] = clamp(r.omega_s, -omega_s_max_, omega_s_max_);
      }
      acado_is_initialized_ = true;
    }

#ifdef ACADO_NY
    for (int i = 0; i < ACADO_N; ++i) {
      const auto & r = horizon[static_cast<std::size_t>(i)];
      acadoVariables.y[i * ACADO_NY + 0] = r.x;
      acadoVariables.y[i * ACADO_NY + 1] = r.y;
      acadoVariables.y[i * ACADO_NY + 2] = r.theta;
      acadoVariables.y[i * ACADO_NY + 3] = clamp(r.varphi, -varphi_max_, varphi_max_);
      acadoVariables.y[i * ACADO_NY + 4] = clamp(r.v, v_min_, v_max_);
      acadoVariables.y[i * ACADO_NY + 5] = clamp(r.omega_s, -omega_s_max_, omega_s_max_);
    }
#endif
#ifdef ACADO_NYN
    const auto & rN = horizon[static_cast<std::size_t>(ACADO_N)];
    acadoVariables.yN[0] = rN.x;
    acadoVariables.yN[1] = rN.y;
    acadoVariables.yN[2] = rN.theta;
    acadoVariables.yN[3] = clamp(rN.varphi, -varphi_max_, varphi_max_);
#endif

    int status = 0;
    status = acado_preparationStep();
    if (status != 0) {
      RCLCPP_WARN(this->get_logger(), "acado_preparationStep returned status %d", status);
    }
    status = acado_feedbackStep();
    if (status != 0) {
      RCLCPP_WARN(this->get_logger(), "acado_feedbackStep returned status %d", status);
      return false;
    }

    v_cmd = acadoVariables.u[0];
    omega_s_cmd = acadoVariables.u[1];
    return std::isfinite(v_cmd) && std::isfinite(omega_s_cmd);
  }
#endif

  std::string frame_id_{"map"};
  std::string base_frame_{"base_link"};
  std::string reference_topic_{"/qcar2/reference_trajectory"};
  std::string cmd_topic_{"/qcar2_cmd"};

  double control_period_{0.10};
  int n_pred_param_{12};
  double L_{0.25725};

  double v_min_{-0.20};
  double v_max_{0.45};
  double omega_s_max_{2.0};
  double varphi_max_{0.5236};

  std::vector<double> Q_{80.0, 80.0, 8.0, 2.0};
  std::vector<double> R_{5.0, 1.0};
  std::vector<double> P_{200.0, 200.0, 20.0, 5.0};

  double goal_xy_tolerance_{0.08};
  double goal_yaw_tolerance_{0.20};
  int goal_index_margin_{10};
  double max_tracking_error_{0.80};
  double reference_timeout_{2.0};
  double tf_timeout_{0.20};
  bool return_steering_to_zero_on_stop_{true};
  bool print_solver_timing_{true};

  ControllerState state_{ControllerState::WAITING_FOR_REFERENCE};
  std::vector<ReferencePoint> reference_;
  std::size_t closest_index_{0U};
  uint64_t active_reference_signature_{0U};
  uint64_t completed_reference_signature_{0U};

  double varphi_estimate_{0.0};
  double last_omega_s_cmd_{0.0};
  bool acado_is_initialized_{false};

  rclcpp::Time last_control_time_;
  rclcpp::Time last_reference_time_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr reference_sub_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    rclcpp::spin(std::make_shared<QCar2NLMPCNode>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("qcar2_nlmpc"), "Exception: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
