#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

class QCar2BezierNode : public rclcpp::Node
{
public:
  QCar2BezierNode() : Node("qcar2_bezier")
  {
    declare_parameters();
    load_parameters();

    path_pub_ = this->create_publisher<nav_msgs::msg::Path>(path_topic_, rclcpp::QoS(10));
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic_, rclcpp::QoS(10));
    reference_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      reference_topic_, rclcpp::QoS(1).transient_local().reliable());

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    if (enable_goal_input_) {
      goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        goal_topic_, rclcpp::QoS(10),
        std::bind(&QCar2BezierNode::goal_callback, this, std::placeholders::_1));
      RCLCPP_INFO(this->get_logger(), "Goal input enabled on topic %s", goal_topic_.c_str());
    }

    compute_trajectory();
    publish_all();

    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(publish_period_),
      std::bind(&QCar2BezierNode::publish_all, this));
  }

private:
  struct Vec2
  {
    double x{0.0};
    double y{0.0};
  };

  struct Waypoint
  {
    double x{0.0};
    double y{0.0};
    double theta{0.0};
    double varphi{0.0};
    double v{0.0};
    double omega_s{0.0};
  };

  struct Sample
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

  static constexpr double kEps = 1e-9;

  void declare_parameters()
  {
    this->declare_parameter<std::string>("frame_id", "map");
    this->declare_parameter<double>("publish_period", 1.0);
    this->declare_parameter<int>("heading_stride", 25);
    this->declare_parameter<double>("heading_scale", 0.18);
    this->declare_parameter<std::string>("path_topic", "/bezier_path");
    this->declare_parameter<std::string>("marker_topic", "/bezier_markers");
    this->declare_parameter<std::string>("reference_topic", "/qcar2/reference_trajectory");
    this->declare_parameter<bool>("show_control_polygon", true);
    this->declare_parameter<bool>("show_waypoints", true);

    this->declare_parameter<double>("L", 0.25725);
    this->declare_parameter<double>("DT", 0.02);
    this->declare_parameter<double>("VARPHI_MAX", 0.5236);
    this->declare_parameter<double>("OMEGA_S_MAX", 2.0);
    this->declare_parameter<double>("default_segment_time", 8.0);

    // Waypoint input mode.
    // "xyv"  mode: each waypoint is [x, y, v]. The node computes theta, varphi, omega_s.
    // "full" mode: each waypoint is [x, y, theta, varphi, v, omega_s].
    this->declare_parameter<std::string>("waypoint_format", "xyv");

    this->declare_parameter<std::vector<double>>(
      "waypoints",
      std::vector<double>{
        0.0,  0.0, 0.35,
       -1.5,  0.2, 0.35,
       -3.0,  1.0, 0.35});

    // Parameters used only in automatic xyv mode.
    this->declare_parameter<double>("min_segment_time", 1.0);
    this->declare_parameter<double>("time_safety_factor", 1.25);
    this->declare_parameter<bool>("zero_endpoint_steering", true);

    this->declare_parameter<bool>("enable_goal_input", false);
    this->declare_parameter<std::string>("goal_topic", "/goal_pose");
    this->declare_parameter<std::string>("base_frame", "base_link");
    this->declare_parameter<double>("goal_cruise_speed", 0.30);
    this->declare_parameter<double>("goal_final_speed", 0.05);
    this->declare_parameter<double>("minimum_goal_distance", 0.10);

    // Optional. If empty in xyv mode, segment times are computed from distance, velocity,
    // and steering-rate limit. If provided, it must contain number_of_waypoints - 1 values.
    this->declare_parameter<std::vector<double>>("segment_times", std::vector<double>{});
  }

  void load_parameters()
  {
    frame_id_ = this->get_parameter("frame_id").as_string();
    publish_period_ = this->get_parameter("publish_period").as_double();
    heading_stride_ = static_cast<int>(this->get_parameter("heading_stride").as_int());
    heading_scale_ = this->get_parameter("heading_scale").as_double();
    path_topic_ = this->get_parameter("path_topic").as_string();
    marker_topic_ = this->get_parameter("marker_topic").as_string();
    reference_topic_ = this->get_parameter("reference_topic").as_string();
    show_control_polygon_ = this->get_parameter("show_control_polygon").as_bool();
    show_waypoints_ = this->get_parameter("show_waypoints").as_bool();

    L_ = this->get_parameter("L").as_double();
    DT_ = this->get_parameter("DT").as_double();
    VARPHI_MAX_ = this->get_parameter("VARPHI_MAX").as_double();
    OMEGA_S_MAX_ = this->get_parameter("OMEGA_S_MAX").as_double();
    default_segment_time_ = this->get_parameter("default_segment_time").as_double();
    min_segment_time_ = this->get_parameter("min_segment_time").as_double();
    time_safety_factor_ = this->get_parameter("time_safety_factor").as_double();
    zero_endpoint_steering_ = this->get_parameter("zero_endpoint_steering").as_bool();
    enable_goal_input_ = this->get_parameter("enable_goal_input").as_bool();
    goal_topic_ = this->get_parameter("goal_topic").as_string();
    base_frame_ = this->get_parameter("base_frame").as_string();
    goal_cruise_speed_ = this->get_parameter("goal_cruise_speed").as_double();
    goal_final_speed_ = this->get_parameter("goal_final_speed").as_double();
    minimum_goal_distance_ = this->get_parameter("minimum_goal_distance").as_double();

    if (DT_ <= 0.0 || L_ <= 0.0 || default_segment_time_ <= 0.0 ||
      min_segment_time_ <= 0.0 || time_safety_factor_ <= 0.0)
    {
      throw std::runtime_error(
        "Parameters DT, L, default_segment_time, min_segment_time, and time_safety_factor must be > 0.");
    }
    if (VARPHI_MAX_ <= 0.0 || OMEGA_S_MAX_ <= 0.0) {
      throw std::runtime_error("Parameters VARPHI_MAX and OMEGA_S_MAX must be > 0.");
    }
    if (publish_period_ <= 0.0) {
      throw std::runtime_error("Parameter publish_period must be > 0.");
    }
    if (heading_stride_ <= 0) {
      throw std::runtime_error("Parameter heading_stride must be > 0.");
    }
    if (heading_scale_ <= 0.0) {
      throw std::runtime_error("Parameter heading_scale must be > 0.");
    }
    if (goal_cruise_speed_ <= 0.0 || goal_final_speed_ < 0.0 || minimum_goal_distance_ <= 0.0) {
      throw std::runtime_error("Parameters goal_cruise_speed and minimum_goal_distance must be > 0; goal_final_speed must be >= 0.");
    }

    const auto waypoint_format = this->get_parameter("waypoint_format").as_string();
    const auto wp_flat = this->get_parameter("waypoints").as_double_array();
    const auto segment_times_param = get_double_array_parameter_or_empty("segment_times");

    if (waypoint_format == "xyv") {
      load_auto_waypoints(wp_flat, segment_times_param);
    } else if (waypoint_format == "full") {
      load_full_waypoints(wp_flat, segment_times_param);
    } else {
      throw std::runtime_error("Invalid waypoint_format. Use 'xyv' or 'full'.");
    }

    for (double Tk : segment_times_) {
      if (Tk <= 0.0) {
        throw std::runtime_error("All segment_times must be > 0.");
      }
    }
  }

  std::vector<double> get_double_array_parameter_or_empty(const std::string & name) const
  {
    const auto p = this->get_parameter(name);

    if (p.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) {
      return {};
    }

    if (p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY) {
      return p.as_double_array();
    }

    if (p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY) {
      const auto values = p.as_integer_array();
      std::vector<double> out;
      out.reserve(values.size());
      for (const auto value : values) {
        out.push_back(static_cast<double>(value));
      }
      return out;
    }

    // YAML empty lists such as `segment_times: []` can be ambiguous in ROS 2.
    // Treat an empty byte array as an empty numeric array instead of aborting.
    if (p.get_type() == rclcpp::ParameterType::PARAMETER_BYTE_ARRAY) {
      const auto values = p.as_byte_array();
      if (values.empty()) {
        return {};
      }
    }

    throw std::runtime_error(
      "Parameter '" + name + "' must be a numeric array, for example [5.4, 6.1], or be omitted.");
  }

  void load_full_waypoints(
    const std::vector<double> & wp_flat,
    const std::vector<double> & segment_times_param)
  {
    if (wp_flat.size() < 12 || wp_flat.size() % 6 != 0) {
      throw std::runtime_error(
        "Parameter 'waypoints' must contain at least 2 waypoints and have size multiple of 6: "
        "[x, y, theta, varphi, v, omega_s, ...]. "
        "Or set waypoint_format='xyv' and use [x, y, v, ...].");
    }

    waypoints_.clear();
    waypoints_.reserve(wp_flat.size() / 6);
    for (std::size_t i = 0; i < wp_flat.size(); i += 6) {
      Waypoint wp;
      wp.x = wp_flat[i + 0];
      wp.y = wp_flat[i + 1];
      wp.theta = wp_flat[i + 2];
      wp.varphi = wp_flat[i + 3];
      wp.v = wp_flat[i + 4];
      wp.omega_s = wp_flat[i + 5];
      waypoints_.push_back(wp);
    }

    const std::size_t n_segments = waypoints_.size() - 1;
    segment_times_.clear();
    if (segment_times_param.empty()) {
      segment_times_.assign(n_segments, default_segment_time_);
    } else {
      if (segment_times_param.size() != n_segments) {
        throw std::runtime_error(
          "Parameter 'segment_times' must be empty or contain exactly number_of_waypoints - 1 values.");
      }
      segment_times_.assign(segment_times_param.begin(), segment_times_param.end());
    }
  }

  void load_auto_waypoints(
    const std::vector<double> & wp_xyv_flat,
    const std::vector<double> & segment_times_param)
  {
    if (wp_xyv_flat.size() < 6 || wp_xyv_flat.size() % 3 != 0) {
      throw std::runtime_error(
        "For waypoint_format='xyv', parameter 'waypoints' must contain at least 2 waypoints and have size multiple of 3: "
        "[x, y, v, x, y, v, ...].");
    }

    waypoints_.clear();
    waypoints_.reserve(wp_xyv_flat.size() / 3);
    for (std::size_t i = 0; i < wp_xyv_flat.size(); i += 3) {
      Waypoint wp;
      wp.x = wp_xyv_flat[i + 0];
      wp.y = wp_xyv_flat[i + 1];
      wp.v = wp_xyv_flat[i + 2];
      if (wp.v < 0.0) {
        throw std::runtime_error("All waypoint velocities in 'waypoints' with waypoint_format='xyv' must be >= 0.");
      }
      waypoints_.push_back(wp);
    }

    compute_auto_heading_and_steering();
    compute_or_load_segment_times(segment_times_param);
    compute_auto_steering_velocity();
    log_auto_waypoints();
  }

  void compute_or_load_segment_times(const std::vector<double> & segment_times_param)
  {
    const std::size_t n_segments = waypoints_.size() - 1;
    segment_times_.clear();

    if (!segment_times_param.empty()) {
      if (segment_times_param.size() != n_segments) {
        throw std::runtime_error(
          "Parameter 'segment_times' must be empty or contain exactly number_of_waypoints - 1 values.");
      }
      segment_times_.assign(segment_times_param.begin(), segment_times_param.end());
      return;
    }

    segment_times_.reserve(n_segments);
    for (std::size_t i = 0; i < n_segments; ++i) {
      const Vec2 pi{waypoints_[i].x, waypoints_[i].y};
      const Vec2 pj{waypoints_[i + 1].x, waypoints_[i + 1].y};
      const double distance = norm(sub(pj, pi));
      const double v_avg = 0.5 * (std::abs(waypoints_[i].v) + std::abs(waypoints_[i + 1].v));

      const double time_from_speed =
        (v_avg > kEps) ? (distance / v_avg) : default_segment_time_;
      const double time_from_steering_rate =
        std::abs(waypoints_[i + 1].varphi - waypoints_[i].varphi) / OMEGA_S_MAX_;

      const double Tk = time_safety_factor_ * std::max(
        {time_from_speed, time_from_steering_rate, min_segment_time_});
      segment_times_.push_back(Tk);
    }
  }

  static double binomial(int n, int k)
  {
    if (k < 0 || k > n) {
      return 0.0;
    }
    if (k == 0 || k == n) {
      return 1.0;
    }
    if (k > n - k) {
      k = n - k;
    }

    double result = 1.0;
    for (int i = 1; i <= k; ++i) {
      result *= static_cast<double>(n - (k - i));
      result /= static_cast<double>(i);
    }
    return result;
  }

  static double bernstein(int n, int i, double s)
  {
    return binomial(n, i) * std::pow(1.0 - s, n - i) * std::pow(s, i);
  }

  static Vec2 add(const Vec2 & a, const Vec2 & b)
  {
    return {a.x + b.x, a.y + b.y};
  }

  static Vec2 sub(const Vec2 & a, const Vec2 & b)
  {
    return {a.x - b.x, a.y - b.y};
  }

  static Vec2 mul(double c, const Vec2 & a)
  {
    return {c * a.x, c * a.y};
  }

  static double norm(const Vec2 & a)
  {
    return std::hypot(a.x, a.y);
  }

  static double cross(const Vec2 & a, const Vec2 & b)
  {
    return a.x * b.y - a.y * b.x;
  }

  static double normalize_angle(double angle)
  {
    return std::atan2(std::sin(angle), std::cos(angle));
  }

  static Vec2 unit_or_zero(const Vec2 & a)
  {
    const double n = norm(a);
    if (n < kEps) {
      return {0.0, 0.0};
    }
    return {a.x / n, a.y / n};
  }

  static double heading_from_vector(const Vec2 & a)
  {
    return std::atan2(a.y, a.x);
  }

  double discrete_curvature(std::size_t i) const
  {
    if (waypoints_.size() < 3) {
      return 0.0;
    }

    std::size_t im = 0;
    std::size_t ic = i;
    std::size_t ip = 0;

    if (i == 0) {
      if (zero_endpoint_steering_) {
        return 0.0;
      }
      im = 0;
      ic = 1;
      ip = 2;
    } else if (i + 1 == waypoints_.size()) {
      if (zero_endpoint_steering_) {
        return 0.0;
      }
      im = waypoints_.size() - 3;
      ic = waypoints_.size() - 2;
      ip = waypoints_.size() - 1;
    } else {
      im = i - 1;
      ic = i;
      ip = i + 1;
    }

    const Vec2 pm{waypoints_[im].x, waypoints_[im].y};
    const Vec2 pc{waypoints_[ic].x, waypoints_[ic].y};
    const Vec2 pp{waypoints_[ip].x, waypoints_[ip].y};

    const Vec2 d_minus = sub(pc, pm);
    const Vec2 d_plus = sub(pp, pc);
    const double a = norm(d_minus);
    const double b = norm(d_plus);
    const double c = norm(sub(pp, pm));

    if (a < kEps || b < kEps || c < kEps) {
      return 0.0;
    }

    return 2.0 * cross(d_minus, d_plus) / (a * b * c);
  }

  void compute_auto_heading_and_steering()
  {
    const std::size_t n = waypoints_.size();
    if (n < 2) {
      throw std::runtime_error("At least 2 waypoints are required.");
    }

    for (std::size_t i = 0; i < n; ++i) {
      Vec2 tangent{0.0, 0.0};

      if (i == 0) {
        tangent = sub(Vec2{waypoints_[1].x, waypoints_[1].y}, Vec2{waypoints_[0].x, waypoints_[0].y});
      } else if (i + 1 == n) {
        tangent = sub(Vec2{waypoints_[n - 1].x, waypoints_[n - 1].y}, Vec2{waypoints_[n - 2].x, waypoints_[n - 2].y});
      } else {
        const Vec2 p_prev{waypoints_[i - 1].x, waypoints_[i - 1].y};
        const Vec2 p_curr{waypoints_[i].x, waypoints_[i].y};
        const Vec2 p_next{waypoints_[i + 1].x, waypoints_[i + 1].y};
        const Vec2 u_in = unit_or_zero(sub(p_curr, p_prev));
        const Vec2 u_out = unit_or_zero(sub(p_next, p_curr));
        tangent = add(u_in, u_out);

        // If the local path makes an almost 180-degree turn, the bisector is near zero.
        // Fall back to the chord direction.
        if (norm(tangent) < kEps) {
          tangent = sub(p_next, p_prev);
        }
      }

      if (norm(tangent) < kEps) {
        throw std::runtime_error("Consecutive waypoints must not have the same x,y position.");
      }

      waypoints_[i].theta = heading_from_vector(tangent);
      const double kappa_i = discrete_curvature(i);
      waypoints_[i].varphi = std::atan(L_ * kappa_i);
    }

    // Unwrap headings for cleaner logging and interpolation around +/-pi.
    for (std::size_t i = 1; i < n; ++i) {
      waypoints_[i].theta = waypoints_[i - 1].theta +
        normalize_angle(waypoints_[i].theta - waypoints_[i - 1].theta);
    }

    for (std::size_t i = 0; i < n; ++i) {
      if (std::abs(waypoints_[i].varphi) > VARPHI_MAX_) {
        RCLCPP_WARN(
          this->get_logger(),
          "Auto waypoint %zu has |varphi| = %.4f rad > limit %.4f rad. "
          "The x,y waypoints make a turn that is too sharp; move the points or add smoother intermediate points.",
          i, std::abs(waypoints_[i].varphi), VARPHI_MAX_);
      }
    }
  }

  void compute_auto_steering_velocity()
  {
    const std::size_t n = waypoints_.size();
    if (n < 2) {
      return;
    }

    if (n == 2) {
      waypoints_[0].omega_s = 0.0;
      waypoints_[1].omega_s = 0.0;
      return;
    }

    waypoints_[0].omega_s =
      (waypoints_[1].varphi - waypoints_[0].varphi) / segment_times_[0];

    for (std::size_t i = 1; i + 1 < n; ++i) {
      const double dt = segment_times_[i - 1] + segment_times_[i];
      waypoints_[i].omega_s =
        (waypoints_[i + 1].varphi - waypoints_[i - 1].varphi) / std::max(dt, kEps);
    }

    waypoints_[n - 1].omega_s =
      (waypoints_[n - 1].varphi - waypoints_[n - 2].varphi) /
      segment_times_[n - 2];

    for (std::size_t i = 0; i < n; ++i) {
      if (std::abs(waypoints_[i].omega_s) > OMEGA_S_MAX_) {
        RCLCPP_WARN(
          this->get_logger(),
          "Auto waypoint %zu has |omega_s| = %.4f rad/s > limit %.4f rad/s. "
          "Increase segment_times or lower the waypoint velocities.",
          i, std::abs(waypoints_[i].omega_s), OMEGA_S_MAX_);
      }
    }
  }

  void log_auto_waypoints() const
  {
    RCLCPP_INFO(this->get_logger(), "Using auto waypoint mode: waypoint_format='xyv', input [x, y, v], computed [theta, varphi, omega_s].");
    for (std::size_t i = 0; i < waypoints_.size(); ++i) {
      RCLCPP_INFO(
        this->get_logger(),
        "WP %zu: x=%.4f y=%.4f theta=%.4f varphi=%.4f v=%.4f omega_s=%.4f",
        i,
        waypoints_[i].x,
        waypoints_[i].y,
        waypoints_[i].theta,
        waypoints_[i].varphi,
        waypoints_[i].v,
        waypoints_[i].omega_s);
    }
    for (std::size_t i = 0; i < segment_times_.size(); ++i) {
      RCLCPP_INFO(this->get_logger(), "Segment %zu time = %.4f s", i, segment_times_[i]);
    }
  }

  Vec2 velocity_from_waypoint(const Waypoint & wp) const
  {
    return {
      wp.v * std::cos(wp.theta),
      wp.v * std::sin(wp.theta)};
  }

  Vec2 acceleration_from_waypoint(const Waypoint & wp) const
  {
    // This assumes dot(v) = 0 at the waypoint.
    const double normal_gain = (wp.v * wp.v / L_) * std::tan(wp.varphi);
    return {
      normal_gain * (-std::sin(wp.theta)),
      normal_gain * ( std::cos(wp.theta))};
  }

  Vec2 jerk_from_waypoint(const Waypoint & wp) const
  {
    // This assumes dot(v) = 0 at the waypoint.
    // e = [cos(theta), sin(theta)], n = [-sin(theta), cos(theta)].
    const double c = std::cos(wp.theta);
    const double s = std::sin(wp.theta);
    const double tan_varphi = std::tan(wp.varphi);
    const double sec2_varphi = 1.0 / std::max(std::pow(std::cos(wp.varphi), 2), kEps);

    const double e_gain = -(wp.v * wp.v * wp.v / (L_ * L_)) * tan_varphi * tan_varphi;
    const double n_gain = (wp.v * wp.v / L_) * sec2_varphi * wp.omega_s;

    return {
      e_gain * c + n_gain * (-s),
      e_gain * s + n_gain * ( c)};
  }

  std::array<Vec2, 8> make_segment_control_points(
    const Waypoint & a, const Waypoint & b, double T) const
  {
    const Vec2 z0{a.x, a.y};
    const Vec2 zT{b.x, b.y};
    const Vec2 dz0 = velocity_from_waypoint(a);
    const Vec2 dzT = velocity_from_waypoint(b);
    const Vec2 ddz0 = acceleration_from_waypoint(a);
    const Vec2 ddzT = acceleration_from_waypoint(b);
    const Vec2 dddz0 = jerk_from_waypoint(a);
    const Vec2 dddzT = jerk_from_waypoint(b);

    std::array<Vec2, 8> C{};

    C[0] = z0;
    C[1] = add(C[0], mul(T / 7.0, dz0));
    C[2] = add(sub(mul(2.0, C[1]), C[0]), mul((T * T) / 42.0, ddz0));
    C[3] = add(
      add(sub(mul(3.0, C[2]), mul(3.0, C[1])), C[0]),
      mul((T * T * T) / 210.0, dddz0));

    C[7] = zT;
    C[6] = sub(C[7], mul(T / 7.0, dzT));
    C[5] = add(sub(mul(2.0, C[6]), C[7]), mul((T * T) / 42.0, ddzT));
    C[4] = sub(add(sub(C[7], mul(3.0, C[6])), mul(3.0, C[5])), mul((T * T * T) / 210.0, dddzT));

    return C;
  }

  void evaluate_segment(
    const std::array<Vec2, 8> & C, double T, double tau,
    Vec2 & z, Vec2 & dz, Vec2 & ddz, Vec2 & dddz) const
  {
    const double s = std::clamp(tau / T, 0.0, 1.0);

    std::array<Vec2, 7> D1{};
    std::array<Vec2, 6> D2{};
    std::array<Vec2, 5> D3{};

    for (int i = 0; i < 7; ++i) {
      D1[i] = sub(C[i + 1], C[i]);
    }
    for (int i = 0; i < 6; ++i) {
      D2[i] = add(sub(C[i + 2], mul(2.0, C[i + 1])), C[i]);
    }
    for (int i = 0; i < 5; ++i) {
      D3[i] = add(
        sub(add(C[i + 3], mul(3.0, C[i + 1])), mul(3.0, C[i + 2])),
        mul(-1.0, C[i]));
    }

    z = {0.0, 0.0};
    dz = {0.0, 0.0};
    ddz = {0.0, 0.0};
    dddz = {0.0, 0.0};

    for (int i = 0; i <= 7; ++i) {
      const double b = bernstein(7, i, s);
      z.x += C[i].x * b;
      z.y += C[i].y * b;
    }
    for (int i = 0; i <= 6; ++i) {
      const double b = bernstein(6, i, s);
      dz.x += D1[i].x * b;
      dz.y += D1[i].y * b;
    }
    dz = mul(7.0 / T, dz);

    for (int i = 0; i <= 5; ++i) {
      const double b = bernstein(5, i, s);
      ddz.x += D2[i].x * b;
      ddz.y += D2[i].y * b;
    }
    ddz = mul(42.0 / (T * T), ddz);

    for (int i = 0; i <= 4; ++i) {
      const double b = bernstein(4, i, s);
      dddz.x += D3[i].x * b;
      dddz.y += D3[i].y * b;
    }
    dddz = mul(210.0 / (T * T * T), dddz);
  }

  Sample make_sample(double global_t, const Vec2 & z, const Vec2 & dz, const Vec2 & ddz, const Vec2 & dddz, int segment) const
  {
    const double v_ref = std::hypot(dz.x, dz.y);
    const double v_safe = std::max(v_ref, kEps);
    const double theta = std::atan2(dz.y, dz.x);

    const double N = dz.x * ddz.y - dz.y * ddz.x;
    const double v3 = std::max(std::pow(v_safe, 3), kEps);
    const double kappa = N / v3;
    const double varphi = std::atan(L_ * kappa);

    const double N_dot = dz.x * dddz.y - dz.y * dddz.x;
    const double v3_dot = 3.0 * v_safe * (dz.x * ddz.x + dz.y * ddz.y);
    const double kappa_dot =
      (N_dot * std::pow(v_safe, 3) - N * v3_dot) / std::max(std::pow(v_safe, 6), kEps);
    const double omega_s = (L_ * kappa_dot) / (1.0 + std::pow(L_ * kappa, 2));

    Sample sample;
    sample.t = global_t;
    sample.x = z.x;
    sample.y = z.y;
    sample.theta = theta;
    sample.varphi = varphi;
    sample.v = v_ref;
    sample.omega_s = omega_s;
    sample.segment = segment;
    return sample;
  }


  static double yaw_from_quaternion(double qx, double qy, double qz, double qw)
  {
    const double siny_cosp = 2.0 * (qw * qz + qx * qy);
    const double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
    return std::atan2(siny_cosp, cosy_cosp);
  }

  void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (!enable_goal_input_) {
      return;
    }

    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_->lookupTransform(
        frame_id_, base_frame_, tf2::TimePointZero,
        std::chrono::milliseconds(100));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(
        this->get_logger(),
        "Cannot create Bezier trajectory from goal because TF %s -> %s is unavailable: %s",
        frame_id_.c_str(), base_frame_.c_str(), ex.what());
      return;
    }

    const double x0 = tf.transform.translation.x;
    const double y0 = tf.transform.translation.y;
    const double theta0 = yaw_from_quaternion(
      tf.transform.rotation.x, tf.transform.rotation.y,
      tf.transform.rotation.z, tf.transform.rotation.w);

    const double xg = msg->pose.position.x;
    const double yg = msg->pose.position.y;
    const double thetag = yaw_from_quaternion(
      msg->pose.orientation.x, msg->pose.orientation.y,
      msg->pose.orientation.z, msg->pose.orientation.w);

    const double distance = std::hypot(xg - x0, yg - y0);
    if (distance < minimum_goal_distance_) {
      RCLCPP_WARN(
        this->get_logger(),
        "Received goal is too close to the current pose: distance=%.3f m < %.3f m. Ignored.",
        distance, minimum_goal_distance_);
      return;
    }

    Waypoint start;
    start.x = x0;
    start.y = y0;
    start.theta = theta0;
    start.varphi = 0.0;
    start.v = goal_cruise_speed_;
    start.omega_s = 0.0;

    Waypoint goal;
    goal.x = xg;
    goal.y = yg;
    goal.theta = thetag;
    goal.varphi = 0.0;
    goal.v = goal_final_speed_;
    goal.omega_s = 0.0;

    waypoints_.clear();
    waypoints_.push_back(start);
    waypoints_.push_back(goal);

    const double v_avg = std::max(0.5 * (goal_cruise_speed_ + goal_final_speed_), 0.05);
    const double T = time_safety_factor_ * std::max(distance / v_avg, min_segment_time_);
    segment_times_.clear();
    segment_times_.push_back(T);

    RCLCPP_INFO(
      this->get_logger(),
      "Received new goal. Generating one-segment Bezier trajectory: start=(%.3f, %.3f, %.3f), goal=(%.3f, %.3f, %.3f), T=%.3f s",
      start.x, start.y, start.theta, goal.x, goal.y, goal.theta, T);

    compute_trajectory();
    publish_all();
  }

  void compute_trajectory()
  {
    controls_.clear();
    controls_.reserve(waypoints_.size() - 1);

    for (std::size_t i = 0; i + 1 < waypoints_.size(); ++i) {
      controls_.push_back(make_segment_control_points(waypoints_[i], waypoints_[i + 1], segment_times_[i]));
    }

    samples_.clear();

    double global_t = 0.0;
    double max_v = 0.0;
    double max_varphi = 0.0;
    double max_omega_s = 0.0;

    for (std::size_t seg = 0; seg < controls_.size(); ++seg) {
      const double T = segment_times_[seg];
      const int n_steps = std::max(1, static_cast<int>(std::ceil(T / DT_)));

      for (int k = 0; k <= n_steps; ++k) {
        if (seg > 0 && k == 0) {
          continue;  // avoid duplicated sample at the shared waypoint
        }

        const double tau = (k == n_steps) ? T : std::min(k * DT_, T);
        Vec2 z, dz, ddz, dddz;
        evaluate_segment(controls_[seg], T, tau, z, dz, ddz, dddz);

        Sample sample = make_sample(global_t + tau, z, dz, ddz, dddz, static_cast<int>(seg));
        samples_.push_back(sample);

        max_v = std::max(max_v, std::abs(sample.v));
        max_varphi = std::max(max_varphi, std::abs(sample.varphi));
        max_omega_s = std::max(max_omega_s, std::abs(sample.omega_s));
      }

      global_t += T;
    }

    RCLCPP_INFO(this->get_logger(), "Generated piecewise Bezier-7 trajectory.");
    RCLCPP_INFO(this->get_logger(), "Waypoints: %zu, segments: %zu, samples: %zu", waypoints_.size(), controls_.size(), samples_.size());
    RCLCPP_INFO(this->get_logger(), "Total time = %.4f s", global_t);
    RCLCPP_INFO(this->get_logger(), "Max speed       = %.4f m/s", max_v);
    RCLCPP_INFO(this->get_logger(), "Max |varphi|    = %.4f rad   (limit %.4f)", max_varphi, VARPHI_MAX_);
    RCLCPP_INFO(this->get_logger(), "Max |omega_s|   = %.4f rad/s (limit %.4f)", max_omega_s, OMEGA_S_MAX_);

    if (max_varphi > VARPHI_MAX_) {
      RCLCPP_WARN(this->get_logger(), "Steering angle limit exceeded. Increase segment time or adjust waypoints.");
    }
    if (max_omega_s > OMEGA_S_MAX_) {
      RCLCPP_WARN(this->get_logger(), "Steering angular velocity limit exceeded. Increase segment time or adjust waypoints.");
    }
  }

  void publish_all()
  {
    if (samples_.empty()) {
      return;
    }

    const auto now = this->now();

    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = now;
    path_msg.header.frame_id = frame_id_;

    for (const auto & s : samples_) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path_msg.header;
      pose.pose.position.x = s.x;
      pose.pose.position.y = s.y;
      pose.pose.position.z = 0.0;
      pose.pose.orientation.x = 0.0;
      pose.pose.orientation.y = 0.0;
      pose.pose.orientation.z = std::sin(0.5 * s.theta);
      pose.pose.orientation.w = std::cos(0.5 * s.theta);
      path_msg.poses.push_back(pose);
    }

    visualization_msgs::msg::MarkerArray marker_array;

    visualization_msgs::msg::Marker traj;
    traj.header.frame_id = frame_id_;
    traj.header.stamp = now;
    traj.ns = "bezier7";
    traj.id = 0;
    traj.type = visualization_msgs::msg::Marker::LINE_STRIP;
    traj.action = visualization_msgs::msg::Marker::ADD;
    traj.scale.x = 0.03;
    traj.pose.orientation.w = 1.0;
    traj.color.r = 1.0f;
    traj.color.g = 0.3f;
    traj.color.b = 0.1f;
    traj.color.a = 1.0f;
    for (const auto & s : samples_) {
      geometry_msgs::msg::Point p;
      p.x = s.x;
      p.y = s.y;
      p.z = 0.02;
      traj.points.push_back(p);
    }
    marker_array.markers.push_back(traj);

    visualization_msgs::msg::Marker start;
    start.header.frame_id = frame_id_;
    start.header.stamp = now;
    start.ns = "bezier7";
    start.id = 1;
    start.type = visualization_msgs::msg::Marker::SPHERE;
    start.action = visualization_msgs::msg::Marker::ADD;
    start.pose.position.x = samples_.front().x;
    start.pose.position.y = samples_.front().y;
    start.pose.position.z = 0.03;
    start.pose.orientation.w = 1.0;
    start.scale.x = 0.12;
    start.scale.y = 0.12;
    start.scale.z = 0.12;
    start.color.r = 0.0f;
    start.color.g = 1.0f;
    start.color.b = 0.0f;
    start.color.a = 1.0f;
    marker_array.markers.push_back(start);

    visualization_msgs::msg::Marker goal;
    goal.header.frame_id = frame_id_;
    goal.header.stamp = now;
    goal.ns = "bezier7";
    goal.id = 2;
    goal.type = visualization_msgs::msg::Marker::SPHERE;
    goal.action = visualization_msgs::msg::Marker::ADD;
    goal.pose.position.x = samples_.back().x;
    goal.pose.position.y = samples_.back().y;
    goal.pose.position.z = 0.03;
    goal.pose.orientation.w = 1.0;
    goal.scale.x = 0.12;
    goal.scale.y = 0.12;
    goal.scale.z = 0.12;
    goal.color.r = 1.0f;
    goal.color.g = 0.0f;
    goal.color.b = 0.0f;
    goal.color.a = 1.0f;
    marker_array.markers.push_back(goal);

    visualization_msgs::msg::Marker heading;
    heading.header.frame_id = frame_id_;
    heading.header.stamp = now;
    heading.ns = "bezier7";
    heading.id = 3;
    heading.type = visualization_msgs::msg::Marker::LINE_LIST;
    heading.action = visualization_msgs::msg::Marker::ADD;
    heading.scale.x = 0.015;
    heading.pose.orientation.w = 1.0;
    heading.color.r = 0.1f;
    heading.color.g = 0.8f;
    heading.color.b = 1.0f;
    heading.color.a = 1.0f;

    const int stride = std::max(1, heading_stride_);
    for (std::size_t i = 0; i < samples_.size(); i += static_cast<std::size_t>(stride)) {
      geometry_msgs::msg::Point p0;
      p0.x = samples_[i].x;
      p0.y = samples_[i].y;
      p0.z = 0.04;

      geometry_msgs::msg::Point p1;
      p1.x = samples_[i].x + heading_scale_ * std::cos(samples_[i].theta);
      p1.y = samples_[i].y + heading_scale_ * std::sin(samples_[i].theta);
      p1.z = 0.04;

      heading.points.push_back(p0);
      heading.points.push_back(p1);
    }
    marker_array.markers.push_back(heading);

    if (show_control_polygon_) {
      visualization_msgs::msg::Marker ctrl;
      ctrl.header.frame_id = frame_id_;
      ctrl.header.stamp = now;
      ctrl.ns = "bezier7";
      ctrl.id = 4;
      ctrl.type = visualization_msgs::msg::Marker::LINE_LIST;
      ctrl.action = visualization_msgs::msg::Marker::ADD;
      ctrl.scale.x = 0.01;
      ctrl.pose.orientation.w = 1.0;
      ctrl.color.r = 1.0f;
      ctrl.color.g = 1.0f;
      ctrl.color.b = 0.0f;
      ctrl.color.a = 1.0f;

      for (const auto & C : controls_) {
        for (std::size_t i = 0; i + 1 < C.size(); ++i) {
          geometry_msgs::msg::Point p0;
          p0.x = C[i].x;
          p0.y = C[i].y;
          p0.z = 0.05;
          geometry_msgs::msg::Point p1;
          p1.x = C[i + 1].x;
          p1.y = C[i + 1].y;
          p1.z = 0.05;
          ctrl.points.push_back(p0);
          ctrl.points.push_back(p1);
        }
      }
      marker_array.markers.push_back(ctrl);
    }

    if (show_waypoints_) {
      visualization_msgs::msg::Marker wp_marker;
      wp_marker.header.frame_id = frame_id_;
      wp_marker.header.stamp = now;
      wp_marker.ns = "bezier7";
      wp_marker.id = 5;
      wp_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
      wp_marker.action = visualization_msgs::msg::Marker::ADD;
      wp_marker.scale.x = 0.08;
      wp_marker.scale.y = 0.08;
      wp_marker.scale.z = 0.08;
      wp_marker.pose.orientation.w = 1.0;
      wp_marker.color.r = 0.8f;
      wp_marker.color.g = 0.2f;
      wp_marker.color.b = 1.0f;
      wp_marker.color.a = 1.0f;

      for (const auto & wp : waypoints_) {
        geometry_msgs::msg::Point p;
        p.x = wp.x;
        p.y = wp.y;
        p.z = 0.06;
        wp_marker.points.push_back(p);
      }
      marker_array.markers.push_back(wp_marker);
    }

    path_pub_->publish(path_msg);
    marker_pub_->publish(marker_array);
    publish_reference_trajectory();
  }

  void publish_reference_trajectory()
  {
    if (!reference_pub_ || samples_.empty()) {
      return;
    }

    std_msgs::msg::Float64MultiArray msg;
    msg.layout.dim.resize(2);
    msg.layout.dim[0].label = "points";
    msg.layout.dim[0].size = static_cast<uint32_t>(samples_.size());
    msg.layout.dim[0].stride = static_cast<uint32_t>(samples_.size() * 8U);
    msg.layout.dim[1].label = "fields_t_x_y_theta_varphi_v_omega_s_segment";
    msg.layout.dim[1].size = 8U;
    msg.layout.dim[1].stride = 8U;
    msg.layout.data_offset = 0U;

    msg.data.reserve(samples_.size() * 8U);
    for (const auto & s : samples_) {
      msg.data.push_back(s.t);
      msg.data.push_back(s.x);
      msg.data.push_back(s.y);
      msg.data.push_back(s.theta);
      msg.data.push_back(s.varphi);
      msg.data.push_back(s.v);
      msg.data.push_back(s.omega_s);
      msg.data.push_back(static_cast<double>(s.segment));
    }

    reference_pub_->publish(msg);
  }

  std::string frame_id_;
  std::string path_topic_{"/bezier_path"};
  std::string marker_topic_{"/bezier_markers"};
  std::string reference_topic_{"/qcar2/reference_trajectory"};
  std::string goal_topic_{"/goal_pose"};
  std::string base_frame_{"base_link"};
  double publish_period_{1.0};
  int heading_stride_{25};
  double heading_scale_{0.18};
  bool show_control_polygon_{true};
  bool show_waypoints_{true};

  double L_{0.25725};
  double DT_{0.02};
  double VARPHI_MAX_{0.5236};
  double OMEGA_S_MAX_{2.0};
  double default_segment_time_{8.0};
  double min_segment_time_{1.0};
  double time_safety_factor_{1.25};
  bool zero_endpoint_steering_{true};
  bool enable_goal_input_{false};
  double goal_cruise_speed_{0.30};
  double goal_final_speed_{0.05};
  double minimum_goal_distance_{0.10};

  std::vector<Waypoint> waypoints_;
  std::vector<double> segment_times_;
  std::vector<std::array<Vec2, 8>> controls_;
  std::vector<Sample> samples_;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr reference_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    rclcpp::spin(std::make_shared<QCar2BezierNode>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("qcar2_bezier"), "Exception: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}

