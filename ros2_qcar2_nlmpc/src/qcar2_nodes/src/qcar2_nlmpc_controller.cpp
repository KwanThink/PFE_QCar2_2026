#include "qcar2_nlmpc_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <stdexcept>
#include <utility>

#include "tf2/exceptions.h"
#include "tf2/time.h"

#ifdef QCAR2_HAS_ACADOS_SOLVER
extern "C" {
#include "acados_solver_qcar2_single_track.h"
#include "acados_c/ocp_nlp_interface.h"
}
#endif

namespace qcar2_nlmpc
{
namespace
{
constexpr double kPi = 3.1415926535897932384626433832795;
constexpr double kRawMotorSpeedToLinearVelocity =  (1.0 / (720.0 * 4.0)) * ((13.0 * 19.0) / (70.0 * 37.0)) * (2.0 * kPi) * 0.033;

// Return true when a floating-point value can safely be used by the controller.
bool isFinite(double value)
{
  return std::isfinite(value);
}

// Return the Euclidean distance between the vehicle state and the active goal position.
double distanceToGoal(const VehicleState & state, const Pose2D & goal_pose)
{
  return std::hypot(state.x - goal_pose.x, state.y - goal_pose.y);
}

}  // namespace
// AcadosSingleTrackSolver constructor
AcadosSingleTrackSolver::AcadosSingleTrackSolver(const NLMPCParameters & parameters)
: parameters_(parameters)
{
}

AcadosSingleTrackSolver::~AcadosSingleTrackSolver()
{
#ifdef QCAR2_HAS_ACADOS_SOLVER
  if (solver_capsule_ != nullptr) {
    auto * typed_capsule = static_cast<qcar2_single_track_solver_capsule *>(solver_capsule_);
    qcar2_single_track_acados_free(typed_capsule);
    qcar2_single_track_acados_free_capsule(typed_capsule);
    solver_capsule_ = nullptr;
  }
#endif
}

bool AcadosSingleTrackSolver::initialize(const rclcpp::Logger & logger)
{
#ifdef QCAR2_HAS_ACADOS_SOLVER
  auto * typed_capsule = qcar2_single_track_acados_create_capsule();
  if (typed_capsule == nullptr) {
    RCLCPP_ERROR(logger, "Failed to allocate qcar2_single_track acados capsule.");
    solver_available_ = false;
    return false;
  }

  const int status = qcar2_single_track_acados_create(typed_capsule);
  if (status != 0) {
    RCLCPP_ERROR(logger, "qcar2_single_track_acados_create failed with status %d.", status);
    qcar2_single_track_acados_free_capsule(typed_capsule);
    solver_available_ = false;
    return false;
  }

  solver_capsule_ = typed_capsule;
  solver_available_ = true;
  RCLCPP_INFO(logger, "Generated acados solver is available for qcar2_nlmpc_controller.");
  return true;
#else
  (void)logger;
  solver_available_ = false;
  return false;
#endif
}

SolverResult AcadosSingleTrackSolver::solve(
  const std::array<double, 4> & current_state,
  const std::vector<std::array<double, 4>> & state_reference_horizon,
  const std::vector<std::array<double, 2>> & input_reference_horizon,
  const std::array<double, 2> & input_initial_guess)
{
  SolverResult result;

#ifndef QCAR2_HAS_ACADOS_SOLVER
  (void)current_state;
  (void)state_reference_horizon;
  (void)input_reference_horizon;
  (void)input_initial_guess;
  result.status = -99;
  result.success = false;
  return result;
#else
  if (!solver_available_ || solver_capsule_ == nullptr) {
    result.status = -98;
    result.success = false;
    return result;
  }

  auto * capsule = static_cast<qcar2_single_track_solver_capsule *>(solver_capsule_);
  ocp_nlp_config * nlp_config = qcar2_single_track_acados_get_nlp_config(capsule);
  ocp_nlp_dims * nlp_dims = qcar2_single_track_acados_get_nlp_dims(capsule);
  ocp_nlp_in * nlp_in = qcar2_single_track_acados_get_nlp_in(capsule);
  ocp_nlp_out * nlp_out = qcar2_single_track_acados_get_nlp_out(capsule);
  
  // Solver fits the 1st state, lowerbounds (lbx) = upperbounds (ubx) = x0
  double x0[4] = {current_state[0], current_state[1], current_state[2], current_state[3]};
  ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, 0, "lbx", x0);
  ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, 0, "ubx", x0);

  for (int stage = 0; stage < parameters_.horizon_steps; ++stage) {
    // Solver tries to find the state & input that are close to yref, depends on Q, R and P (or Qe)
    double yref[6] = {
      state_reference_horizon[stage][0],
      state_reference_horizon[stage][1],
      state_reference_horizon[stage][2],
      state_reference_horizon[stage][3],
      input_reference_horizon[stage][0],
      input_reference_horizon[stage][1]};
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, stage, "yref", yref);

    // Solver initializes the guessing solution
    double x_guess[4] = {
      state_reference_horizon[stage][0],
      state_reference_horizon[stage][1],
      state_reference_horizon[stage][2],
      state_reference_horizon[stage][3]
    };
    double u_guess[2] = {
      input_reference_horizon[stage][0], 
      input_reference_horizon[stage][1]
    };
    ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, nlp_in, stage, "x", x_guess);
    ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, nlp_in, stage, "u", u_guess);
  }
  
  // Set the terminal reference state
  double yref_terminal[4] = {
    state_reference_horizon[parameters_.horizon_steps][0],
    state_reference_horizon[parameters_.horizon_steps][1],
    state_reference_horizon[parameters_.horizon_steps][2],
    state_reference_horizon[parameters_.horizon_steps][3]};
  // Solver has to achieve the final state
  ocp_nlp_cost_model_set(
    nlp_config, nlp_dims, nlp_in, parameters_.horizon_steps, "yref", yref_terminal);
  // Set the initial guess by the final state yref
  ocp_nlp_out_set(
    nlp_config, nlp_dims, nlp_out, nlp_in, parameters_.horizon_steps, "x", yref_terminal);

  // Only 1 time: Process the initial guess for u0, then call the solver later
  if (!has_initialized_guess_) {
    double first_u_guess[2] = {
      input_initial_guess[0], 
      input_initial_guess[1]
    };
    ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, nlp_in, 0, "u", first_u_guess);
    has_initialized_guess_ = true;
  }

  result.status = qcar2_single_track_acados_solve(capsule);
  if (result.status != 0) {
    result.success = false;
    return result;
  }

  double optimal_input[2] = {0.0, 0.0};
  ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, 0, "u", optimal_input);
  result.delta = optimal_input[0];
  result.ax = optimal_input[1];
  result.success = true;
  return result;
#endif
}

QCar2NLMPCController::QCar2NLMPCController(): Node("qcar2_nlmpc_controller")
{
  declareAndLoadParameters();

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  trajectory_generator_ = std::make_unique<QCar2BezierTrajectoryGenerator>(trajectory_parameters_);
  solver_ = std::make_unique<AcadosSingleTrackSolver>(nlmpc_parameters_);
  if (!solver_->initialize(this->get_logger())) {
    RCLCPP_WARN(
      this->get_logger(),
      "Generated acados solver was not found at build time. The controller will stay safe with zero commands until the solver is generated and the package is rebuilt.");
  }

  command_publisher_ = this->create_publisher<qcar2_interfaces::msg::MotorCommands>(command_topic_, 10);
  reference_path_publisher_ = this->create_publisher<nav_msgs::msg::Path>(reference_path_topic_, 10);

  joint_state_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
    joint_state_topic_, 10, std::bind(&QCar2NLMPCController::jointStateCallback, this, std::placeholders::_1));

  goal_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    goal_topic_, 10, std::bind(&QCar2NLMPCController::goalCallback, this, std::placeholders::_1));

  const auto period = std::chrono::duration<double>(control_period_);
  control_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&QCar2NLMPCController::controlTimerCallback, this));

  publishZeroCommand();
  RCLCPP_INFO(this->get_logger(), "qcar2_nlmpc_controller is waiting for map -> base_link and /goal_pose.");
}

void QCar2NLMPCController::declareAndLoadParameters()
{
  this->declare_parameter("map_frame", map_frame_);
  this->declare_parameter("base_frame", base_frame_);
  this->declare_parameter("goal_topic", goal_topic_);
  this->declare_parameter("joint_state_topic", joint_state_topic_);
  this->declare_parameter("command_topic", command_topic_);
  this->declare_parameter("reference_path_topic", reference_path_topic_);
  this->declare_parameter("control_period", control_period_);
  this->declare_parameter("transform_timeout", transform_timeout_);
  this->declare_parameter("feedback_timeout", feedback_timeout_);
  this->declare_parameter("goal_position_tolerance", goal_position_tolerance_);
  this->declare_parameter("stop_on_solver_failure", stop_on_solver_failure_);

  this->declare_parameter("wheelbase", nlmpc_parameters_.wheelbase);
  this->declare_parameter("Ts", nlmpc_parameters_.sample_time);
  this->declare_parameter("mpc_N", nlmpc_parameters_.horizon_steps);
  this->declare_parameter("vx_min", nlmpc_parameters_.vx_min);
  this->declare_parameter("vx_max", nlmpc_parameters_.vx_max);
  this->declare_parameter("delta_min", nlmpc_parameters_.delta_min);
  this->declare_parameter("delta_max", nlmpc_parameters_.delta_max);
  this->declare_parameter("ax_min", nlmpc_parameters_.ax_min);
  this->declare_parameter("ax_max", nlmpc_parameters_.ax_max);
  this->declare_parameter<std::vector<double>>("Q", {25.0, 25.0, 8.0, 3.0});
  this->declare_parameter<std::vector<double>>("R", {0.4, 0.4});
  this->declare_parameter<std::vector<double>>("Qe", {60.0, 60.0, 15.0, 6.0});

  this->declare_parameter("trajectory_generation_mode", trajectory_parameters_.trajectory_generation_mode);
  this->declare_parameter("number_of_waypoints", trajectory_parameters_.number_of_waypoints);
  this->declare_parameter<std::vector<double>>("segment_times", trajectory_parameters_.segment_times);
  this->declare_parameter("default_speed", trajectory_parameters_.default_speed);
  this->declare_parameter("minimum_segment_time", trajectory_parameters_.minimum_segment_time);
  this->declare_parameter("intermediate_tangent_scale", trajectory_parameters_.intermediate_tangent_scale);
  this->declare_parameter("minimum_goal_distance", trajectory_parameters_.minimum_goal_distance);
  this->declare_parameter("zero_endpoint_steering", trajectory_parameters_.zero_endpoint_steering);

  map_frame_ = this->get_parameter("map_frame").as_string();
  base_frame_ = this->get_parameter("base_frame").as_string();
  goal_topic_ = this->get_parameter("goal_topic").as_string();
  joint_state_topic_ = this->get_parameter("joint_state_topic").as_string();
  command_topic_ = this->get_parameter("command_topic").as_string();
  reference_path_topic_ = this->get_parameter("reference_path_topic").as_string();
  control_period_ = this->get_parameter("control_period").as_double();
  transform_timeout_ = this->get_parameter("transform_timeout").as_double();
  feedback_timeout_ = this->get_parameter("feedback_timeout").as_double();
  goal_position_tolerance_ = this->get_parameter("goal_position_tolerance").as_double();
  stop_on_solver_failure_ = this->get_parameter("stop_on_solver_failure").as_bool();

  nlmpc_parameters_.wheelbase = this->get_parameter("wheelbase").as_double();
  nlmpc_parameters_.sample_time = this->get_parameter("Ts").as_double();
  nlmpc_parameters_.horizon_steps = this->get_parameter("mpc_N").as_int();
  nlmpc_parameters_.vx_min = this->get_parameter("vx_min").as_double();
  nlmpc_parameters_.vx_max = this->get_parameter("vx_max").as_double();
  nlmpc_parameters_.delta_min = this->get_parameter("delta_min").as_double();
  nlmpc_parameters_.delta_max = this->get_parameter("delta_max").as_double();
  nlmpc_parameters_.ax_min = this->get_parameter("ax_min").as_double();
  nlmpc_parameters_.ax_max = this->get_parameter("ax_max").as_double();

  trajectory_parameters_.wheelbase = nlmpc_parameters_.wheelbase;
  trajectory_parameters_.sample_time = nlmpc_parameters_.sample_time;
  trajectory_parameters_.trajectory_generation_mode = this->get_parameter("trajectory_generation_mode").as_string();
  trajectory_parameters_.number_of_waypoints = this->get_parameter("number_of_waypoints").as_int();
  trajectory_parameters_.segment_times = this->get_parameter("segment_times").as_double_array();
  trajectory_parameters_.default_speed = this->get_parameter("default_speed").as_double();
  trajectory_parameters_.minimum_segment_time = this->get_parameter("minimum_segment_time").as_double();
  trajectory_parameters_.intermediate_tangent_scale = this->get_parameter("intermediate_tangent_scale").as_double();
  trajectory_parameters_.minimum_goal_distance = this->get_parameter("minimum_goal_distance").as_double();
  trajectory_parameters_.zero_endpoint_steering = this->get_parameter("zero_endpoint_steering").as_bool();
}

void QCar2NLMPCController::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  if (msg->velocity.empty()) {
    return;
  }

  velocity_feedback_ = msg->velocity[0] * kRawMotorSpeedToLinearVelocity;
  last_velocity_feedback_time_ = msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0 ?
    this->get_clock()->now() : rclcpp::Time(msg->header.stamp);
  has_velocity_feedback_ = true;
}

void QCar2NLMPCController::goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  publishZeroCommand();
  mode_ = ControllerMode::WaitingForGoal;

  if (!isGoalPoseValid(*msg)) {
    RCLCPP_WARN(this->get_logger(), "Received invalid /goal_pose. The car will remain stopped.");
    return;
  }

  VehicleState current_state;
  if (!readCurrentVehicleState(current_state)) {
    RCLCPP_WARN(this->get_logger(), "Cannot create trajectory because current state is unavailable.");
    publishZeroCommand();
    return;
  }

  const Pose2D start_pose{current_state.x, current_state.y, current_state.psi};
  const Pose2D goal_pose{
    msg->pose.position.x,
    msg->pose.position.y,
    yawFromQuaternion(
      msg->pose.orientation.x,
      msg->pose.orientation.y,
      msg->pose.orientation.z,
      msg->pose.orientation.w)};

  try {
    active_trajectory_ = trajectory_generator_->generateTrajectory(start_pose, goal_pose);
  } catch (const std::exception & exception) {
    RCLCPP_ERROR(this->get_logger(), "Trajectory generation failed: %s", exception.what());
    publishZeroCommand();
    return;
  }

  active_goal_pose_ = goal_pose;
  longitudinal_velocity_command_ = clip(current_state.vx, nlmpc_parameters_.vx_min, nlmpc_parameters_.vx_max);
  last_control_input_ = {0.0, 0.0};
  tracking_start_time_ = this->get_clock()->now();
  mode_ = ControllerMode::Tracking;

  publishReferencePath();
  RCLCPP_INFO(
    this->get_logger(),
    "New NLMPC reference is ready: %zu samples, %.3f s duration. Tracking starts now.",
    active_trajectory_.samples.size(),
    active_trajectory_.duration());
}

void QCar2NLMPCController::controlTimerCallback()
{
  if (mode_ != ControllerMode::Tracking) {
    publishZeroCommand();
    return;
  }

  VehicleState current_state;
  if (!readCurrentVehicleState(current_state)) {
    publishZeroCommand();
    return;
  }

  const double goal_distance = distanceToGoal(current_state, active_goal_pose_);
  if (goal_distance <= goal_position_tolerance_) {
    stopTracking("goal tolerance reached");
    return;
  }

  const double elapsed_time = (this->get_clock()->now() - tracking_start_time_).seconds();
  std::size_t reference_index = static_cast<std::size_t>(std::max(0.0, std::round(elapsed_time / nlmpc_parameters_.sample_time)));
  if (active_trajectory_.empty() || reference_index >= active_trajectory_.samples.size() - 1) {
    stopTracking("end of reference trajectory reached");
    return;
  }

  if (!solver_->isAvailable()) {
    publishZeroCommand();
    return;
  }

  std::vector<std::array<double, 4>> state_reference_horizon;
  std::vector<std::array<double, 2>> input_reference_horizon;
  buildReferenceHorizon(reference_index, state_reference_horizon, input_reference_horizon);

  const std::array<double, 4> current_state_array{{
    current_state.x,
    current_state.y,
    current_state.psi,
    current_state.vx}};

  const SolverResult solver_result = solver_->solve(
    current_state_array,
    state_reference_horizon,
    input_reference_horizon,
    last_control_input_);

  if (!solver_result.success) {
    RCLCPP_ERROR_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      1000,
      "acados solver failed with status %d. Publishing zero command.",
      solver_result.status);
    publishZeroCommand();
    if (stop_on_solver_failure_) {
      mode_ = ControllerMode::WaitingForGoal;
    }
    return;
  }

  // Packaging 2 optimal control input, also clip the over-limite values
  const double steering_angle = clip(solver_result.delta, nlmpc_parameters_.delta_min, nlmpc_parameters_.delta_max);
  const double longitudinal_acceleration = clip(solver_result.ax, nlmpc_parameters_.ax_min, nlmpc_parameters_.ax_max);
  longitudinal_velocity_command_ = clip(
    longitudinal_velocity_command_ + longitudinal_acceleration * nlmpc_parameters_.sample_time,
    nlmpc_parameters_.vx_min,
    nlmpc_parameters_.vx_max);

  publishMotorCommand(steering_angle, longitudinal_velocity_command_);
  last_control_input_ = {steering_angle, longitudinal_acceleration};
}

bool QCar2NLMPCController::readCurrentVehicleState(VehicleState & state)
{
  geometry_msgs::msg::TransformStamped transform;
  try {
    (void)transform_timeout_;
    transform = tf_buffer_->lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
  } catch (const tf2::TransformException & exception) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "Waiting for transform %s -> %s: %s", map_frame_.c_str(), base_frame_.c_str(), exception.what());
    return false;
  }

  if (!has_velocity_feedback_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Waiting for qcar2_joint velocity feedback.");
    return false;
  }

  const double feedback_age = (this->get_clock()->now() - last_velocity_feedback_time_).seconds();
  if (feedback_age > feedback_timeout_) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "Velocity feedback timeout: %.3f s > %.3f s.", feedback_age, feedback_timeout_);
    return false;
  }

  state.x = transform.transform.translation.x;
  state.y = transform.transform.translation.y;
  state.psi = yawFromQuaternion(
    transform.transform.rotation.x,
    transform.transform.rotation.y,
    transform.transform.rotation.z,
    transform.transform.rotation.w);
  state.vx = velocity_feedback_;

  return isFinite(state.x) && isFinite(state.y) && isFinite(state.psi) && isFinite(state.vx);
}

void QCar2NLMPCController::buildReferenceHorizon(
  std::size_t start_index,
  std::vector<std::array<double, 4>> & state_reference_horizon,
  std::vector<std::array<double, 2>> & input_reference_horizon) const
{
  state_reference_horizon.clear();
  input_reference_horizon.clear();
  state_reference_horizon.reserve(static_cast<std::size_t>(nlmpc_parameters_.horizon_steps + 1));
  input_reference_horizon.reserve(static_cast<std::size_t>(nlmpc_parameters_.horizon_steps));

  for (int offset = 0; offset <= nlmpc_parameters_.horizon_steps; ++offset) {
    const std::size_t index = std::min(start_index + static_cast<std::size_t>(offset), active_trajectory_.samples.size() - 1);
    const auto & sample = active_trajectory_.samples[index];
    state_reference_horizon.push_back({sample.x, sample.y, sample.psi, sample.vx});
  }

  for (int offset = 0; offset < nlmpc_parameters_.horizon_steps; ++offset) {
    const std::size_t index = std::min(start_index + static_cast<std::size_t>(offset), active_trajectory_.samples.size() - 1);
    const auto & sample = active_trajectory_.samples[index];
    input_reference_horizon.push_back({sample.delta, sample.ax});
  }
}

void QCar2NLMPCController::publishReferencePath()
{
  nav_msgs::msg::Path path_msg;
  path_msg.header.stamp = this->get_clock()->now();
  path_msg.header.frame_id = map_frame_;

  path_msg.poses.reserve(active_trajectory_.samples.size());
  for (const auto & sample : active_trajectory_.samples) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path_msg.header;
    pose.pose.position.x = sample.x;
    pose.pose.position.y = sample.y;
    pose.pose.position.z = 0.0;
    pose.pose.orientation = quaternionFromYaw(sample.psi);
    path_msg.poses.push_back(pose);
  }

  reference_path_publisher_->publish(path_msg);
}

void QCar2NLMPCController::publishZeroCommand()
{
  longitudinal_velocity_command_ = 0.0;
  publishMotorCommand(0.0, 0.0);
}

void QCar2NLMPCController::publishMotorCommand(double steering_angle, double longitudinal_velocity_command)
{
  qcar2_interfaces::msg::MotorCommands command;
  command.motor_names = {"steering_angle", "motor_throttle"};
  command.values = {steering_angle, longitudinal_velocity_command};
  command_publisher_->publish(command);
}

void QCar2NLMPCController::stopTracking(const std::string & reason)
{
  publishZeroCommand();
  mode_ = ControllerMode::WaitingForGoal;
  RCLCPP_INFO(this->get_logger(), "NLMPC tracking stopped: %s. Waiting for a new 2D Goal.", reason.c_str());
}

bool QCar2NLMPCController::isGoalPoseValid(const geometry_msgs::msg::PoseStamped & goal_pose) const
{
  const double yaw = yawFromQuaternion(
    goal_pose.pose.orientation.x,
    goal_pose.pose.orientation.y,
    goal_pose.pose.orientation.z,
    goal_pose.pose.orientation.w);

  return isFinite(goal_pose.pose.position.x) &&
         isFinite(goal_pose.pose.position.y) &&
         isFinite(goal_pose.pose.orientation.x) &&
         isFinite(goal_pose.pose.orientation.y) &&
         isFinite(goal_pose.pose.orientation.z) &&
         isFinite(goal_pose.pose.orientation.w) &&
         isFinite(yaw);
}

double QCar2NLMPCController::yawFromQuaternion(double x, double y, double z, double w)
{
  const double siny_cosp = 2.0 * (w * z + x * y);
  const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
  return std::atan2(siny_cosp, cosy_cosp);
}

geometry_msgs::msg::Quaternion QCar2NLMPCController::quaternionFromYaw(double yaw)
{
  geometry_msgs::msg::Quaternion quaternion;
  quaternion.x = 0.0;
  quaternion.y = 0.0;
  quaternion.z = std::sin(0.5 * yaw);
  quaternion.w = std::cos(0.5 * yaw);
  return quaternion;
}

double QCar2NLMPCController::clip(double value, double lower_bound, double upper_bound)
{
  return std::min(std::max(value, lower_bound), upper_bound);
}

}  // namespace qcar2_nlmpc

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<qcar2_nlmpc::QCar2NLMPCController>());
  rclcpp::shutdown();
  return 0;
}
