#include <ros/ros.h>
#include <mav_msgs/Actuators.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/LU>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include "prehover_gain_schedule.hpp"

// Isolated controller experiment. The original GitHub package stays untouched.
// Model, frame convention, X allocation and signed rotor speeds match LAB4.
class SprintController {
 public:
  SprintController() : pnh_("~") {
    parameter("kx", kx_, 25.0);
    parameter("kv", kv_, 10.0);
    parameter("kr", kr_, 60.0);
    parameter("komega", kw_, 8.0);
    parameter("control_hz", hz_, 1000.0);
    parameter("Jx", jx_, 1.0);
    parameter("Jy", jy_, 1.0);
    parameter("Jz", jz_, 1.0);
    pnh_.param("use_rate_feedforward", use_rate_ff_, true);
    pnh_.param("use_accel_feedforward", use_accel_ff_, false);
    pnh_.param("feedforward_reference_only", reference_only_ff_, true);
    pnh_.param("log_attitude_error", log_attitude_error_, false);
    pnh_.param("free_heading", free_heading_, false);
    pnh_.param("use_attitude_preview", use_attitude_preview_, false);
    pnh_.param("signed_attitude", signed_attitude_, false);
    pnh_.param("signed_attitude_hysteresis", branch_hysteresis_, 0.2);
    pnh_.param("rate_feedforward_gain", rate_ff_gain_, 1.0);
    pnh_.param("accel_feedforward_gain", accel_ff_gain_, 1.0);
    pnh_.param("omega_d_limit", omega_limit_, 6.0);
    pnh_.param("omega_dot_d_limit", alpha_limit_, 40.0);
    pnh_.param("omega_d_filter_tau", rate_filter_tau_, 0.02);
    pnh_.param("omega_dot_d_filter_tau", accel_filter_tau_, 0.04);
    pnh_.param("derivative_min_dt", min_dt_, 0.0005);
    pnh_.param("derivative_reset_dt", reset_dt_, 0.2);
    pnh_.param("prehover_kr", prehover_kr_, 12.0);
    pnh_.param("prehover_komega", prehover_kw_, 1.0);
    pnh_.param("prehover_kx", prehover_kx_, 4.0);
    pnh_.param("prehover_kv", prehover_kv_, 3.0);
    pnh_.param("hover_release_t", hover_release_t_, 5.0);
    pnh_.param("gain_transition_s", gain_transition_s_, 0.3);
    pnh_.param("race_end_plan_s", race_end_plan_s_, 60.0);
    pnh_.param("control_on_state", control_on_state_, false);
    pnh_.param("state_wait_ms", state_wait_ms_, 8.0);
    if (!validParameters()) {
      ROS_FATAL("Invalid controller_sprint parameters");
      throw std::runtime_error("Invalid controller_sprint parameters");
    }

    // Unchanged course constants. Optional J parameters change this
    // controller's model only, never the simulator's physical inertia.
    inertia_ = Eigen::Vector3d(jx_, jy_, jz_).asDiagonal();
    const double cf = 1e-3, cd = 1e-5, d = 0.3;
    const double arm = cf * d / std::sqrt(2.0);
    Eigen::Matrix4d allocation;
    allocation << cf, cf, cf, cf,
                  arm, arm, -arm, -arm,
                  -arm, arm, arm, -arm,
                  cd, -cd, cd, -cd;
    inverse_allocation_ = allocation.inverse();
    desired_sub_ = nh_.subscribe("desired_state", 1,
                                &SprintController::desiredState, this);
    current_sub_ = nh_.subscribe("current_state", 1,
                                &SprintController::currentState, this);
    if (use_attitude_preview_)
      preview_sub_ = nh_.subscribe("/sprint_acceleration_preview", 1,
                                   &SprintController::accelerationPreview, this);
    rotor_pub_ = nh_.advertise<mav_msgs::Actuators>("rotor_speed_cmds", 1);
    schedule_pub_ = pnh_.advertise<geometry_msgs::Vector3Stamped>("gain_schedule", 10);
    position_schedule_pub_ = pnh_.advertise<geometry_msgs::Vector3Stamped>("pos_gain_schedule", 10);
    timer_ = nh_.createWallTimer(ros::WallDuration(1.0 / hz_),
                                &SprintController::control, this);
    ROS_INFO("controller_sprint kx=%g kv=%g kr=%g komega=%g wall_hz=%g "
             "rate_ff=%d accel_ff=%d omega_limit=%g alpha_limit=%g "
             "J=(%g,%g,%g) signed_attitude=%d reference_only_ff=%d log_error=%d free_heading=%d attitude_preview=%d control_on_state=%d state_wait_ms=%g",
             kx_, kv_, kr_, kw_, hz_, use_rate_ff_, use_accel_ff_,
             omega_limit_, alpha_limit_, jx_, jy_, jz_, signed_attitude_,
             reference_only_ff_, log_attitude_error_, free_heading_, use_attitude_preview_, control_on_state_, state_wait_ms_);
  }

 private:
  void parameter(const char* name, double& value, double fallback) {
    // Preserve launch compatibility with the course's global gains;
    // explicit private parameters take priority over the global values.
    nh_.param(name, value, fallback);
    pnh_.param(name, value, value);
  }

  bool validParameters() const {
    const double positive[] = {kx_, kv_, kr_, kw_, hz_, jx_, jy_, jz_, omega_limit_,
                               alpha_limit_, min_dt_, reset_dt_};
    for (double v : positive)
      if (!std::isfinite(v) || v <= 0) return false;
    const double nonnegative[] = {rate_filter_tau_, accel_filter_tau_,
                                  rate_ff_gain_, accel_ff_gain_, branch_hysteresis_};
    for (double v : nonnegative)
      if (!std::isfinite(v) || v < 0) return false;
    const double scheduled[] = {prehover_kr_, prehover_kw_, prehover_kx_, prehover_kv_, gain_transition_s_, race_end_plan_s_};
    for (double v : scheduled) if (!std::isfinite(v) || v <= 0) return false;
    if (!std::isfinite(hover_release_t_) || hover_release_t_ < 0 ||
        race_end_plan_s_ < hover_release_t_+2*gain_transition_s_) return false;
    if (!std::isfinite(state_wait_ms_) || state_wait_ms_ < 0 || state_wait_ms_ > 19) return false;
    return hz_ <= 10000 && reset_dt_ > min_dt_;
  }

  static Eigen::Vector3d vee(const Eigen::Matrix3d& m) {
    return Eigen::Vector3d(m(2, 1), m(0, 2), m(1, 0));
  }

  static Eigen::Vector3d rotationLog(const Eigen::Matrix3d& rotation) {
    Eigen::Quaterniond q(rotation);
    q.normalize();
    if (q.w() < 0) q.coeffs() *= -1;
    const Eigen::AngleAxisd aa(q);
    return aa.axis() * aa.angle();
  }

  static Eigen::Vector3d bounded(const Eigen::Vector3d& v, double maximum) {
    const double n = v.norm();
    if (!v.allFinite() || !std::isfinite(n)) return Eigen::Vector3d::Zero();
    if (n > maximum) return (maximum / n) * v;
    return v;
  }

  static double filterWeight(double dt, double tau) {
    return tau > 0 ? -std::expm1(-dt / tau) : 1.0;
  }

  void desiredState(const trajectory_msgs::MultiDOFJointTrajectoryPoint& msg) {
    if (msg.transforms.empty() || msg.velocities.empty() ||
        msg.accelerations.empty()) return;
    const double plan_time = msg.time_from_start.toSec();
    if (!std::isfinite(plan_time) || plan_time < 0) return;
    const auto& t = msg.transforms[0];
    const auto& v = msg.velocities[0].linear;
    const auto& a = msg.accelerations[0].linear;
    Eigen::Vector3d xd(t.translation.x, t.translation.y, t.translation.z);
    Eigen::Vector3d vd(v.x, v.y, v.z), ad(a.x, a.y, a.z);
    Eigen::Quaterniond q(t.rotation.w, t.rotation.x,
                         t.rotation.y, t.rotation.z);
    const double qn = q.norm();
    if (!xd.allFinite() || !vd.allFinite() || !ad.allFinite() ||
        !q.coeffs().allFinite() || !std::isfinite(qn) || qn < 1e-9) return;
    q.normalize();
    const double yaw = std::atan2(2 * (q.w() * q.z() + q.x() * q.y()),
                                 1 - 2 * (q.y() * q.y() + q.z() * q.z()));
    if (!std::isfinite(yaw)) return;
    xd_ = xd;
    vd_ = vd;
    ad_ = ad;
    yawd_ = yaw;
    desired_plan_time_ = plan_time;
    got_desired_ = true;
    // Synchronous mode: one command per simulator step, issued as soon as the
    // reference for the step that produced the pending state has arrived.
    if (control_on_state_ && state_pending_) {
      state_pending_ = false;
      step();
    }
  }

  void currentState(const nav_msgs::Odometry& msg) {
    const auto& p = msg.pose.pose.position;
    const auto& v = msg.twist.twist.linear;
    const auto& w = msg.twist.twist.angular;
    const auto& o = msg.pose.pose.orientation;
    Eigen::Vector3d x(p.x, p.y, p.z), velocity(v.x, v.y, v.z);
    Eigen::Vector3d world_omega(w.x, w.y, w.z);
    Eigen::Quaterniond q(o.w, o.x, o.y, o.z);
    const double qn = q.norm();
    if (!x.allFinite() || !velocity.allFinite() || !world_omega.allFinite() ||
        !q.coeffs().allFinite() || !std::isfinite(qn) || qn < 1e-9) return;
    q.normalize();
    x_ = x;
    v_ = velocity;
    R_ = q.toRotationMatrix();  // body -> world, as in the GitHub controller.
    omega_ = R_.transpose() * world_omega;
    got_current_ = true;
    if (control_on_state_) {
      state_pending_ = true;
      state_wall_ = ros::WallTime::now();
    }
  }

  static bool freshPreview(const ros::Time& stamp, const ros::Time& now) {
    const double age = (now - stamp).toSec();
    return std::isfinite(age) && age >= -1e-6 && age <= 0.1;
  }

  void accelerationPreview(const geometry_msgs::Vector3Stamped& msg) {
    const Eigen::Vector3d acceleration(msg.vector.x, msg.vector.y, msg.vector.z);
    if (!acceleration.allFinite() ||
        !freshPreview(msg.header.stamp, ros::Time::now())) return;
    // This is a future planned acceleration in world axes, not an observed
    // acceleration and not a replacement for the current translational ad.
    preview_acceleration_ = acceleration;
    preview_stamp_ = msg.header.stamp;
    got_preview_ = true;
  }

  bool rotationForForce(const Eigen::Vector3d& force, Eigen::Matrix3d* rd) {
    if (!force.allFinite() || force.norm() < 1e-6) return false;
    const Eigen::Vector3d b3 = force.normalized();
    Eigen::Vector3d heading(std::cos(yawd_), std::sin(yawd_), 0);
    Eigen::Vector3d cross = b3.cross(heading);
    if (cross.norm() < 1e-4) {
      // At the heading/thrust singularity, retain the previous horizontal
      // direction when possible instead of normalizing a zero vector.
      if (derivative_ready_) heading = previous_rd_.col(0);
      else heading = Eigen::Vector3d::UnitX();
      cross = b3.cross(heading);
      if (cross.norm() < 1e-4) {
        Eigen::Vector3d alternative = Eigen::Vector3d::UnitY();
        if (std::abs(b3.x()) < 0.8) alternative = Eigen::Vector3d::UnitX();
        cross = b3.cross(alternative);
      }
    }
    const Eigen::Vector3d b2 = cross.normalized();
    rd->col(0) = b2.cross(b3);
    rd->col(1) = b2;
    rd->col(2) = b3;
    return rd->allFinite();
  }

  static double rotationDistance(const Eigen::Matrix3d& a,
                                 const Eigen::Matrix3d& b) {
    const double cosine = ((a.transpose() * b).trace() - 1.0) * 0.5;
    return std::acos(std::max(-1.0, std::min(1.0, cosine)));
  }

  static bool parallelTransport(const Eigen::Vector3d& force,
                                const Eigen::Matrix3d& frame,
                                Eigen::Matrix3d* result) {
    const double magnitude = force.norm();
    if (!force.allFinite() || !std::isfinite(magnitude) || magnitude < 1e-6)
      return false;
    const Eigen::Vector3d target = force / magnitude;
    Eigen::Quaterniond rotation =
        Eigen::Quaterniond::FromTwoVectors(frame.col(2), target);
    rotation.normalize();
    *result = rotation.toRotationMatrix() * frame;
    return result->allFinite();
  }

  bool feedbackRotationForForce(const Eigen::Vector3d& force,
                                Eigen::Matrix3d* result) {
    if (free_heading_) return parallelTransport(force, R_, result);
    return rotationForForce(force, result);
  }

  bool desiredRotation(const Eigen::Vector3d& force, Eigen::Matrix3d* rd) {
    Eigen::Matrix3d positive, negative;
    if (!feedbackRotationForForce(force, &positive)) return false;
    if (!signed_attitude_) {
      *rd = positive;
      return true;
    }
    if (!feedbackRotationForForce(-force, &negative)) return false;
    // Signed thrust is already supported by the official actuator model.
    // Choose between the two valid body-z directions without changing F,
    // gravity, gates or the wrench-to-rotor map. Hysteresis avoids switching
    // repeatedly when both orientations are nearly equally far from R.
    const double positive_cost = rotationDistance(R_, positive);
    const double negative_cost = rotationDistance(R_, negative);
    int selected = branch_sign_;
    if (!branch_initialized_)
      selected = positive_cost <= negative_cost ? 1 : -1;
    else if (branch_sign_ > 0 && negative_cost + branch_hysteresis_ < positive_cost)
      selected = -1;
    else if (branch_sign_ < 0 && positive_cost + branch_hysteresis_ < negative_cost)
      selected = 1;
    if (branch_initialized_ && selected != branch_sign_) {
      // A deliberate pi branch jump must not create a numerical rate impulse.
      derivative_ready_ = false;
      ROS_INFO_THROTTLE(0.5, "controller_sprint selected signed thrust branch %d", selected);
    }
    branch_sign_ = selected;
    branch_initialized_ = true;
    if (selected > 0) *rd = positive;
    else *rd = negative;
    return true;
  }

  void updateDesiredRates(const Eigen::Matrix3d& rd, const ros::Time& now) {
    const double dt = derivative_ready_ ? (now - previous_time_).toSec() : 0;
    if (!derivative_ready_ || dt < 0 || dt > reset_dt_) {
      previous_rd_ = rd;
      previous_time_ = now;
      omega_d_.setZero();
      omega_dot_d_.setZero();
      derivative_ready_ = true;
      return;
    }
    // A 1 kHz wall timer sees repeated /clock values. Hold the derivatives
    // between distinct simulator timestamps; never divide by wall-clock dt.
    if (dt < min_dt_) return;
    Eigen::Quaterniond increment(previous_rd_.transpose() * rd);
    increment.normalize();
    if (increment.w() < 0) increment.coeffs() *= -1;
    const Eigen::AngleAxisd aa(increment);
    const Eigen::Vector3d raw = bounded(aa.axis() * (aa.angle() / dt),
                                         omega_limit_);
    const Eigen::Matrix3d transport = rd.transpose() * previous_rd_;
    const Eigen::Vector3d old_rate = transport * omega_d_;
    const double rate_weight = filterWeight(dt, rate_filter_tau_);
    const Eigen::Vector3d next_rate =
        bounded(old_rate + rate_weight * (raw - old_rate), omega_limit_);
    const Eigen::Vector3d raw_alpha =
        bounded((next_rate - old_rate) / dt, alpha_limit_);
    const Eigen::Vector3d old_alpha = transport * omega_dot_d_;
    const double accel_weight = filterWeight(dt, accel_filter_tau_);
    omega_dot_d_ = bounded(old_alpha + accel_weight * (raw_alpha - old_alpha),
                            alpha_limit_);
    omega_d_ = next_rate;
    previous_rd_ = rd;
    previous_time_ = now;
  }

  void control(const ros::WallTimerEvent&) {
    if (control_on_state_) {
      // Watchdog: if the reference for this step has not arrived in time,
      // command from the latest available reference rather than skip the step.
      if (state_pending_ &&
          (ros::WallTime::now() - state_wall_).toSec() * 1000.0 >= state_wait_ms_) {
        state_pending_ = false;
        step();
      }
      return;
    }
    step();
  }

  void step() {
    if (!got_current_ || !got_desired_) return;
    const Eigen::Vector3d e3 = Eigen::Vector3d::UnitZ();
    const lab4_prehover::Gains gains = lab4_prehover::scheduledGains(
        desired_plan_time_, hover_release_t_, gain_transition_s_, race_end_plan_s_,
        prehover_kr_, prehover_kw_, kr_, kw_);
    const lab4_prehover::PositionGains position_gains = lab4_prehover::scheduledPositionGains(
        gains.race_weight, prehover_kx_, prehover_kv_, kx_, kv_);
    const Eigen::Vector3d force =
        -position_gains.kx * (x_ - xd_) - position_gains.kv * (v_ - vd_) + mass_ * gravity_ * e3 + mass_ * ad_;
    Eigen::Vector3d attitude_acceleration = ad_;
    if (use_attitude_preview_ && got_preview_ &&
        freshPreview(preview_stamp_, ros::Time::now()))
      attitude_acceleration = preview_acceleration_;
    const Eigen::Vector3d attitude_force =
        force + mass_ * (attitude_acceleration - ad_);
    Eigen::Matrix3d rd;
    if (!desiredRotation(attitude_force, &rd)) return;
    Eigen::Matrix3d rate_rotation = rd;
    bool valid_rate_rotation = true;
    if (reference_only_ff_) {
      // Differentiate only the nominal reference. Differencing the PD force
      // would feed sampled position/velocity errors back into the rate loop.
      const double branch = signed_attitude_ ? branch_sign_ : 1.0;
      const Eigen::Vector3d nominal_force =
          branch * mass_ * (gravity_ * e3 + attitude_acceleration);
      if (free_heading_) {
        // Initialize the free nominal heading once, then transport only its
        // own previous reference frame. Current-state changes therefore do
        // not enter the numerical feedforward derivative.
        if (!ff_frame_ready_) {
          ff_frame_ = R_;
          ff_frame_ready_ = true;
        }
        valid_rate_rotation = parallelTransport(nominal_force, ff_frame_,
                                                &rate_rotation);
        if (valid_rate_rotation) ff_frame_ = rate_rotation;
      } else {
        valid_rate_rotation = rotationForForce(nominal_force, &rate_rotation);
      }
    }
    if (valid_rate_rotation)
      updateDesiredRates(rate_rotation, ros::Time::now());
    else {
      // A zero nominal force has no defined attitude: retain PD control but
      // suppress its undefined feedforward, rather than reverting to PD-rate
      // differencing while reference-only mode is selected.
      derivative_ready_ = false;
      omega_d_.setZero();
      omega_dot_d_.setZero();
    }
    Eigen::Vector3d eR;
    if (log_attitude_error_)
      eR = rotationLog(rd.transpose() * R_);
    else
      eR = 0.5 * vee(rd.transpose() * R_ - R_.transpose() * rd);
    // previous_rd_ now denotes the held differentiation frame (Rff when
    // reference_only_ff is true). R^T Rff_hold maps its rate into body axes.
    const Eigen::Matrix3d rate_to_body = R_.transpose() * previous_rd_;
    Eigen::Vector3d wd = Eigen::Vector3d::Zero();
    Eigen::Vector3d ad = Eigen::Vector3d::Zero();
    if (use_rate_ff_ && valid_rate_rotation)
      wd = bounded(rate_ff_gain_ * omega_d_, omega_limit_);
    if (use_rate_ff_ && use_accel_ff_ && valid_rate_rotation)
      ad = bounded(accel_ff_gain_ * omega_dot_d_, alpha_limit_);
    const Eigen::Vector3d reference_rate = rate_to_body * wd;
    const Eigen::Vector3d reference_acceleration = rate_to_body * ad;
    const Eigen::Vector3d eomega = omega_ - reference_rate;
    const double thrust = force.dot(R_ * e3);
    // Lee-style feedforward structure with an independently selected rate
    // reference. Angular acceleration is opt-in because it is a second
    // numerical difference. Position/velocity feedback still sets force/Rd.
    const ros::Time gain_stamp = ros::Time::now();
    if (gain_stamp != previous_gain_stamp_) {
      geometry_msgs::Vector3Stamped sample;
      sample.header.stamp = gain_stamp;
      sample.header.frame_id = "kr_komega_race_weight";
      sample.vector.x = gains.kr; sample.vector.y = gains.komega; sample.vector.z = gains.race_weight;
      schedule_pub_.publish(sample);
      sample.header.frame_id = "kx_kv_race_weight";
      sample.vector.x = position_gains.kx; sample.vector.y = position_gains.kv;
      position_schedule_pub_.publish(sample);
      previous_gain_stamp_ = gain_stamp;
    }
    const Eigen::Vector3d torque =
        -gains.kr * eR - gains.komega * eomega + omega_.cross(inertia_ * omega_)
        - inertia_ * (omega_.cross(reference_rate) - reference_acceleration);
    const Eigen::Vector4d wrench(thrust, torque.x(), torque.y(), torque.z());
    const Eigen::Vector4d signed_squares = inverse_allocation_ * wrench;
    if (!signed_squares.allFinite()) return;
    mav_msgs::Actuators command;
    command.header.stamp = ros::Time::now();
    command.angular_velocities.resize(4);
    for (int i = 0; i < 4; ++i)
      command.angular_velocities[i] =
          std::copysign(std::sqrt(std::abs(signed_squares[i])), signed_squares[i]);
    rotor_pub_.publish(command);
  }

  ros::NodeHandle nh_, pnh_;
  ros::Subscriber desired_sub_, current_sub_, preview_sub_;
  ros::Publisher rotor_pub_;
  ros::Publisher schedule_pub_;
  ros::Publisher position_schedule_pub_;
  ros::Time previous_gain_stamp_;
  double prehover_kr_, prehover_kw_, hover_release_t_, gain_transition_s_, race_end_plan_s_;
  double prehover_kx_, prehover_kv_;
  double desired_plan_time_ = 0.0;
  ros::WallTimer timer_;
  double kx_, kv_, kr_, kw_, hz_, jx_, jy_, jz_;
  double rate_ff_gain_, accel_ff_gain_, omega_limit_, alpha_limit_;
  double rate_filter_tau_, accel_filter_tau_, min_dt_, reset_dt_;
  bool use_rate_ff_, use_accel_ff_;
  bool reference_only_ff_, log_attitude_error_;
  bool free_heading_;
  bool use_attitude_preview_;
  bool signed_attitude_;
  double branch_hysteresis_;
  bool branch_initialized_ = false;
  int branch_sign_ = 1;
  const double mass_ = 1.0, gravity_ = 9.81;
  Eigen::Matrix3d inertia_ = Eigen::Matrix3d::Identity();
  Eigen::Matrix4d inverse_allocation_;
  Eigen::Vector3d x_ = Eigen::Vector3d::Zero(), v_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d omega_ = Eigen::Vector3d::Zero();
  Eigen::Matrix3d R_ = Eigen::Matrix3d::Identity();
  Eigen::Vector3d xd_ = Eigen::Vector3d::Zero(), vd_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d ad_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d preview_acceleration_ = Eigen::Vector3d::Zero();
  ros::Time preview_stamp_;
  bool got_preview_ = false;
  double yawd_ = 0;
  bool got_current_ = false, got_desired_ = false, derivative_ready_ = false;
  bool control_on_state_ = false, state_pending_ = false;
  double state_wait_ms_ = 8.0;
  ros::WallTime state_wall_;
  ros::Time previous_time_;
  Eigen::Matrix3d previous_rd_ = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d ff_frame_ = Eigen::Matrix3d::Identity();
  bool ff_frame_ready_ = false;
  Eigen::Vector3d omega_d_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d omega_dot_d_ = Eigen::Vector3d::Zero();
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "controller_sprint");
  try {
    SprintController controller;
    ros::spin();
  } catch (const std::exception& e) {
    ROS_FATAL("controller_sprint: %s", e.what());
    return 1;
  }
  return 0;
}
