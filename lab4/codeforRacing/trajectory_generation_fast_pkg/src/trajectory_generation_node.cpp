//
// Created by stewart on 9/20/20.
//

#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include <mav_trajectory_generation/polynomial_optimization_linear.h>
#include <mav_trajectory_generation/trajectory.h>

#include <tf/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <eigen3/Eigen/Dense>
#include <fstream>
#include <numeric>

class WaypointFollower {
  [[maybe_unused]] ros::Subscriber currentStateSub;
  [[maybe_unused]] ros::Subscriber poseArraySub;
  ros::Publisher desiredStatePub;

  // Current state
  Eigen::Vector3d x;  // current position of the UAV's c.o.m. in the world frame
  double yaw = 0;     // current yaw, used as the start of the yaw trajectory
  bool has_current_state = false;

  // Segment-time limits and publish rate (template values by default)
  double v_max = 8.0;
  double a_max = 4.0;
  double publish_rate = 5.0;

  // Optional (faster racing): segment times scaled to peak |acc| = a_lim and
  // peak |jerk| = j_lim (0 = off), and a run-out vertex after the last gate
  double a_lim = 0.0;
  double j_lim = 0.0;
  double down_lim = 0.0;  // max downward acceleration [m/s^2], 0 = off
  double runout = 0.0;
  std::string dump_file;  // optional: write the planned trajectory as CSV

  ros::Timer desiredStateTimer;

  ros::Time trajectoryStartTime;
  mav_trajectory_generation::Trajectory trajectory;
  mav_trajectory_generation::Trajectory yaw_trajectory;

  void onCurrentState(nav_msgs::Odometry const& cur_state) {
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    //  PART 1.1 |  16.485 - Fall 2021  - Lab 4 coding assignment (5 pts)
    // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~  ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
    // ~
    //
    //  Populate the variable x, which encodes the current world position of the
    //  UAV
    // ~~~~ begin solution

    x << cur_state.pose.pose.position.x, cur_state.pose.pose.position.y,
        cur_state.pose.pose.position.z;
    yaw = tf::getYaw(cur_state.pose.pose.orientation);
    has_current_state = true;

    // ~~~~ end solution
    // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
    // ~
    //                                 end part 1.1
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  }

  void generateOptimizedTrajectory(geometry_msgs::PoseArray const& poseArray) {
    if (poseArray.poses.size() < 1) {
      ROS_ERROR("Must have at least one pose to generate trajectory!");
      trajectory.clear();
      yaw_trajectory.clear();
      return;
    }

    if (!trajectory.empty()) return;

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    //  PART 1.2 |  16.485 - Fall 2021  - Lab 4 coding assignment (35 pts)
    // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~  ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
    // ~
    //
    //  We are using the mav_trajectory_generation library
    //  (https://github.com/ethz-asl/mav_trajectory_generation) to perform
    //  trajectory optimization given the waypoints (based on the position and
    //  orientation of the gates on the race course).
    //  We will be finding the trajectory for the position and the trajectory
    //  for the yaw in a decoupled manner.
    //  In this section:
    //  1. Fill in the correct number for D, the dimension we should apply to
    //  the solver to find the positional trajectory
    //  2. Correctly populate the Vertex::Vector structure below (vertices,
    //  yaw_vertices) using the position of the waypoints and the yaw of the
    //  waypoints respectively
    //
    //  Hints:
    //  1. Use vertex.addConstraint(POSITION, position) where position is of
    //  type Eigen::Vector3d to enforce a waypoint position.
    //  2. Use vertex.addConstraint(ORIENTATION, yaw) where yaw is a double
    //  to enforce a waypoint yaw.
    //  3. Remember angle wraps around 2 pi. Be careful!
    //  4. For the ending waypoint for position use .makeStartOrEnd as seen with
    //  the starting waypoint instead of .addConstraint as you would do for the
    //  other waypoints.
    //
    // ~~~~ begin solution

    // the start vertex needs the UAV state: wait for the first odometry
    if (!has_current_state) return;

    const int D = 3;  // dimension of each vertex in the trajectory
    mav_trajectory_generation::Vertex start_position(D), end_position(D);
    mav_trajectory_generation::Vertex::Vector vertices;
    mav_trajectory_generation::Vertex start_yaw(1), end_yaw(1);
    mav_trajectory_generation::Vertex::Vector yaw_vertices;

    // ============================================
    // Convert the pose array to a list of vertices
    // ============================================

    // Start at rest from the current position, keeping the current yaw
    using namespace mav_trajectory_generation::derivative_order;
    start_position.makeStartOrEnd(x, SNAP);
    vertices.push_back(start_position);
    start_yaw.makeStartOrEnd(yaw, SNAP);
    yaw_vertices.push_back(start_yaw);

    double last_yaw = yaw;
    const size_t n_gates = poseArray.poses.size();
    for (size_t i = 0; i < n_gates; ++i) {
      const geometry_msgs::Pose& gate = poseArray.poses[i];
      const Eigen::Vector3d position(
          gate.position.x, gate.position.y, gate.position.z);

      // unwrap: use the equivalent angle closest to the previous yaw
      double gate_yaw = tf::getYaw(gate.orientation);
      gate_yaw = last_yaw + std::remainder(gate_yaw - last_yaw, 2.0 * M_PI);
      last_yaw = gate_yaw;

      if (i + 1 < n_gates || runout > 0) {
        // intermediate gate: only position / yaw are fixed
        mav_trajectory_generation::Vertex vertex(D), yaw_vertex(1);
        vertex.addConstraint(POSITION, position);
        yaw_vertex.addConstraint(ORIENTATION, gate_yaw);
        vertices.push_back(vertex);
        yaw_vertices.push_back(yaw_vertex);
      } else {
        // last gate: come to rest there
        end_position.makeStartOrEnd(position, SNAP);
        end_yaw.makeStartOrEnd(gate_yaw, SNAP);
        vertices.push_back(end_position);
        yaw_vertices.push_back(end_yaw);
      }
    }
    if (runout > 0) {
      // optional: stop `runout` metres beyond the last gate, along its heading
      const geometry_msgs::Point& p = poseArray.poses.back().position;
      const Eigen::Vector3d stop =
          Eigen::Vector3d(p.x, p.y, p.z) +
          runout * Eigen::Vector3d(std::cos(last_yaw), std::sin(last_yaw), 0);
      end_position.makeStartOrEnd(stop, SNAP);
      end_yaw.makeStartOrEnd(last_yaw, SNAP);
      vertices.push_back(end_position);
      yaw_vertices.push_back(end_yaw);
    }

    // ~~~~ end solution
    // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
    // ~
    //                                 end part 1.2
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    // ============================================================
    // Estimate the time to complete each segment of the trajectory
    // ============================================================

    // HINT: play with these segment times and see if you can finish
    // the race course faster!
    std::vector<double> segment_times;
    segment_times = estimateSegmentTimes(vertices, v_max, a_max);
    if (a_lim > 0) scaleSegmentTimes(vertices, &segment_times);

    // =====================================================
    // Solve for the optimized trajectory (linear optimizer)
    // =====================================================
    // Position
    const int N = 10;
    mav_trajectory_generation::PolynomialOptimization<N> opt(D);
    opt.setupFromVertices(vertices, segment_times, SNAP);
    opt.solveLinear();
    opt.getTrajectory(&trajectory);

    // Yaw
    mav_trajectory_generation::PolynomialOptimization<N> yaw_opt(1);
    yaw_opt.setupFromVertices(yaw_vertices, segment_times, SNAP);
    yaw_opt.solveLinear();

    // ============================
    // Get the optimized trajectory
    // ============================
    mav_trajectory_generation::Segment::Vector segments;
    //        opt.getSegments(&segments); // Unnecessary?
    yaw_opt.getTrajectory(&yaw_trajectory);
    trajectoryStartTime = ros::Time::now();

    ROS_INFO("Generated optimizes trajectory from %d waypoints",
             vertices.size());
    // planned peaks; tilt = angle of the thrust direction a + g e3
    double peak_v = 0, peak_a = 0, peak_tilt = 0;
    for (double t = 0; t < trajectory.getMaxTime(); t += 0.01) {
      const Eigen::Vector3d a = trajectory.evaluate(t, ACCELERATION);
      const Eigen::Vector3d f = a + Eigen::Vector3d(0, 0, 9.81);
      peak_v = std::max(peak_v, trajectory.evaluate(t, VELOCITY).norm());
      peak_a = std::max(peak_a, a.norm());
      peak_tilt = std::max(peak_tilt, std::acos(f.z() / f.norm()));
    }
    ROS_INFO("v_max %.1f, a_max %.1f, a_lim %.1f, j_lim %.1f, down_lim %.1f, "
             "runout %.1f: trajectory takes %.2f s (peak speed %.2f m/s, peak "
             "acc %.2f m/s^2, peak tilt %.1f deg)",
             v_max, a_max, a_lim, j_lim, down_lim, runout, trajectory.getMaxTime(),
             peak_v, peak_a, peak_tilt * 180.0 / M_PI);
    std::string times;
    for (double t : segment_times) times += std::to_string(t).substr(0, 5) + " ";
    ROS_INFO("segment times: %s", times.c_str());
    if (!dump_file.empty()) {
      std::ofstream f(dump_file);
      f << "t,x,y,z,vx,vy,vz,ax,ay,az,yaw\n";
      for (double t = 0; t <= trajectory.getMaxTime(); t += 0.01) {
        const Eigen::Vector3d p = trajectory.evaluate(t, POSITION);
        const Eigen::Vector3d v = trajectory.evaluate(t, VELOCITY);
        const Eigen::Vector3d a = trajectory.evaluate(t, ACCELERATION);
        f << t << "," << p.x() << "," << p.y() << "," << p.z() << "," << v.x() << ","
          << v.y() << "," << v.z() << "," << a.x() << "," << a.y() << "," << a.z()
          << "," << yaw_trajectory.evaluate(t, ORIENTATION)[0] << "\n";
      }
    }
  }

  // Peak |acc| / a_lim and |jerk| / j_lim of each segment, expressed as the
  // factor r by which the segment would have to be stretched (a ~ 1/T^2,
  // jerk ~ 1/T^3)
  std::vector<double> stretchFactors(
      const mav_trajectory_generation::Vertex::Vector& vertices,
      const std::vector<double>& times) {
    using namespace mav_trajectory_generation::derivative_order;
    mav_trajectory_generation::PolynomialOptimization<10> opt(3);
    opt.setupFromVertices(vertices, times, SNAP);
    opt.solveLinear();
    mav_trajectory_generation::Segment::Vector segments;
    opt.getSegments(&segments);
    std::vector<double> r(segments.size());
    for (size_t j = 0; j < segments.size(); ++j) {
      double peak_a = 0, peak_j = 0, peak_down = 0;
      const double T = segments[j].getTime();
      for (int k = 0; k <= 50; ++k) {
        const Eigen::Vector3d a = segments[j].evaluate(T * k / 50, ACCELERATION);
        peak_a = std::max(peak_a, a.norm());
        peak_down = std::max(peak_down, -a.z());
        peak_j = std::max(peak_j, segments[j].evaluate(T * k / 50, JERK).norm());
      }
      r[j] = std::sqrt(peak_a / a_lim);
      if (j_lim > 0) r[j] = std::max(r[j], std::cbrt(peak_j / j_lim));
      // keep the thrust pointing up: downward acceleration well below g
      if (down_lim > 0) r[j] = std::max(r[j], std::sqrt(peak_down / down_lim));
      if (!std::isfinite(r[j])) r[j] = 1.0;
    }
    return r;
  }

  // Optional time allocation. With all limits homogeneous in time scaling,
  // max_j r_j(t) * sum(t) is the duration after scaling the times t so that the
  // most loaded point just meets its limit; minimise it over the ratios of the
  // segment times by coordinate search, then apply that scaling
  void scaleSegmentTimes(
      const mav_trajectory_generation::Vertex::Vector& vertices,
      std::vector<double>* times) {
    auto duration = [&](const std::vector<double>& t) {
      const std::vector<double> r = stretchFactors(vertices, t);
      return *std::max_element(r.begin(), r.end()) *
             std::accumulate(t.begin(), t.end(), 0.0);
    };
    std::vector<double> t = *times;
    double best = duration(t);
    const double start = best;
    int evaluations = 1;
    for (double step = 0.2; step > 0.005;) {
      bool improved = false;
      for (size_t j = 0; j < t.size(); ++j) {
        for (double f : {1.0 - step, 1.0 + step}) {
          std::vector<double> trial = t;
          trial[j] *= f;
          const double d = duration(trial);
          ++evaluations;
          if (d < best - 1e-6) {
            best = d;
            t = trial;
            improved = true;
            break;
          }
        }
      }
      if (!improved) step /= 2;
    }
    const std::vector<double> r = stretchFactors(vertices, t);
    const double s = *std::max_element(r.begin(), r.end());
    for (double& ti : t) ti *= s;
    *times = t;
    ROS_INFO("time allocation: %.2f s -> %.2f s (%d evaluations)", start, best,
             evaluations);
  }

  void publishDesiredState(ros::TimerEvent const& ev) {
    if (trajectory.empty()) return;

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    //  PART 1.3 |  16.485 - Fall 2021  - Lab 4 coding assignment (15 pts)
    // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~  ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
    // ~
    //
    //  Finally we get to send commands to our controller! First fill in
    //  properly the value for 'nex_point.time_from_start' and 'sampling_time'
    //  (hint: not 0) and after extracting the state information from our
    //  optimized trajectory, finish populating next_point.
    //
    // ~~~~ begin solution
    trajectory_msgs::MultiDOFJointTrajectoryPoint next_point;
    // time since the trajectory started; clamped so the UAV holds the last gate
    double sampling_time = (ros::Time::now() - trajectoryStartTime).toSec();
    if (sampling_time > trajectory.getMaxTime())
      sampling_time = trajectory.getMaxTime();
    next_point.time_from_start = ros::Duration(sampling_time);

    // Getting the desired state based on the optimized trajectory we found.
    using namespace mav_trajectory_generation::derivative_order;
    Eigen::Vector3d des_position = trajectory.evaluate(sampling_time, POSITION);
    Eigen::Vector3d des_velocity = trajectory.evaluate(sampling_time, VELOCITY);
    Eigen::Vector3d des_accel =
        trajectory.evaluate(sampling_time, ACCELERATION);
    Eigen::VectorXd des_orientation =
        yaw_trajectory.evaluate(sampling_time, ORIENTATION);
    ROS_INFO("Traversed %f percent of the trajectory.",
             sampling_time / trajectory.getMaxTime() * 100);

    const double des_yaw_rate =
        yaw_trajectory.evaluate(sampling_time, ANGULAR_VELOCITY)[0];

    // Populate next_point
    // pose: position + yaw-only orientation
    geometry_msgs::Transform transform;
    tf::vectorEigenToMsg(des_position, transform.translation);
    transform.rotation = tf::createQuaternionMsgFromYaw(des_orientation[0]);
    next_point.transforms.push_back(transform);

    // velocity (yaw rate in angular.z) and acceleration
    geometry_msgs::Twist velocity, acceleration;
    tf::vectorEigenToMsg(des_velocity, velocity.linear);
    velocity.angular.z = des_yaw_rate;
    tf::vectorEigenToMsg(des_accel, acceleration.linear);
    next_point.velocities.push_back(velocity);
    next_point.accelerations.push_back(acceleration);

    desiredStatePub.publish(next_point);

    // ~~~~ end solution
    // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
    // ~
    //                                 end part 1.3
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  }

 public:
  explicit WaypointFollower(ros::NodeHandle& nh) {
    currentStateSub = nh.subscribe(
        "/current_state", 1, &WaypointFollower::onCurrentState, this);
    poseArraySub = nh.subscribe("/desired_traj_vertices",
                                1,
                                &WaypointFollower::generateOptimizedTrajectory,
                                this);
    desiredStatePub =
        nh.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>(
            "/desired_state", 1);
    ros::NodeHandle pnh("~");
    pnh.param("v_max", v_max, v_max);
    pnh.param("a_max", a_max, a_max);
    pnh.param("publish_rate", publish_rate, publish_rate);
    pnh.param("a_lim", a_lim, a_lim);
    pnh.param("j_lim", j_lim, j_lim);
    pnh.param("down_lim", down_lim, down_lim);
    pnh.param("runout", runout, runout);
    pnh.param("dump_file", dump_file, dump_file);
    desiredStateTimer = nh.createTimer(
        ros::Rate(publish_rate), &WaypointFollower::publishDesiredState, this);
    desiredStateTimer.start();
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "trajectory_generation_fast_node");
  ros::NodeHandle nh;

  WaypointFollower waypointFollower(nh);

  ros::spin();
  return 0;
}
