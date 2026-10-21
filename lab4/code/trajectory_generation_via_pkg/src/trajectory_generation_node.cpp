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
  // Extra position-only vertices after a gate, flat list [after_gate, x, y, z, ...]
  // (copy of the node for the main demo: keeps the gate 9 -> 10 turn off the wall)
  std::vector<double> via_points;

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

      if (i + 1 < n_gates) {
        // intermediate gate: only position / yaw are fixed
        mav_trajectory_generation::Vertex vertex(D), yaw_vertex(1);
        vertex.addConstraint(POSITION, position);
        yaw_vertex.addConstraint(ORIENTATION, gate_yaw);
        vertices.push_back(vertex);
        yaw_vertices.push_back(yaw_vertex);
        // via points between this gate and the next one
        for (size_t k = 0; k + 3 < via_points.size(); k += 4) {
          if (static_cast<size_t>(via_points[k]) != i) continue;
          double next_yaw = tf::getYaw(poseArray.poses[i + 1].orientation);
          next_yaw = gate_yaw + std::remainder(next_yaw - gate_yaw, 2.0 * M_PI);
          mav_trajectory_generation::Vertex via(D), via_yaw(1);
          via.addConstraint(POSITION, Eigen::Vector3d(via_points[k + 1],
                                                      via_points[k + 2],
                                                      via_points[k + 3]));
          via_yaw.addConstraint(ORIENTATION, 0.5 * (gate_yaw + next_yaw));
          vertices.push_back(via);
          yaw_vertices.push_back(via_yaw);
        }
      } else {
        // last gate: come to rest there
        end_position.makeStartOrEnd(position, SNAP);
        end_yaw.makeStartOrEnd(gate_yaw, SNAP);
        vertices.push_back(end_position);
        yaw_vertices.push_back(end_yaw);
      }
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
    ROS_INFO("v_max %.1f, a_max %.1f: trajectory takes %.2f s "
             "(peak speed %.2f m/s, peak acc %.2f m/s^2, peak tilt %.1f deg)",
             v_max, a_max, trajectory.getMaxTime(), peak_v, peak_a,
             peak_tilt * 180.0 / M_PI);
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
    pnh.getParam("via_points", via_points);
    desiredStateTimer = nh.createTimer(
        ros::Rate(publish_rate), &WaypointFollower::publishDesiredState, this);
    desiredStateTimer.start();
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "trajectory_generation_node");
  ros::NodeHandle nh;

  WaypointFollower waypointFollower(nh);

  ros::spin();
  return 0;
}
