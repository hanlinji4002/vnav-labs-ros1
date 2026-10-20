#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/String.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <sstream>
#include <iostream>

#include <math.h>
#include <algorithm>

#define STATIC_POSE 0

#define PI M_PI

#define TFOUTPUT 1

// Smooth take-off (param ~smooth_start, off by default): the reference starts at the spawn point,
// climbs to 2 m and spirals out onto the circle while the speed ramps up; the yaw starts at the
// spawn yaw (-166 deg), so the drone never has to jump to a reference 5 m away.
static const double T_RAMP = 8.0;                 // s, radius and speed ramp
static const double T_CLIMB = 3.0;                // s, 0.7 m -> 2 m
static const double THETA0 = 166.0 * PI / 180.0;  // start phase, yaw = -phase

// quintic smoothstep (velocity and acceleration continuous at both ends) and its integral
static double ramp(double s) { s = std::min(std::max(s, 0.0), 1.0); return s * s * s * (10.0 - 15.0 * s + 6.0 * s * s); }
static double rampIntegral(double s) { s = std::min(std::max(s, 0.0), 1.0); return s * s * s * s * (2.5 - 3.0 * s + s * s); }

// circle phase; its rate ramps from 0 to 1/timeScale
static double smoothPhase(double t, double timeScale) {
    if (t < T_RAMP) return THETA0 + T_RAMP * rampIntegral(t / T_RAMP) / timeScale;
    return THETA0 + (0.5 * T_RAMP + (t - T_RAMP)) / timeScale;
}

static tf::Vector3 smoothPosition(double t, double R, double timeScale) {
    double th = smoothPhase(t, timeScale), r = R * ramp(t / T_RAMP);
    return tf::Vector3(r * sin(th), r * cos(th), 0.7 + 1.3 * ramp(t / T_CLIMB));
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "traj_publisher");
    ros::NodeHandle n;
    ros::Publisher desired_state_pub = n.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>("desired_state", 1);
    ros::Rate loop_rate(500);
    bool smooth_start = false;
    ros::NodeHandle("~").param("smooth_start", smooth_start, false);
    if (smooth_start) ros::Time::waitForValid();   // sim time: wait for /clock so t starts at 0
    ros::Time start(ros::Time::now());

#if TFOUTPUT
    tf::TransformBroadcaster br;
#endif

    int count = 0;
    while (ros::ok()) {
        tf::Vector3 origin(0,0,0);

        double t = (ros::Time::now()-start).toSec();

        // Quantities to fill in
        tf::Transform desired_pose(tf::Transform::getIdentity());

        geometry_msgs::Twist velocity;
        velocity.linear.x = velocity.linear.y = velocity.linear.z = 0;
        velocity.angular.x = velocity.angular.y = velocity.angular.z = 0;
        geometry_msgs::Twist acceleration;
        acceleration.linear.x = acceleration.linear.y = acceleration.linear.z = 0;
        acceleration.angular.x = acceleration.angular.y = acceleration.angular.z = 0;

#if STATIC_POSE
        // Static Pose
        tf::Vector3 displacement(0,0,2);
        desired_pose.setOrigin(origin+displacement);
        tf::Quaternion q;
        q.setRPY(0,0,PI/4);
        count++;
        std::cout<<"Desired Orientation" << count << std::endl;
        desired_pose.setRotation(q);

#else
        // Circle
        double R = 5.0;
        double timeScale = 2.0;
        if (smooth_start) {
            const double h = 1e-3;   // velocity and acceleration by central differences
            tf::Vector3 p = smoothPosition(t, R, timeScale);
            tf::Vector3 pm = smoothPosition(t - h, R, timeScale), pp = smoothPosition(t + h, R, timeScale);
            desired_pose.setOrigin(origin + p);
            tf::Quaternion q;
            q.setRPY(0, 0, -smoothPhase(t, timeScale));
            desired_pose.setRotation(q);
            tf::Vector3 vel = (pp - pm) / (2 * h), acc = (pp - 2 * p + pm) / (h * h);
            velocity.linear.x = vel.x();
            velocity.linear.y = vel.y();
            velocity.linear.z = vel.z();
            velocity.angular.z = -(smoothPhase(t + h, timeScale) - smoothPhase(t - h, timeScale)) / (2 * h);
            acceleration.linear.x = acc.x();
            acceleration.linear.y = acc.y();
            acceleration.linear.z = acc.z();
        } else {
            desired_pose.setOrigin(
                            origin + tf::Vector3(R*sin(t/timeScale), R*cos(t/timeScale), 2)
                         );
            tf::Quaternion q;
            q.setRPY(0,0,-t/timeScale);
            desired_pose.setRotation(q);
            velocity.linear.x = R*cos(t/timeScale)/timeScale;
            velocity.linear.y = -R*sin(t/timeScale)/timeScale;
            velocity.linear.z = 0;

            velocity.angular.x = 0.0;
            velocity.angular.y = 0.0;
            velocity.angular.z = -1.0/timeScale;

            acceleration.linear.x = -R*sin(t/timeScale)/timeScale/timeScale;
            acceleration.linear.y = -R*cos(t/timeScale)/timeScale/timeScale;
            acceleration.linear.z = 0;
        }
#endif

        // Publish
        trajectory_msgs::MultiDOFJointTrajectoryPoint msg;
        msg.transforms.resize(1);
        msg.transforms[0].translation.x = desired_pose.getOrigin().x();
        msg.transforms[0].translation.y = desired_pose.getOrigin().y();
        msg.transforms[0].translation.z = desired_pose.getOrigin().z();
        msg.transforms[0].rotation.x = desired_pose.getRotation().getX();
        msg.transforms[0].rotation.y = desired_pose.getRotation().getY();
        msg.transforms[0].rotation.z = desired_pose.getRotation().getZ();
        msg.transforms[0].rotation.w = desired_pose.getRotation().getW();
        msg.velocities.resize(1);
        msg.velocities[0] = velocity;
        msg.accelerations.resize(1);
        msg.accelerations[0] = acceleration;
        desired_state_pub.publish(msg);

        std::stringstream ss;
        ss << "Trajectory Position"
           << " x:" << desired_pose.getOrigin().x()
           << " y:" << desired_pose.getOrigin().y()
           << " z:" << desired_pose.getOrigin().z();
        ROS_INFO("%s", ss.str().c_str());

#if TFOUTPUT
        br.sendTransform(tf::StampedTransform(desired_pose, ros::Time::now(),
                                              "world", "av-desired"));
#endif

        ros::spinOnce();

        loop_rate.sleep();
        ++count;
    }


    return 0;
}
