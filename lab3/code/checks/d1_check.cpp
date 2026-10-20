// D1 answers compiled and checked on ROS Noetic
#include <geometry_msgs/Quaternion.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>
#include <Eigen/Dense>
#include <cstdio>

int main() {
  geometry_msgs::Quaternion in_msg;            // yaw 0.7, pitch 0.2, roll -0.3
  tf2::Quaternion ref; ref.setRPY(-0.3, 0.2, 0.7);
  in_msg.x = ref.x(); in_msg.y = ref.y(); in_msg.z = ref.z(); in_msg.w = ref.w();

  // A1: geometry_msgs::Quaternion -> existing tf2::Quaternion
  geometry_msgs::Quaternion quat_msg_in = in_msg;
  tf2::Quaternion quat_tf;
  tf2::fromMsg(quat_msg_in, quat_tf);
  printf("A1 fromMsg   : %.6f %.6f %.6f %.6f\n", quat_tf.x(), quat_tf.y(), quat_tf.z(), quat_tf.w());

  // A2: tf2::Quaternion -> new geometry_msgs::Quaternion quat_msg
  geometry_msgs::Quaternion quat_msg = tf2::toMsg(quat_tf);
  printf("A2 toMsg     : %.6f %.6f %.6f %.6f\n", quat_msg.x, quat_msg.y, quat_msg.z, quat_msg.w);

  // A3: scalar part
  printf("A3 getW()    : %.6f (w() = %.6f)\n", quat_tf.getW(), quat_tf.w());

  // B1: yaw in one call
  tf2::Quaternion quat_t = quat_tf;
  printf("B1 getYaw    : %.6f (expected 0.7)\n", tf2::getYaw(quat_t));

  // B2: geometry_msgs::Quaternion -> Eigen 3x3
  Eigen::Quaterniond q_eig;
  tf2::fromMsg(quat_msg, q_eig);
  Eigen::Matrix3d R = q_eig.toRotationMatrix();
  tf2::Matrix3x3 R_ref(ref);
  double err = 0;
  for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j) err = std::max(err, std::abs(R(i, j) - R_ref[i][j]));
  printf("B2 Eigen R   : max |R - R_ref| = %.2e\n", err);
  return 0;
}
