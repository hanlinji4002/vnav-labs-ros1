/*
 * @file self_flow.cpp
 * Feel free to edit any part of this file!
 */
#include <memory>
#include <string>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

using namespace std;

/** imageCallback This function is called when a new image is published. */
void imageCallback(const sensor_msgs::ImageConstPtr &msg) {
  try {
    // Convert ROS msg type to OpenCV image type.
    cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    //  DELIVERABLE 8 | Optical Flow
    // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~  ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
    //
    // LK tracker estimates the optical flow for sparse points in the image.
    // Alternatively, dense approaches try to estimate the optical flow for the whole image.
    // Try to calculate your own optical flow using Farneback’s algorithm (see OpenCV documentation).
    //
    // ~~~~ begin solution
    cv::Mat gray;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    static cv::Mat prev_gray;
    static unsigned int num_frames = 0;
    if (!prev_gray.empty()) {
      cv::Mat flow;
      cv::calcOpticalFlowFarneback(prev_gray, gray, flow, 0.5, 3, 15, 3, 5, 1.2, 0);

      // Direction -> hue, magnitude -> brightness
      cv::Mat flow_xy[2], magnitude, angle;
      cv::split(flow, flow_xy);
      cv::cartToPolar(flow_xy[0], flow_xy[1], magnitude, angle, true);
      cv::Mat hue, value, hsv;
      angle.convertTo(hue, CV_8U, 0.5);
      // Fixed scale, full brightness at 10 px: min-max lets one bad blob darken all the rest
      magnitude.convertTo(value, CV_8U, 255.0 / 10.0);
      cv::Mat channels[] = {hue, cv::Mat(hue.size(), CV_8U, cv::Scalar(255)), value};
      cv::merge(channels, 3, hsv);
      cv::Mat img_flow;
      cv::cvtColor(hsv, img_flow, cv::COLOR_HSV2BGR);

      cv::imshow("view", img_flow);
      cv::waitKey(1);

      static std::string const save_path = ros::param::param<std::string>("~save_path", "");
      if (!save_path.empty() && num_frames == 100) {
        cv::Mat side_by_side;
        cv::hconcat(image, img_flow, side_by_side);
        cv::imwrite(save_path, side_by_side);
      }
    }
    prev_gray = gray;
    ++num_frames;
    // ~~~~ end solution

  } catch (cv_bridge::Exception &e) {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

/**
 * @function main
 * @brief Main function
 */
int main(int argc, char **argv) {
  ros::init(argc, argv, "optical_flow");
  ros::NodeHandle local_nh("~");

  cv::namedWindow("view", cv::WINDOW_NORMAL);
  image_transport::ImageTransport it(local_nh);
  image_transport::Subscriber sub = it.subscribe("/images_topic", 100, imageCallback);

  ros::spin();
  cv::destroyWindow("view");
  return EXIT_SUCCESS;
}
