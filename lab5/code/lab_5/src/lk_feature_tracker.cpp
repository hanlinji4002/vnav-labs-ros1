#include "lk_feature_tracker.h"

#include <numeric>
#include <string>
#include <vector>

#include <glog/logging.h>

#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

#include <ros/ros.h>

using namespace cv;
using namespace cv::xfeatures2d;

/**
   LK feature tracker Constructor.
*/
LKFeatureTracker::LKFeatureTracker() {
  // Feel free to modify if you want!
  cv::namedWindow(window_name_, cv::WINDOW_NORMAL);
}

LKFeatureTracker::~LKFeatureTracker() {
  // Feel free to modify if you want!
  cv::destroyWindow(window_name_);
}

/** TODO This is the main tracking function, given two images, it detects,
 * describes and matches features.
 * We will be modifying this function incrementally to plot different figures
 * and compute different statistics.
 @param[in] frame Current image frame
*/
void LKFeatureTracker::trackFeatures(const cv::Mat& frame) {

  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  //  DELIVERABLE 7 | Feature Tracking: Lucas-Kanade Tracker
  // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~  ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
  //
  // For this part, you will need to:
  //
  //   1. Using OpenCV’s documentation and the C++ API for the LK tracker, track
  //   features for the video sequences we provided you by using the Harris
  //   corner detector (like here). Show the feature tracks at a given frame
  //   extracted when using the Harris corners (consider using the 'show'
  //   function below)
  //
  //   Hint 1: take a look at cv::goodFeaturesToTrack and cv::calcOpticalFlowPyrLK
  //
  //   2. Add an extra entry in the table you made previously for the Harris +
  //   LK tracker
  //
  //   Note: LKFeatureTracker does not inherit from the base tracker like other
  //   feature trackers, so you need to also implement the statistics gathering
  //   code right here.
  //
  // ~~~~ begin solution
  static int const max_corners = ros::param::param<int>("~max_corners", 1000);
  static constexpr double quality_level = 0.01;
  static constexpr double min_distance = 10.0;
  static constexpr int block_size = 3;
  static constexpr double harris_k = 0.04;

  cv::Mat gray;
  cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

  cv::TickMeter timer;
  timer.start();
  // Track the corners of the previous frame, the first frame only detects
  std::vector<cv::Point2f> prev_tracked, curr_tracked;
  if (!prev_corners_.empty()) {
    std::vector<cv::Point2f> curr_corners;
    std::vector<uchar> status;
    std::vector<float> error;
    cv::calcOpticalFlowPyrLK(prev_frame_, gray, prev_corners_, curr_corners, status, error);
    for (size_t i = 0; i < status.size(); ++i) {
      if (status[i] != 0) {
        prev_tracked.push_back(prev_corners_[i]);
        curr_tracked.push_back(curr_corners[i]);
      }
    }
  }

  // Keep only the inliers, outliers would stay wrong in the next frames
  std::vector<uchar> inlier_mask;
  inlierMaskComputation(prev_tracked, curr_tracked, &inlier_mask);
  std::vector<cv::Point2f> prev_inliers, curr_inliers;
  for (size_t i = 0; i < inlier_mask.size(); ++i) {
    if (inlier_mask[i] != 0) {
      prev_inliers.push_back(prev_tracked[i]);
      curr_inliers.push_back(curr_tracked[i]);
    }
  }

  // Top up with new Harris corners, away from the corners still tracked
  cv::Mat mask(gray.size(), CV_8UC1, cv::Scalar(255));
  for (const cv::Point2f& corner : curr_inliers) {
    cv::circle(mask, corner, static_cast<int>(min_distance), cv::Scalar(0), -1);
  }
  std::vector<cv::Point2f> new_corners;
  int const num_missing = max_corners - static_cast<int>(curr_inliers.size());
  if (num_missing > 0) {
    cv::goodFeaturesToTrack(gray, new_corners, num_missing, quality_level, min_distance,
                            mask, block_size, true, harris_k);
  }
  timer.stop();

  // Statistics of this pair of frames (no pair yet in the first frame)
  if (!prev_frame_.empty()) {
    float const new_num_samples = static_cast<float>(num_samples_) + 1.0f;
    float const old_num_samples = static_cast<float>(num_samples_);
    float const num_tracked = static_cast<float>(curr_tracked.size());
    float const num_inliers = static_cast<float>(curr_inliers.size());
    float const inlier_ratio = curr_tracked.empty() ? 0.0f : num_inliers / num_tracked;
    avg_num_keypoints_img1_ = (avg_num_keypoints_img1_ * old_num_samples + static_cast<float>(prev_corners_.size())) / new_num_samples;
    avg_num_keypoints_img2_ = (avg_num_keypoints_img2_ * old_num_samples + num_tracked) / new_num_samples;
    avg_num_matches_ = (avg_num_matches_ * old_num_samples + num_tracked) / new_num_samples;
    avg_num_inliers_ = (avg_num_inliers_ * old_num_samples + num_inliers) / new_num_samples;
    avg_inlier_ratio_ = (avg_inlier_ratio_ * old_num_samples + inlier_ratio) / new_num_samples;
    avg_time_ms_ = (avg_time_ms_ * old_num_samples + static_cast<float>(timer.getTimeMilli())) / new_num_samples;
    ++num_samples_;
    show(frame, prev_inliers, curr_inliers);
  }

  prev_frame_ = gray;
  prev_corners_ = curr_inliers;
  prev_corners_.insert(prev_corners_.end(), new_corners.begin(), new_corners.end());
  // ~~~~ end solution
  // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
  //                             end deliverable 7
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

}

/** TODO Display image with tracked features from prev to curr on the image
 * corresponding to 'frame'
 * @param[in] frame The current image frame, to draw the feature track on
 * @param[in] prev The previous set of keypoints
 * @param[in] curr The set of keypoints for the current frame
 */
void LKFeatureTracker::show(const cv::Mat& frame, std::vector<cv::Point2f>& prev,
                            std::vector<cv::Point2f>& curr) {
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // ~~~~ begin solution
  // Old track segments fade away
  if (tracks_img_.empty()) {
    tracks_img_ = cv::Mat::zeros(frame.size(), CV_8UC3);
  }
  tracks_img_ *= 0.95;

  cv::Mat img_tracks = frame.clone();
  for (size_t i = 0; i < curr.size(); ++i) {
    cv::line(tracks_img_, prev[i], curr[i], cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
    cv::circle(img_tracks, curr[i], 2, cv::Scalar(0, 0, 255), -1, cv::LINE_AA);
  }
  cv::add(img_tracks, tracks_img_, img_tracks);
  cv::putText(img_tracks, "track count: " + std::to_string(curr.size()), cv::Point(10, 25),
              cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
  cv::imshow(window_name_, img_tracks);
  if (!save_prefix_.empty() && num_samples_ == kSavedVideoFrame) {
    cv::imwrite(save_prefix_ + "tracks.png", img_tracks);
  }
  // ~~~~ end solution
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
}

void LKFeatureTracker::printStats() const {
  std::cout << "Avg. Keypoints 1 Size: " << avg_num_keypoints_img1_ << std::endl;
  std::cout << "Avg. Keypoints 2 Size: " << avg_num_keypoints_img2_ << std::endl;
  std::cout << "Avg. Number of matches: " << avg_num_matches_ << std::endl;
  std::cout << "Avg. Number of Inliers: " << avg_num_inliers_ << std::endl;
  std::cout << "Avg. Inliers ratio: " << avg_inlier_ratio_ << std::endl;
  std::cout << "Avg. Time per pair (ms): " << avg_time_ms_ << std::endl;
  std::cout << "Num. of samples: " << num_samples_ << std::endl;
}

/** Compute Inlier Mask out of the given matched keypoints.
 @param[in] pts1 List of keypoints detected on the first image.
 @param[in] pts2 List of keypoints detected on the second image.
 @param[out] inlier_mask Mask indicating inliers (1) from outliers (0).
*/
void LKFeatureTracker::inlierMaskComputation(const std::vector<cv::Point2f>& pts1,
                                             const std::vector<cv::Point2f>& pts2,
                                             std::vector<uchar>* inlier_mask) const {
  CHECK_NOTNULL(inlier_mask);

  static constexpr double max_dist_from_epi_line_in_px = 3.0;
  static constexpr double confidence_prob = 0.99;
  try {
    findFundamentalMat(pts1, pts2, cv::FM_RANSAC,
                       max_dist_from_epi_line_in_px, confidence_prob,
                       *inlier_mask);
  } catch(...) {
    ROS_WARN("Inlier Mask could not be computed, this can happen if there"
             "are not enough features tracked.");
  }
}
