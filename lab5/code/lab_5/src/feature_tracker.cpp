#include "feature_tracker.h"

#include <algorithm>
#include <string>
#include <utility>
#include <vector>
#include <numeric>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <opencv2/features2d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>

#include <ros/ros.h>

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  16.485 - Fall 2021  - Lab 5 coding assignment
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~  ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//
//  In this code, we ask you to implement a generic feature tracker base class
//  which you will augment with several derived classes for SIFT, SURF, ORB, and
//  FAST+BRIEF feature tracking.
//
// NOTE: Deliverables for the TEAM portion of this assignment start at number 3
// and end at number 7. If you have completed the parts labeled Deliverable 3-7,
// you are done with the TEAM portion of the lab. Deliverables 1-2 are
// individual.

using namespace cv;
using namespace cv::xfeatures2d;

/** TODO This is the main tracking function, given two images, it detects,
 * describes and matches features.
 * We will be modifying this function incrementally to plot different figures
 * and compute different statistics.
 @param[in] img_1, img_2 Images where to track features.
 @param[out] matched_kp_1_kp_2 pair of vectors of keypoints with the same size
 so that matched_kp_1_kp_2.first[i] matches with matched_kp_1_kp_2.second[i].
*/
void FeatureTracker::trackFeatures(const cv::Mat &img_1,
                                   const cv::Mat &img_2,
                                   std::pair<std::vector<cv::KeyPoint>, std::vector<cv::KeyPoint>> *matched_kp_1_kp_2) {

  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  //  DELIVERABLE 3 | Feature Descriptors (SIFT)
  // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~  ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
  //
  // For your convenience,  
  // here are two vectors of keypoints representing keypoints for image 1 and 2.
  std::vector<KeyPoint> keypoints_1, keypoints_2; 
  //
  //
  // For this part, you will need to complete the following tasks.
  //
  // Note: the actual logic for computing features  will happen inside each of the 
  // derived class. Here we are just calling the derived classes functions 
  // (because they share the same interface), and operate on the results on a
  // higher level.
  //
  //   1. Detect the keypoints for each image independently. Implement and use
  //   the skeleton function 'detectKeypoints' in the derived class, (i.e. in
  //   SiftFeatureTracker)
  //
  // ~~~~ begin solution
  cv::TickMeter timer;
  timer.start();
  detectKeypoints(img_1, &keypoints_1);
  detectKeypoints(img_2, &keypoints_2);
  timer.stop();
  // ~~~~ end solution
  //
  //   2. Display detected keypoints on both images, and save them for the
  //   deliverable. Make use of the OpenCV functions 'drawKeypoints', 'imwrite'. 
  //   Tip: use 'waitKey(0);' to avoid opencv closing the visualization window. 
  //   Mind that using waitKey(0) will block ROS from dying (bottom of this 
  //   file we offer an alternative).
  //
  // ~~~~ begin solution
  if (pair_mode_) {
    cv::Mat img_keypoints_1, img_keypoints_2;
    cv::drawKeypoints(img_1, keypoints_1, img_keypoints_1);
    cv::drawKeypoints(img_2, keypoints_2, img_keypoints_2);
    showFigure("keypoints_1", img_keypoints_1);
    showFigure("keypoints_2", img_keypoints_2);
  }
  // ~~~~ end solution
  //
  //   3. Compute descriptors by filling in the skeleton function
  //   `describeKeypoints` in the relevant derived classes (i.e.
  //   SiftFeatureTracker)
  //
  // ~~~~ begin solution
  cv::Mat descriptors_1, descriptors_2;
  timer.start();
  describeKeypoints(img_1, &keypoints_1, &descriptors_1);
  describeKeypoints(img_2, &keypoints_2, &descriptors_2);
  timer.stop();
  if (pair_mode_) {
    ROS_INFO("Descriptor dimension: %d (%s)", descriptors_1.cols,
             descriptors_1.depth() == CV_8U ? "binary, bytes" : "float");
  }
  // ~~~~ end solution
  //
  // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
  //                             end deliverable 3
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  //  DELIVERABLE 4 | Descriptor-based Feature Matching
  // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~  ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
  // Matches and good matches to be populated
  std::vector<std::vector<DMatch>> matches;
  std::vector<DMatch> good_matches;
  //
  // For this part, you will need to:
  //   1. Match descriptor vectors using the FLANN matcher. FLANN = Fast
  //   Approximate Nearest Neighbor Search Library. Use and fill in the skeleton
  //   function 'matchDescriptors' in the derived class (e.g.
  //   SiftFeatureTracker), and call them here.
  //
  // ~~~~ begin solution
  timer.start();
  matchDescriptors(descriptors_1, descriptors_2, &matches, &good_matches);
  timer.stop();
  // ~~~~ end solution
  //
  //   2. Plot the matches using the opencv function 'drawMatches'. Save the
  //   image for deliverable.
  //
  // ~~~~ begin solution
  if (pair_mode_) {
    drawMatches(img_1, img_2, keypoints_1, keypoints_2, matches);
    cv::Mat img_good_matches;
    cv::drawMatches(img_1, keypoints_1, img_2, keypoints_2, good_matches, img_good_matches,
                    Scalar::all(-1), Scalar::all(-1), std::vector<char>(),
                    DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
    showFigure("good_matches", img_good_matches);
  }
  // ~~~~ end solution
  //
  //   3. Use the function 'inlierMaskComputation' to get the inliers amongst
  //   the matches you computed. (You are not supposed to modify this function,
  //   but you will learn what it is doing in the next lecture).
  //   Note: you may need to create additional variables based on match, 
  //   good_matches and keypoints_1/2 as inputs to the
  //   'inlierMaskComputation' function.
  //
  // ~~~~ begin solution
  std::vector<KeyPoint> matched_kp_1, matched_kp_2;
  for (const DMatch& match : good_matches) {
    matched_kp_1.push_back(keypoints_1[match.queryIdx]);
    matched_kp_2.push_back(keypoints_2[match.trainIdx]);
  }
  std::vector<uchar> inlier_mask;
  inlierMaskComputation(matched_kp_1, matched_kp_2, &inlier_mask);
  if (matched_kp_1_kp_2 != nullptr) {
    *matched_kp_1_kp_2 = std::make_pair(matched_kp_1, matched_kp_2);
  }
  // ~~~~ end solution
  //
  // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
  //                             end deliverable 4
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  //  DELIVERABLE 5 | Keypoint Matching Quality
  // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~  ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
  //
  unsigned int num_inliers = 0;
  // 
  // For this part, you will need to:
  //   1. Draw the inlier (green) and outlier (red) matches in a similar way
  //   than in the handout. Make use of the function 'drawMatches'. Tip: you can
  //   first draw the outliers and then re-use the same image to draw the
  //   inliers. See the handout for more details
  //
  // ~~~~ begin solution
  std::vector<char> char_inlier_mask{inlier_mask.begin(), inlier_mask.end()};
  num_inliers = std::count_if(inlier_mask.begin(), inlier_mask.end(),
                              [](uchar inlier) { return inlier != 0; });
  cv::Mat img_inliers;
  cv::drawMatches(img_1, keypoints_1, img_2, keypoints_2, good_matches, img_inliers,
                  Scalar(0, 0, 255), Scalar(0, 0, 255), std::vector<char>(),
                  DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
  // An empty mask would mean "draw all", it happens with too few matches
  if (!char_inlier_mask.empty()) {
    cv::drawMatches(img_1, keypoints_1, img_2, keypoints_2, good_matches, img_inliers,
                    Scalar(0, 255, 0), Scalar(0, 255, 0), char_inlier_mask,
                    DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS | DrawMatchesFlags::DRAW_OVER_OUTIMG);
  }
  showFigure("inliers_outliers", img_inliers);
  // ~~~~ end solution
  //
  //   2. Calculate the statistics to fill the table in the handout.
  //   Feel free to edit the code below to include more interesting stats.

  float const new_num_samples = static_cast<float>(num_samples_) + 1.0f;
  float const old_num_samples = static_cast<float>(num_samples_);
  avg_num_keypoints_img1_ = (avg_num_keypoints_img1_ * old_num_samples + static_cast<float>(keypoints_1.size())) / new_num_samples;
  avg_num_keypoints_img2_ = (avg_num_keypoints_img2_ * old_num_samples + static_cast<float>(keypoints_2.size())) / new_num_samples;
  avg_num_matches_ = (avg_num_matches_ * old_num_samples + static_cast<float>(matches.size())) / new_num_samples;
  avg_num_good_matches_ = (avg_num_good_matches_ * old_num_samples + static_cast<float>(good_matches.size())) / new_num_samples;
  avg_num_inliers_ = (avg_num_inliers_ * old_num_samples + static_cast<float>(num_inliers)) / new_num_samples;
  // No good matches means ratio 0, not NaN which would spoil the whole average
  float const inlier_ratio = good_matches.empty() ? 0.0f : static_cast<float>(num_inliers) / static_cast<float>(good_matches.size());
  avg_inlier_ratio_ = (avg_inlier_ratio_ * old_num_samples + inlier_ratio) / new_num_samples;
  avg_time_ms_ = (avg_time_ms_ * old_num_samples + static_cast<float>(timer.getTimeMilli())) / new_num_samples;
  ++num_samples_;

  // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
  //                             end deliverable 5
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  // Wait for a key, but allow ROS to also kill everything (instead of waitKey(0)).
  // In video mode the image callback calls waitKey instead.
  if (pair_mode_) {
    while (ros::ok() && waitKey(10) == -1) {}
  }
}

void FeatureTracker::setVisualization(bool pair_mode, const std::string& save_prefix) {
  pair_mode_ = pair_mode;
  save_prefix_ = save_prefix;
}

/** Shows a figure and saves it: all figures for a pair of images, a single frame
 *  for a video.
 */
void FeatureTracker::showFigure(const std::string& name, const cv::Mat& img) const {
  cv::namedWindow(name, cv::WINDOW_NORMAL);
  cv::imshow(name, img);
  if (!save_prefix_.empty() && (pair_mode_ || num_samples_ == kSavedVideoFrame)) {
    cv::imwrite(save_prefix_ + name + ".png", img);
  }
}

/** Compute Inlier Mask out of the given matched keypoints.
   *  Both keypoints_1 and keypoints_2 input parameters must be ordered by match
   * i.e. keypoints_1[0] has been matched to keypoints_2[0].
   * Therefore, both keypoints vectors must have the same length.
    @param[in] keypoints_1 List of keypoints detected on the first image.
    @param[in] keypoints_2 List of keypoints detected on the second image.
    @param[out] inlier_mask Mask indicating inliers (1) from outliers (0).
  */
void FeatureTracker::inlierMaskComputation(const std::vector<KeyPoint> &keypoints_1,
                                           const std::vector<KeyPoint> &keypoints_2,
                                           std::vector<uchar> *inlier_mask) const {
  CHECK_NOTNULL(inlier_mask);
  const size_t size = keypoints_1.size();
  CHECK_EQ(keypoints_2.size(), size) << "Size of keypoint vectors "
                                        "should be the same!";

  std::vector<Point2f> pts1(size);
  std::vector<Point2f> pts2(size);
  for (size_t i = 0; i < keypoints_1.size(); i++) {
    pts1[i] = keypoints_1[i].pt;
    pts2[i] = keypoints_2[i].pt;
  }

  static constexpr double max_dist_from_epi_line_in_px = 3.0;
  static constexpr double confidence_prob = 0.99;
  try {
    findFundamentalMat(pts1, pts2, cv::FM_RANSAC, max_dist_from_epi_line_in_px, confidence_prob, *inlier_mask);
  } catch (...) {
    ROS_WARN("Inlier Mask could not be computed, this can happen if there"
             "are not enough features tracked.");
  }
}

/** Example of function to draw matches. Feel free to re-use this example or
 *  create your own. You will need to modify it in order to plot the different
 *  figures. You can add more functions to this class if needed.
 */
void FeatureTracker::drawMatches(const cv::Mat &img_1,
                                 const cv::Mat &img_2,
                                 const std::vector<KeyPoint> &keypoints_1,
                                 const std::vector<KeyPoint> &keypoints_2,
                                 const std::vector<std::vector<DMatch>> &matches) {
  // Only the best neighbour, the second one is just there for the ratio test
  std::vector<DMatch> best_matches;
  for (const auto& match : matches) {
    if (!match.empty()) {
      best_matches.push_back(match[0]);
    }
  }
  cv::Mat img_matches;
  cv::drawMatches(img_1,
                  keypoints_1,
                  img_2,
                  keypoints_2,
                  best_matches,
                  img_matches,
                  Scalar::all(-1),
                  Scalar::all(-1),
                  std::vector<char>(),
                  DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
  // Show detected matches
  showFigure("matches", img_matches);
}

void FeatureTracker::printStats() const {
  std::cout << "Avg. Keypoints 1 Size: " << avg_num_keypoints_img1_ << std::endl;
  std::cout << "Avg. Keypoints 2 Size: " << avg_num_keypoints_img2_ << std::endl;
  std::cout << "Avg. Number of matches: " << avg_num_matches_ << std::endl;
  std::cout << "Avg. Number of good matches: " << avg_num_good_matches_ << std::endl;
  std::cout << "Avg. Number of Inliers: " << avg_num_inliers_ << std::endl;
  std::cout << "Avg. Inliers ratio: " << avg_inlier_ratio_ << std::endl;
  std::cout << "Avg. Time per pair (ms): " << avg_time_ms_ << std::endl;
  std::cout << "Num. of samples: " << num_samples_ << std::endl;
}
