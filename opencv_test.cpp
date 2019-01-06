#include "opencv2/imgcodecs.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/sfm/reconstruct.hpp"

#include <stdio.h>
#include <ctime>

using namespace cv;
using namespace std;


int main(int argc, const char** argv) {

  // Feature detector options
  int max_n_features = 500;
  float scale_factor = 1.2f;
  int n_levels = 8;
  int edge_threshold = 31;
  int first_level = 0;
  int wta_k = 2;
  int score_type = cv::ORB::HARRIS_SCORE;
  int patch_size = 31;
  int fast_threshold = 20;

  // Feature matching options
  int matcher_type = DescriptorMatcher::BRUTEFORCE_HAMMING;
  float good_match_ratio = 0.8;

  // Camera options
  float cx = 0;
  float cy = 0;
  int fx = 0;
  int fy = 0;
  bool is_projective = false;


  // Collect input filenames from arguments array
  const char* filename_1 = argv[1];
  const char* filename_2 = argv[2];

  // Read images
  cv::Mat image_1 = imread(filename_1, IMREAD_GRAYSCALE);
  cv::Mat image_2 = imread(filename_2, IMREAD_GRAYSCALE);

  // Define feature detector (set parameters)
  Ptr<ORB> detector = ORB::create(
    max_n_features,
    scale_factor,
    n_levels,
    edge_threshold,
    first_level,
    wta_k,
    score_type,
    patch_size,
    fast_threshold
  );

  // Initialise empty keypoints and descriptors vectors
  std::vector<KeyPoint> keypoints_1;
  std::vector<KeyPoint> keypoints_2;
  cv::Mat descriptors_1;
  cv::Mat descriptors_2;

  // Detect features and compute descriptions
  printf("Detecting features in image 1\n");
  detector->detectAndCompute(image_1, noArray(), keypoints_1, descriptors_1);
  printf("Detecting features in image 2\n");
  detector->detectAndCompute(image_2, noArray(), keypoints_2, descriptors_2);

  // Define brute force feature matcher
  Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create(matcher_type);

  // Initialise empty matches vector
  std::vector< std::vector<DMatch> > matches;

  // Match image features
  int match_k = 2;
  printf("Matching features\n");
  matcher->knnMatch(descriptors_1, descriptors_2, matches, match_k);

  // Reject poor matches using ratio between best and second-best
  std::vector<DMatch> good_matches;

  for (int i = 0; i < matches.size(); i++) {
    if (matches[i][0].distance < good_match_ratio*matches[i][1].distance) {
      good_matches.push_back(matches[i][0]);
    }
  }

  printf("Number of good matches: %d\n", (int) good_matches.size());

  // Initialise matched points matrices
  std::vector<cv::Mat> matched_points;
  matched_points.push_back(cv::Mat(good_matches.size(), 2, DataType<float>::type));
  matched_points.push_back(cv::Mat(good_matches.size(), 2, DataType<float>::type));

  // Extract point correspondances from good matches into matrix
  for (int i = 0; i < good_matches.size(); i++) {
    Point2f matched_point_1 = keypoints_1[good_matches[i].trainIdx].pt;
    Point2f matched_point_2 = keypoints_2[good_matches[i].queryIdx].pt;
    matched_points[0].at<float>(i, 0) = matched_point_1.x;
    matched_points[0].at<float>(i, 1) = matched_point_1.y;
    matched_points[1].at<float>(i, 0) = matched_point_2.x;
    matched_points[1].at<float>(i, 1) = matched_point_2.y;
  }

  // Initialise camera calibration matrix
  Matx33d camera_k = Matx33d( fx, 0,  cx,
                              0,  fy, cy,
                              0,  0,  1 );

  // Reconstruct 3D scene from points
  printf("Reconstructing 3D points from matches\n");
  std::vector<Mat> camera_rotations, camera_translations, points_3d;
  cv::sfm::reconstruct(matched_points, camera_rotations, camera_translations, camera_k, points_3d, is_projective);
}