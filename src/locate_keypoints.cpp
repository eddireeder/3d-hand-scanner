#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <array>
#include <ctime>

#include <boost/filesystem.hpp>
#include "OpenMVS/MVS.h"
#include "system.h"


struct Keypoint {
  std::string filename;
  float x;
  float y;
  float score;
};


// Function to detect the keypoints of the hand from a 2D image
std::vector<Keypoint> detect_keypoints(std::string filename) {

  // Retrieve openpose root path from env variable
  char * openpose_root = getenv("OPENPOSE_ROOT");
  if (openpose_root == NULL) {
    std::cout << "OPENPOSE_ROOT environment variable not set" << std::endl;
  }

  // Change directory to OpenPose root
  boost::filesystem::path openpose_root_path = boost::filesystem::system_complete(openpose_root);
  boost::filesystem::current_path(openpose_root_path);

  // Execute openpose usercode file on image
  boost::filesystem::path user_code_path = openpose_root_path / "build" / "examples" / "user_code" / "hand_from_image.bin";
  boost::filesystem::path image_path = boost::filesystem::system_complete(filename);
  std::string openpose_cmd = user_code_path.string() + " -image_path " + image_path.string() + " -no_display true";
  std::string openpose_response = exec(openpose_cmd);

  // Initialise vector of keypoints
  std::vector<Keypoint> keypoints;

  // Parse and store keypoints
  std::string data_start_delimiter = "Left hand keypoints: Array<T>::toString():\n";
  openpose_response.erase(0, openpose_response.find(data_start_delimiter) + data_start_delimiter.length());

  // Loop through each keypoint (line)
  for (int keypoint_i = 0; keypoint_i < 21; keypoint_i++) {

    Keypoint keypoint;
    std::string line = openpose_response.substr(0, openpose_response.find("\n"));

    // Extract keypoint (x, y, score)
    keypoint.x = std::stof(line.substr(0, line.find(" ")));
    line.erase(0, line.find(" ") + 1);
    keypoint.y = std::stof(line.substr(0, line.find(" ")));
    line.erase(0, line.find(" ") + 1);
    keypoint.score = std::stof(line.substr(0, line.find(" ")));
    line.erase(0, line.find(" ") + 1);
    openpose_response.erase(0, openpose_response.find("\n") + 1);

    // Add keypoint to vector
    keypoints.emplace_back(keypoint);
  }
  return keypoints;
}


// Function to locate keypoints in the 3D space via back-tracing
void locate_keypoints(std::vector<Keypoint> keypoints, std::string mvs_filename) {

  // Open .mvs file and save to object
  MVS::Scene scene;
  if (!scene.LoadInterface(mvs_filename)) {
    std::cout << "Could not load " << mvs_filename << std::endl;
  }

  // Locate image corresponding to keypoints
  std::cout << scene.images[0].scale << " " << scene.images[0].width << " " << scene.images[0].height << std::endl;
  MVS::Image image = scene.images[0];

  // Extract the camera from the image
  MVS::Camera camera = image.camera;
  std::cout << camera.K << std::endl;

  // Cast a ray from the view in the direction of the keypoint
  MVS::TPoint2 x = MVS::TPoint2(keypoints[0].x, keypoints[1].y);
  MVS::TPoint3 ray = MVS::RayPoint(x);

  // See where the ray intersects the mesh
  
}


// Main function
int main(int argc, const char** argv) {
  
  // Test keypoint detection
  std::vector<Keypoint> keypoints;
  /*
  keypoints = detect_keypoints(argv[1]);

  for (int i = 0; i < 21; i++) {
    std::cout << keypoints[i].x << " " << keypoints[i].y << " " << keypoints[i].score << std::endl;
  }
  */

  locate_keypoints(keypoints, argv[1]);
}