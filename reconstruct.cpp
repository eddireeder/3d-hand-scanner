#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <array>
#include <ctime>

#include <opencv2/imgcodecs.hpp>
#include <boost/filesystem.hpp>
#include <boost/range/iterator_range.hpp>
#include <openpose/headers.hpp>


// Define the OpenMVG build and installed directory
std::string openmvg_build = "/Users/Eddie/university/dissertation/libraries/sources/openMVG/build";

// Define the OpenMVS build directory
std::string openmvs_build = "/Users/Eddie/university/dissertation/libraries/sources/openMVS/openMVS_build";

// Define the OpenPose build directory
std::string openpose_root = "/Users/Eddie/university/dissertation/libraries/sources/openpose";


// Function to convert a string to a char array (C-string)
char * to_char_array(std::string string) {
  char * char_array = new char [string.length() + 1];
  strcpy(char_array, string.c_str());
  return char_array;
}


// Function to execute bash commands with input and output
std::string exec(const std::string string_cmd) {
  char* cmd = to_char_array(string_cmd);
  std::array<char, 128> buffer;
  std::string result;
  std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd, "r"), pclose);
  if (!pipe) {
    throw std::runtime_error("popen() failed!");
  }
  while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
    result += buffer.data();
  }
  return result;
}


// Main function
int main(int argc, const char** argv) {

  // Retrieve input and output directories
  std::string input_dir = argv[1];
  std::string output_dir = argv[2];

/*

  // STAGE 1 - Execute SFM pipeline

  std::cout << "Executing SFM pipeline" << std::endl;
  std::string cmd = "python " + openmvg_build + "/software/SfM/SfM_GlobalPipeline.py " + input_dir + " " + output_dir;
  std::string response = exec(cmd);
  //std::cout << response;


  // STAGE 2 - Convert SFM output to MVS scene

  std::cout << "Converting SFM output to MVS scene" << std::endl;
  cmd = "mkdir " + output_dir + "/mvs";
  system(cmd.c_str());
  cmd = "openMVG_main_openMVG2openMVS -i " + output_dir + "/reconstruction_global/sfm_data.bin -o " + output_dir + "/mvs/scene.mvs -d " + output_dir + "/mvs/scene_undistorted_images";
  response = exec(cmd);
  //std::cout << response;


  // STAGE 3 - Execute MVS pipeline

  std::cout << "Densifying point cloud with MVS pipeline" << std::endl;
  cmd = openmvs_build + "/bin/DensifyPointCloud " + output_dir + "/mvs/scene.mvs";
  response = exec(cmd);
  //std::cout << response;
  std::cout << "Done" << std::endl;

*/

  // STAGE 4 - Search every image for hand keypoints

  std::cout << "Finding hand keypoints in images" << std::endl;
  // Change directory to OpenPose root
  boost::filesystem::current_path(openpose_root);

  // Count number of files and create variable for storing sets of (21 * [x, y, score] keypoints)
  boost::filesystem::path input_path(input_dir);
  int num_files = 0;
  for (auto& entry: boost::make_iterator_range(boost::filesystem::directory_iterator(input_path), {})) {
    num_files++;
  }
  float keypoint_sets[num_files][21][3];

  // Loop through files in input directory
  int file_num = 0;
  for (auto& entry: boost::make_iterator_range(boost::filesystem::directory_iterator(input_path), {})) {
    std::cout << "Finding hand keypoints in: " << entry << std::endl;

    // Execute OpenPose usercode file on each 'entry'
    std::ostringstream openpose_cmd;
    openpose_cmd << "build/examples/user_code/hand_from_image.bin -image_path " << entry << " -no_display true";
    std::string openpose_response = exec(openpose_cmd.str());
    std::string data_start_delimiter = "Left hand keypoints: Array<T>::toString():\n";
    openpose_response.erase(0, openpose_response.find(data_start_delimiter) + data_start_delimiter.length());
    
    // Loop through each keypoint (line)
    for (int i = 0; i < 21; i++) {
      std::string line = openpose_response.substr(0, openpose_response.find("\n"));
      for (int j = 0; j < 3; j++) {
        std::string value = line.substr(0, line.find(" "));
        keypoint_sets[file_num][i][j] = std::stof(value);
        line.erase(0, line.find(" ") + 1);
      }
      openpose_response.erase(0, openpose_response.find("\n") + 1);
    }
    file_num++;
  }

  // Extract colour range

  // Read the saved point cloud

  // Remove points in dense point cloud not in colour range

  // Save new point cloud

  return 0;
}
