#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <array>
#include <ctime>


// Define the OpenMVG build and installed directory
std::string openmvg_build = "/Users/Eddie/university/dissertation/cpp/libraries/sources/openMVG/build";
std::string openmvg_installed = "/Users/Eddie/university/dissertation/cpp/libraries/installed/openMVG";

// Define the OpenMVS build directory
std::string openmvs_build = "/Users/Eddie/university/dissertation/cpp/libraries/sources/openMVS/openMVS_build";


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

  // Execute SFM pipeline
  std::cout << "Executing SFM pipeline" << std::endl;
  std::string cmd = "python " + openmvg_build + "/software/SfM/SfM_GlobalPipeline.py " + input_dir + " " + output_dir;
  std::string response = exec(cmd);
  //std::cout << response;

  // Convert SFM output to MVS scene
  std::cout << "Converting SFM output to MVS scene" << std::endl;
  cmd = "mkdir " + output_dir + "/mvs";
  system(cmd.c_str());
  cmd = openmvg_installed + "/bin/openMVG_main_openMVG2openMVS -i " + output_dir + "/reconstruction_global/sfm_data.bin -o " + output_dir + "/mvs/scene.mvs -d " + output_dir + "/mvs/scene_undistorted_images";
  response = exec(cmd);
  //std::cout << response;

  // Execute MVS pipeline
  std::cout << "Densifying point cloud with MVS pipeline" << std::endl;
  cmd = openmvs_build + "/bin/DensifyPointCloud " + output_dir + "/mvs/scene.mvs";
  response = exec(cmd);
  //std::cout << response;
  std::cout << "Done" << std::endl;

  // Search every image for hand keypoints

  // Extract colour range

  // Read the saved point cloud

  // Remove points in dense point cloud not in colour range

  // Save new point cloud

  return 0;
}
