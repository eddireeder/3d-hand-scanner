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


// Class to store hand view data
class HandView {
  public:
    // Path to view image
    boost::filesystem::path image_path;
    // Set of 21 * [x, y, score] keypoints
    float keypoints[21][3];
    // Average red, blue, green values across keypoints
    float average_rgb[3];
};


// Main function
int main(int argc, const char** argv) {

  // Initialise arguments
  std::string input_dir = "";
  std::string output_dir = "";
  bool stage_1 = false;
  bool stage_2 = false;
  bool stage_3 = false;
  bool stage_4 = false;
  bool stage_5 = false;
  bool stage_6 = false;
  bool stage_7 = false;

  // Loop through arguments and check for flags
  for (int i = 1; i < argc; i++) {
    if (strcmp(argv[i], "--input") == 0) {
      input_dir = argv[i + 1];
    } else if (strcmp(argv[i], "--output") == 0) {
      output_dir = argv[i + 1];
    } else if (strcmp(argv[i], "--stage-1") == 0) {
      stage_1 = true;
    } else if (strcmp(argv[i], "--stage-2") == 0) {
      stage_2 = true;
    } else if (strcmp(argv[i], "--stage-3") == 0) {
      stage_3 = true;
    } else if (strcmp(argv[i], "--stage-4") == 0) {
      stage_4 = true;
    } else if (strcmp(argv[i], "--stage-5") == 0) {
      stage_5 = true;
    } else if (strcmp(argv[i], "--stage-6") == 0) {
      stage_6 = true;
    } else if (strcmp(argv[i], "--stage-7") == 0) {
      stage_7 = true;
    }
  }

  // Define boost file paths
  const boost::filesystem::path input_path = boost::filesystem::system_complete(input_dir);
  const boost::filesystem::path output_path = boost::filesystem::system_complete(output_dir);
  const boost::filesystem::path openmvg_build_path = boost::filesystem::system_complete(openmvg_build);
  const boost::filesystem::path openmvs_build_path = boost::filesystem::system_complete(openmvs_build);
  const boost::filesystem::path openpose_root_path = boost::filesystem::system_complete(openpose_root);

  // Count number of files in input dir
  int num_files = 0;
  for (auto& entry: boost::make_iterator_range(boost::filesystem::directory_iterator(input_path), {})) {
    num_files++;
  }


  // ==============================================================
  // STAGE 1 - Execute SFM pipeline
  // ==============================================================

  // Make directory for SFM output
  boost::filesystem::path sfm_output_path = output_path / "sfm";
  std::string mkdir_sfm_cmd = "mkdir " + sfm_output_path.string();
  system(mkdir_sfm_cmd.c_str());

  std::cout << "Executing SFM pipeline" << std::endl;
  boost::filesystem::path sfm_pipeline_path = openmvg_build_path / "software" / "SfM" / "SfM_GlobalPipeline.py";
  std::string sfm_pipeline_cmd = "python " + sfm_pipeline_path.string() + " " + input_path.string() + " " + sfm_output_path.string();
  std::string sfm_pipeline_response = exec(sfm_pipeline_cmd);


  // ==============================================================
  // STAGE 2 - Convert SFM output to MVS scene
  // ==============================================================

  // Make directory for MVS output
  boost::filesystem::path mvs_output_path = output_path / "mvs";
  std::string mkdir_mvs_cmd = "mkdir " + mvs_output_path.string();
  system(mkdir_mvs_cmd.c_str());

  std::cout << "Converting SFM output to MVS scene" << std::endl;
  boost::filesystem::path sfm_data_bin_path = output_path / "sfm" / "reconstruction_global" / "sfm_data.bin";
  boost::filesystem::path mvs_scene_path = output_path / "mvs" / "scene.mvs";
  boost::filesystem::path undistorted_images_path = output_path / "mvs" / "scene_undistorted_images";
  // Can execute the following because OpenMVG installed binaries to PATH dir
  std::string sfm_to_mvs_cmd = "openMVG_main_openMVG2openMVS -i " + sfm_data_bin_path.string() + " -o " + mvs_scene_path.string() + " -d " + undistorted_images_path.string();
  std::string sfm_to_mvs_response = exec(sfm_to_mvs_cmd);


  // ==============================================================
  // STAGE 3 - Execute MVS pipeline
  // ==============================================================

  std::cout << "Densifying point cloud with MVS pipeline" << std::endl;
  boost::filesystem::path densifypointcloud_path = openmvs_build_path / "bin" / "DensifyPointCloud";
  std::string mvs_pipeline_cmd = densifypointcloud_path.string() + " " + mvs_scene_path.string();
  std::string mvs_pipeline_response = exec(mvs_pipeline_cmd);


  // ==============================================================
  // STAGE 4 - Search every image for hand keypoints
  // ==============================================================

  // Change directory to OpenPose root
  boost::filesystem::current_path(openpose_root_path);

  // Create array for storing hand view data
  HandView hand_views[num_files];
  
  // Loop through files in input directory
  int file_num = 0;
  for (auto& entry: boost::make_iterator_range(boost::filesystem::directory_iterator(input_path), {})) {
    std::cout << "Finding hand keypoints in: " << entry << std::endl;

    // Create new Hand View and assign image path
    HandView hand_view;
    hand_view.image_path = boost::filesystem::system_complete(entry);

    // Execute OpenPose user code file on image
    boost::filesystem::path user_code_path = openpose_root_path / "build" / "examples" / "user_code" / "hand_from_image.bin";
    std::string openpose_cmd = user_code_path.string() + " -image_path " + hand_view.image_path.string() + "-no_display true";
    std::string openpose_response = exec(openpose_cmd);
    std::string data_start_delimiter = "Left hand keypoints: Array<T>::toString():\n";
    openpose_response.erase(0, openpose_response.find(data_start_delimiter) + data_start_delimiter.length());

    // Loop through each keypoint (line)
    for (int i = 0; i < 21; i++) {
      std::string line = openpose_response.substr(0, openpose_response.find("\n"));
      for (int j = 0; j < 3; j++) {
        std::string value = line.substr(0, line.find(" "));
        // Store keypoint values in hand view
        hand_view.keypoints[i][j] = std::stof(value);
        line.erase(0, line.find(" ") + 1);
      }
      openpose_response.erase(0, openpose_response.find("\n") + 1);
    }
    // Add view to array
    hand_views[file_num] = hand_view;

    // Increment file num
    file_num++;
  }


  // ==============================================================
  // STAGE 5 - Filter keypoint sets
  // ==============================================================


  // ==============================================================
  // STAGE 6 - Extract hand colour range from keypoint sets
  // ==============================================================

  std::cout << "Extracting colour range and average colour from image keypoints" << std::endl;
  for (int i = 0; i < num_files; i++) {

    // Read image file
    cv::Mat img = cv::imread(hand_views[i].image_path.string());

    // Loop through all 21 keypoints and average the colour values
    int total_blue = 0;
    int total_green = 0;
    int total_red = 0;

    for (int keypoint_i; keypoint_i < 21; keypoint_i++) {
      // Extract colour values at keypoint
      const float x = hand_views[i].keypoints[keypoint_i][0];
      const float y = hand_views[i].keypoints[keypoint_i][1];
      const float score = hand_views[i].keypoints[keypoint_i][2];
      printf("x: %f y: %f score: %f\n", x, y, score);

      cv::Vec3b intensity = img.at<cv::Vec3b>(y, x);
      total_blue = total_blue + intensity[0];
      total_green = total_green + intensity[1];
      total_red = total_red + intensity[2];
    }

    const float average_blue = (float) total_blue/21;
    const float average_green = (float) total_green/21;
    const float average_red = (float) total_red/21;
    printf("r: %f g: %f b: %f\n", average_red, average_green, average_blue);

    // Assign colour values to hand view
    hand_views[i].average_rgb[0] = average_red;
    hand_views[i].average_rgb[1] = average_green;
    hand_views[i].average_rgb[2] = average_blue;
  }


  // ==============================================================
  // STAGE 7 - Locate 21 keypoints in 3D space
  // ==============================================================

  // Convert sfm_data.bin to json
  boost::filesystem::path sfm_data_json_path = output_path / "sfm" / "reconstruction_global" / "sfm_data.json";
  std::string sfm_data_to_json_cmd = "openMVG_main_ConvertSfM_DataFormat -i " + sfm_data_bin_path.string() + " -o " + sfm_data_json_path.string();
  system(sfm_data_to_json_cmd.c_str());

  // Read sfm_data.json to get camera/view information (intrinsics and extrinsics)



  // Read the saved point cloud
  // Remove points in dense point cloud not in colour range
  // Save new point cloud

  std::cout << "Done" << std::endl;
  return 0;
}
