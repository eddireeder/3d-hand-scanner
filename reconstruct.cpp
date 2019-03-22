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


  // STAGE 1 - Execute SFM pipeline

  if (stage_1) {
    std::cout << "Executing SFM pipeline" << std::endl;
    std::string sfm_pipeline_cmd = "python " + openmvg_build + "/software/SfM/SfM_GlobalPipeline.py " + input_dir + " " + output_dir;
    std::string sfm_pipeline_response = exec(sfm_pipeline_cmd);
    //std::cout << sfm_pipeline_response;
  }


  // STAGE 2 - Convert SFM output to MVS scene

  if (stage_2) {
    std::cout << "Converting SFM output to MVS scene" << std::endl;
    std::string mkdir_cmd = "mkdir " + output_dir + "/mvs";
    system(mkdir_cmd.c_str());
    std::string sfm_to_mvs_cmd = "openMVG_main_openMVG2openMVS -i " + output_dir + "/reconstruction_global/sfm_data.bin -o " + output_dir + "/mvs/scene.mvs -d " + output_dir + "/mvs/scene_undistorted_images";
    std::string sfm_to_mvs_response = exec(sfm_to_mvs_cmd);
    //std::cout << sfm_to_mvs_response;
  }


  // STAGE 3 - Execute MVS pipeline

  if (stage_3) {
    std::cout << "Densifying point cloud with MVS pipeline" << std::endl;
    std::string mvs_pipeline_cmd = openmvs_build + "/bin/DensifyPointCloud " + output_dir + "/mvs/scene.mvs";
    std::string mvs_pipeline_response = exec(mvs_pipeline_cmd);
    //std::cout << mvs_pipeline_response;
  }


  // STAGE 4 - Search every image for hand keypoints

  if (stage_4) {
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
          // Store keypoint values for later use
          keypoint_sets[file_num][i][j] = std::stof(value);
          line.erase(0, line.find(" ") + 1);
        }
        openpose_response.erase(0, openpose_response.find("\n") + 1);
      }

      // Increment file num
      file_num++;
    }
  }


  // STAGE 5 - Filter keypoint sets

  if (stage_5) {

  }


  // STAGE 6 - Extract hand colour range from keypoint sets

  if (stage_6) {
    file_num = 0;
    std::cout << "Extracting colour range and average colour from image keypoints" << std::endl;
    for (auto& entry: boost::make_iterator_range(boost::filesystem::directory_iterator(input_path), {})) {

      // Read image
      std::ostringstream file_path;
      file_path << entry;
      const std::string file_path_string = file_path.str();
      const std::string file_path_string_cut = file_path_string.substr(1, file_path_string.size() - 2);
      const cv::Mat img = cv::imread(file_path_string_cut);

      // Start loop through all 21 keypoints and average the color values
      int total_blue = 0;
      int total_green = 0;
      int total_red = 0;
      int num_keypoints_used = 0;
    
      for (int i = 0; i < 21; i++) {
        // Extract colour values at keypoint
        const float x = keypoint_sets[file_num][i][0];
        const float y = keypoint_sets[file_num][i][1];
        const float score = keypoint_sets[file_num][i][2];
        printf("x: %f y: %f score: %f\n", x, y, score);
        
        cv::Vec3b intensity = img.at<cv::Vec3b>(y, x);
        //std::cout << intensity << std::endl;

        total_blue = total_blue + intensity[0];
        total_green = total_green + intensity[1];
        total_red = total_red + intensity[2];
      }

      const float average_blue = (float) total_blue/21;
      const float average_green = (float) total_green/21;
      const float average_red = (float) total_red/21;
      printf("r: %f g: %f b: %f\n", average_red, average_green, average_blue);

      // Increment file num
      file_num++;
    }
  }


  // STAGE 7 - Locate 21 keypoints in 3D space

  if (stage_7) {

  }


  // Read the saved point cloud
  // Remove points in dense point cloud not in colour range
  // Save new point cloud

  std::cout << "Done" << std::endl;
  return 0;
}
