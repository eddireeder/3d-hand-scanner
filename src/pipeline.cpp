#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <array>
#include <ctime>

//#include <nlohmann/json.hpp>
//#include <opencv2/imgcodecs.hpp>
#include <boost/filesystem.hpp>
#include <boost/range/iterator_range.hpp>
//#include <openpose/headers.hpp>
//#include <openMVG/numeric/numeric.h>
//#include <openMVG/multiview/projection.hpp>
//#include <openMVG/multiview/triangulation_nview.hpp>
//#include <openMVG/multiview/triangulation.hpp>
//#include <openMVG/sfm/sfm_data.hpp>
//#include <openMVG/sfm/sfm_data_io.hpp>
//#include <openMVG/cameras/Camera_Pinhole_Radial.hpp>
//#include <openMVG/sfm/sfm_data_io_ply.hpp>
//#include <openMVG/sfm/sfm_data_triangulation.hpp>

#include "system.h"


// Function to write vertices to .ply file
/*
void write_to_ply(const openMVG::Vec3 vertices[], const int num_vertices, const std::string filepath) {

  std::cout << "Writing to " << filepath << std::endl;

  std::ofstream file;
  file.open(filepath);
  file << "ply\n";
  file << "format ascii 1.0\n";
  file << "element vertex " << num_vertices << "\n";
  file << "property float x\n";
  file << "property float y\n";
  file << "property float z\n";
  file << "end_header\n";

  for (int i = 0; i < num_vertices; i++) {
    file << vertices[i][0] << " " << vertices[i][1] << " " << vertices[i][2] << "\n";
  }

  file.close();
}
*/


// Class to store hand view data
/*
class HandView {
  public:
    boost::filesystem::path image_path;

    // Camera intrinsics
    float focal_length;
    float principal_point[2];

    // Camera extrinsics/pose
    openMVG::Vec3 center;
    openMVG::Mat3 rotation;

    // Camera translation
    openMVG::Vec3 translation;

    // View projection matrix
    openMVG::Mat34 projection;

    // Set of 21 * [x, y, score] keypoints
    float keypoints[21][3];
    bool use_keypoints = true;
    float average_rgb[3];  
};
*/


// Main function
int main(int argc, const char** argv) {

  // Initialise arguments
  std::string input_dir = "";
  std::string output_dir = "";
  int focal_length_pix = 0;

  // Loop through arguments and check for flags
  for (int i = 1; i < argc; i++) {
    if (strcmp(argv[i], "--input") == 0) {
      input_dir = argv[i + 1];
    } else if (strcmp(argv[i], "--output") == 0) {
      output_dir = argv[i + 1];
    } else if (strcmp(argv[i], "--focal") == 0) {
      focal_length_pix = std::stoi(argv[i + 1]);
    } 
  }

  // Define boost file paths
  const boost::filesystem::path input_path = boost::filesystem::system_complete(input_dir);
  const boost::filesystem::path output_path = boost::filesystem::system_complete(output_dir);

  // Count number of files in input dir
  int num_files = 0;
  for (auto& entry: boost::make_iterator_range(boost::filesystem::directory_iterator(input_path), {})) {
    num_files++;
  }



  // ==============================================================
  // STAGE 1 - Initialise sfm_data object with images
  // ==============================================================

  // Make directory for SFM output
  std::string mkdir_sfm_cmd = "mkdir " + (output_path / "sfm").string();
  system(mkdir_sfm_cmd.c_str());

  std::cout << "Initialising sfm_data object with images" << std::endl;
  std::string init_image_listing_cmd;
  if (focal_length_pix == 0) {
    // Get global path to sensor width database file
    boost::filesystem::path sensor_width_db = boost::filesystem::system_complete("../sensor_width_camera_database.txt");
    init_image_listing_cmd = "openMVG_main_SfMInit_ImageListing -i " + input_path.string() + " -o " + (output_path / "sfm").string() + " -d " + sensor_width_db.string();
  } else {
    init_image_listing_cmd = "openMVG_main_SfMInit_ImageListing -i " + input_path.string() + " -o " + (output_path / "sfm").string() + " -f " + std::to_string(focal_length_pix);
  }
  std::string init_image_listing_response = exec(init_image_listing_cmd);



  // ==============================================================
  // STAGE 2 - Compute image features
  // ==============================================================

  // Make directory for detected features and matches
  std::string mkdir_matches_cmd = "mkdir " + (output_path / "sfm" / "matches").string();
  system(mkdir_matches_cmd.c_str());
  
  std::cout << "Detecting features in images" << std::endl;
  std::string compute_features_cmd = "openMVG_main_ComputeFeatures -i " + (output_path / "sfm" / "sfm_data.json").string() + " -o " + (output_path / "sfm" / "matches").string() + " -p HIGH";
  std::string compute_features_response = exec(compute_features_cmd);



  // ==============================================================
  // STAGE 3 - Compute matching features
  // ==============================================================
  
  std::cout << "Matching detected features" << std::endl;
  std::string compute_matches_cmd = "openMVG_main_ComputeMatches -i " + (output_path / "sfm" / "sfm_data.json").string() + " -o " + (output_path / "sfm" / "matches").string() + " -g e";
  std::string compute_matches_response = exec(compute_matches_cmd);



  // ==============================================================
  // STAGE 4 - Compute global SFM
  // ==============================================================

  std::cout << "Computing global SFM" << std::endl;
  std::string compute_global_sfm_cmd = "openMVG_main_GlobalSfM -i " + (output_path / "sfm" / "sfm_data.json").string() + " -m " + (output_path / "sfm" / "matches").string() + " -o " + (output_path / "sfm" / "reconstruction_global").string();
  std::string compute_global_sfm_response = exec(compute_global_sfm_cmd);
  

/*
  // ==============================================================
  // STAGE 5 - Convert SFM output to MVS scene
  // ==============================================================

  // Make directory for MVS output
  std::string mkdir_mvs_cmd = "mkdir " + (output_path / "mvs").string();
  system(mkdir_mvs_cmd.c_str());

  std::cout << "Converting SFM output to MVS scene" << std::endl;
  boost::filesystem::path sfm_data_bin_path = output_path / "sfm" / "reconstruction_global" / "sfm_data.bin";
  boost::filesystem::path mvs_scene_path = output_path / "mvs" / "scene.mvs";
  boost::filesystem::path undistorted_images_path = output_path / "mvs" / "scene_undistorted_images";
  std::string sfm_to_mvs_cmd = "openMVG_main_openMVG2openMVS -i " + sfm_data_bin_path.string() + " -o " + mvs_scene_path.string() + " -d " + undistorted_images_path.string();
  std::string sfm_to_mvs_response = exec(sfm_to_mvs_cmd);



  // ==============================================================
  // STAGE 6 - Execute MVS pipeline
  // ==============================================================

  std::cout << "Densifying point cloud using MVS" << std::endl;
  std::string mvs_pipeline_cmd = "/usr/local/bin/OpenMVS/DensifyPointCloud " + (output_path / "mvs" / "scene.mvs").string() + " -w " + (output_path / "mvs").string();
  std::string mvs_pipeline_response = exec(mvs_pipeline_cmd);



  // ==============================================================
  // STAGE 7 - Reconstruct mesh from dense point cloud
  // ==============================================================

  std::cout << "Reconstructing a mesh from the dense point cloud" << std::endl;
  std::string reconstruct_mesh_cmd = "/usr/local/bin/OpenMVS/ReconstructMesh " + (output_path / "mvs" / "scene_dense.mvs").string() + " -w " + (output_path / "mvs").string() + " -d 5";
  std::string reconstruct_mesh_response = exec(reconstruct_mesh_cmd);



  // ==============================================================
  // STAGE 8 - Texture the reconstructed mesh
  // ==============================================================

  std::cout << "Texturing the mesh" << std::endl;
  std::string texture_mesh_cmd = "/usr/local/bin/OpenMVS/TextureMesh " + (output_path / "mvs" / "scene_dense_mesh.mvs").string() + " -w " + (output_path / "mvs").string() + " --resolution-level 2";
  std::string texture_mesh_response = exec(texture_mesh_cmd);

*/
  // ==============================================================
  // STAGE 7 - Extract SFM data
  // ==============================================================
/*
  std::cout << "Loading sfm_data from sfm_data.bin" << std::endl;

  // Initialise empty sfm_data
  openMVG::sfm::SfM_Data sfm_data;
  sfm_data.s_root_path = input_path.string();

  // Convert sfm_data.bin to json
  boost::filesystem::path sfm_data_bin_path = output_path / "sfm" / "reconstruction_global" / "sfm_data.bin";
  boost::filesystem::path sfm_data_json_path = output_path / "sfm" / "reconstruction_global" / "sfm_data.json";
  std::string sfm_data_to_json_cmd = "openMVG_main_ConvertSfM_DataFormat -i " + sfm_data_bin_path.string() + " -o " + sfm_data_json_path.string();
  system(sfm_data_to_json_cmd.c_str());

  // Read sfm_data.json to extract camera/view information (views, intrinsics and poses)
  std::ifstream i(sfm_data_json_path.string());
  nlohmann::json sfm_data_json = nlohmann::json::parse(i);
  nlohmann::json json_views = sfm_data_json["views"];
  nlohmann::json json_intrinsics = sfm_data_json["intrinsics"];
  nlohmann::json json_poses = sfm_data_json["extrinsics"];

  // Iterate through views
  const int num_views = json_views.size();
  for (int i = 0; i < num_views; i++) {

    // Retrieve view
    openMVG::IndexT key = json_views[i]["key"];
    nlohmann::json view = json_views[i]["value"]["ptr_wrapper"]["data"];
    sfm_data.views[key] = std::make_shared<openMVG::sfm::View>(
      view["filename"].get<std::string>(),
      view["id_view"].get<openMVG::IndexT>(),
      view["id_intrinsic"].get<openMVG::IndexT>(),
      view["id_pose"].get<openMVG::IndexT>(),
      view["width"].get<openMVG::IndexT>(),
      view["height"].get<openMVG::IndexT>()
    );
  }

  // Iterate through poses
  const int num_poses = json_poses.size();
  for (int i = 0; i < num_poses; i++) {

    // Retrieve pose
    openMVG::IndexT key = json_poses[i]["key"];
    nlohmann::json pose = json_poses[i]["value"];
    openMVG::Mat3 r;
    openMVG::Vec3 c;
    r(0, 0) = pose["rotation"][0][0];
    r(0, 1) = pose["rotation"][0][1];
    r(0, 2) = pose["rotation"][0][2];
    r(1, 0) = pose["rotation"][1][0];
    r(1, 1) = pose["rotation"][1][1];
    r(1, 2) = pose["rotation"][1][2];
    r(2, 0) = pose["rotation"][2][0];
    r(2, 1) = pose["rotation"][2][1];
    r(2, 2) = pose["rotation"][2][2];
    c(0) = pose["center"][0];
    c(1) = pose["center"][1];
    c(2) = pose["center"][2];
    sfm_data.poses[key] = openMVG::geometry::Pose3(r, c);
  }

  // Iterate through intrinsics
  const int num_intrinsics = json_intrinsics.size();
  for (int i = 0; i < num_intrinsics; i++) {

    // Retrieve intrinsic
    openMVG::IndexT key = json_intrinsics[i]["key"];
    nlohmann::json intrinsic = json_intrinsics[i]["value"]["ptr_wrapper"]["data"];
    
    sfm_data.intrinsics[key] = std::make_shared<openMVG::cameras::Pinhole_Intrinsic_Radial_K3>(
      intrinsic["width"].get<int>(),
      intrinsic["height"].get<int>(),
      intrinsic["focal_length"].get<double>(),
      intrinsic["principal_point"][0].get<double>(),
      intrinsic["principal_point"][1].get<double>(),
      intrinsic["disto_k3"][0].get<double>(),
      intrinsic["disto_k3"][1].get<double>(),
      intrinsic["disto_k3"][2].get<double>()
    );
  }
*/

  // ==============================================================
  // STAGE 8 - Search each view for hand keypoints
  // ==============================================================
/*
  // Create directory if it doesn't exist
  if (!boost::filesystem::is_directory(output_path / "hand_keypoints")) {
    boost::filesystem::create_directory(output_path / "hand_keypoints");
  }

  // Initialise keypoints variable
  std::stringstream openpose_keypoints;

  // Check for keypoints file
  if (boost::filesystem::exists(output_path / "hand_keypoints" / "2d_keypoints.dat")) {

    // Read file
    std::ifstream keypoints_infile((output_path / "hand_keypoints" / "2d_keypoints.dat").string());
    std::string line;
    while (getline(keypoints_infile, line)) {
      openpose_keypoints << line << std::endl;
    }
    keypoints_infile.close();

  } else {

    // Retrieve openpose root path from env variable
    char * openpose_root = getenv("OPENPOSE_ROOT");
    if (openpose_root == NULL) {
      std::cout << "OPENPOSE_ROOT environment variable not set" << std::endl;
    }

    // Change directory to OpenPose root
    boost::filesystem::path openpose_root_path = boost::filesystem::system_complete(openpose_root);
    boost::filesystem::current_path(openpose_root_path);

    // Only extract keypoints for views with poses and intrinsics
    bool first_detection_iteration = true;
    for (openMVG::IndexT view_i = 0; view_i < num_views; view_i++) {
      std::shared_ptr<openMVG::sfm::View> view = sfm_data.views[view_i];
      if (sfm_data.IsPoseAndIntrinsicDefined(view.get())) {
        
        // Log the current view
        if (first_detection_iteration) {
          first_detection_iteration = false;
        } else {
          std::cout << "\33[2K\r";
        }
        std::cout << "Searching image " << view->s_Img_path << " for keypoints" << std::flush;

        // Execute OpenPose user code file on image
        boost::filesystem::path user_code_path = openpose_root_path / "build" / "examples" / "user_code" / "hand_from_image.bin";
        boost::filesystem::path image_path = input_path / view->s_Img_path;
        std::string openpose_cmd = user_code_path.string() + " -image_path " + image_path.string() + " -no_display true -write_images_format png -write_images " + (output_path / "hand_keypoints" / view->s_Img_path).string();
        std::string openpose_response = exec(openpose_cmd);
        openpose_keypoints << "VIEW ID: " << view_i << std::endl << openpose_response;
      }
    }
    // Print new line
    std::cout << std::endl;

    // Write to file
    std::ofstream keypoints_outfile((output_path / "hand_keypoints" / "2d_keypoints.dat").string());
    keypoints_outfile << openpose_keypoints.str();
    keypoints_outfile.close();
  }

  // Convert keypoints to string for parsing
  std::string openpose_keypoints_string = openpose_keypoints.str();

  // Repeat for number of poses that exist and parse views
  for (int i = 0; i < num_poses; i++) {

    // Move to next view
    std::string view_start_delimiter = "VIEW ID: ";
    openpose_keypoints_string.erase(0, openpose_keypoints_string.find(view_start_delimiter) + view_start_delimiter.length());

    // Extract view id
    openMVG::IndexT view_i = std::stof(openpose_keypoints_string.substr(0, openpose_keypoints_string.find(" ")));

    // Parse keypoints
    std::string data_start_delimiter = "Left hand keypoints: Array<T>::toString():\n";
    openpose_keypoints_string.erase(0, openpose_keypoints_string.find(data_start_delimiter) + data_start_delimiter.length());

    // Loop through each keypoint (line)
    for (openMVG::IndexT keypoint_i = 0; keypoint_i < 21; keypoint_i++) {
      std::string line = openpose_keypoints_string.substr(0, openpose_keypoints_string.find("\n"));
      // Extract keypoint (x, y, score)
      float x = std::stof(line.substr(0, line.find(" ")));
      line.erase(0, line.find(" ") + 1);
      float y = std::stof(line.substr(0, line.find(" ")));
      line.erase(0, line.find(" ") + 1);
      float score = std::stof(line.substr(0, line.find(" ")));
      line.erase(0, line.find(" ") + 1);
      openpose_keypoints_string.erase(0, openpose_keypoints_string.find("\n") + 1);

      // Create Observation and add it to sfm_data track (if score is high enough)
      if (score > 0.2) {
        std::cout << "Inserting observation with keypoint_i " << keypoint_i << " and view_i " << view_i << " -> score " << score << std::endl;
        sfm_data.structure[keypoint_i].obs[view_i] = openMVG::sfm::Observation(openMVG::Vec2(x, y), keypoint_i);
      }
    }
  }
*/

  // ==============================================================
  // STAGE 9 - Locate 21 keypoints in 3D space
  // ==============================================================
/*
  openMVG::sfm::SfM_Data_Structure_Computation_Robust structure_computation_robust = openMVG::sfm::SfM_Data_Structure_Computation_Robust(4.0, 3, 3, true);
  structure_computation_robust.triangulate(sfm_data);


  // Take a look at images/views and manually select 2 to use (I've selected IMG_2215.JPG and IMG_2236.JPG)
  // Find the views for these images (view 9 and view 30)
  // Find the best track/keypoint to triangulate for view 9 and 30 (It is track/keypoint 11)

  std::cout << sfm_data.structure[11].obs[9].x << std::endl;
  std::cout << sfm_data.structure[11].obs[30].x << std::endl;

  std::shared_ptr<openMVG::sfm::View> view_9 = sfm_data.views[9];
  std::shared_ptr<openMVG::sfm::View> view_30 = sfm_data.views[30];

  std::shared_ptr<openMVG::cameras::IntrinsicBase> cam = sfm_data.intrinsics[0];

  openMVG::geometry::Pose3 pose_9 = sfm_data.GetPoseOrDie(view_9.get());
  openMVG::geometry::Pose3 pose_30 = sfm_data.GetPoseOrDie(view_30.get());

  std::vector<openMVG::Vec3> bearing;
  std::vector<openMVG::Mat34> poses;
  bearing.reserve(2);
  poses.reserve(2);

  openMVG::Vec3 bearing_1 = (*cam)(cam->get_ud_pixel(sfm_data.structure[11].obs[9].x));
  openMVG::Vec3 bearing_2 = (*cam)(cam->get_ud_pixel(sfm_data.structure[11].obs[30].x));

  bearing.emplace_back(bearing_1);
  bearing.emplace_back(bearing_2);

  openMVG::Mat34 projection_matrix_1 = pose_9.asMatrix();
  openMVG::Mat34 projection_matrix_2 = pose_30.asMatrix();

  poses.emplace_back(projection_matrix_1);
  poses.emplace_back(projection_matrix_2);

  openMVG::Vec4 Xhomogeneous;

  const Eigen::Map<const openMVG::Mat3X> bearing_matrix(bearing[0].data(), 3, bearing.size());
  
  openMVG::TriangulateNViewAlgebraic (
    bearing_matrix,
    poses, // Ps are projective cameras.
    &Xhomogeneous
  );
*/
/*
  openMVG::TriangulateDLT(projection_matrix_1, bearing_1, projection_matrix_2, bearing_2, &Xhomogeneous);

  sfm_data.structure[11].X = Xhomogeneous.hnormalized();
*/




/*
  // Loop through each keypoint track
  for (openMVG::IndexT track_i = 0; track_i < 21; track_i++) {

    std::cout << "Triangulating keypoint tracks" << std::endl;

    // DEBUG: Print number of observations
    std::cout << "Size: " << sfm_data.structure[track_i].obs.size() << std::endl;

    // Fill the samples set
    std::set<openMVG::IndexT> samples;
    for (size_t i = 0; i < sfm_data.structure[track_i].obs.size(); ++i) {
      samples.insert(i);
    }


    if (samples.size() >= 2 && sfm_data.structure[track_i].obs.size() >= 2) {

      std::vector<openMVG::Vec3> bearing;
      std::vector<openMVG::Mat34> poses;
      bearing.reserve(samples.size());
      poses.reserve(samples.size());

      for (const auto& idx : samples) {

        openMVG::sfm::Observations::const_iterator itObs = sfm_data.structure[track_i].obs.begin();
        std::advance(itObs, idx);
        const openMVG::sfm::View * view = sfm_data.views.at(itObs->first).get();
        if (!sfm_data.IsPoseAndIntrinsicDefined(view)) {
          continue;
        }
        const openMVG::cameras::IntrinsicBase * cam = sfm_data.GetIntrinsics().at(view->id_intrinsic).get();
        const openMVG::geometry::Pose3 pose = sfm_data.GetPoseOrDie(view);
        bearing.emplace_back((*cam)(cam->get_ud_pixel(itObs->second.x)));
        poses.emplace_back(pose.asMatrix());
      }
      if (bearing.size() >= 2) {
        const Eigen::Map<const openMVG::Mat3X> bearing_matrix(bearing[0].data(), 3, bearing.size());
        openMVG::Vec4 Xhomogeneous;
        openMVG::TriangulateNViewAlgebraic (
          bearing_matrix,
          poses, // Ps are projective cameras.
          &Xhomogeneous
        );
        sfm_data.structure[track_i].X = Xhomogeneous.hnormalized();
      }
    } else {
      std::cout << "Not enough keypoint observations to triangulate" << std::endl;
    }
  }
*/
/*
  // Retrieve 3D points
  openMVG::Vec3 hand_keypoint_vertices[21];
  for (openMVG::IndexT i = 0; i < 21; i++) {
    hand_keypoint_vertices[i] = sfm_data.structure[i].X;
  }

  // Write to .ply file
  write_to_ply(hand_keypoint_vertices, 21, (output_path / "hand_keypoints" / "3d_keypoints_dlt.ply").string());
*/
/*
  // ==============================================================
  // STAGE 10 - Extract hand colour range from keypoint sets
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
*/


  // Read the saved point cloud
  // Remove points in dense point cloud not in colour range
  // Save new point cloud

  std::cout << "Done" << std::endl;
  return 0;
}
