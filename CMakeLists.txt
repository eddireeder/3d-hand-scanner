cmake_minimum_required(VERSION 3.13.2)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Insert path to include headers
include_directories("include")
include_directories("/usr/local/include/opencv4")
include_directories("/usr/local/include")

# Insert path to libraries
link_directories("/usr/local/lib")
link_directories("/usr/local/lib/OpenMVS")

# Create executable
#add_executable(reconstruct reconstruct.cpp)
add_executable(locate_keypoints_test src/locate_keypoints.cpp src/system.cpp)

set(
  LIBS 
  opencv_stitching opencv_superres opencv_videostab opencv_aruco opencv_bgsegm opencv_bioinspired opencv_ccalib opencv_dnn_objdetect opencv_dpm opencv_face opencv_photo opencv_fuzzy opencv_hfs opencv_img_hash opencv_line_descriptor opencv_optflow opencv_reg opencv_rgbd opencv_saliency opencv_stereo opencv_structured_light opencv_phase_unwrapping opencv_surface_matching opencv_tracking opencv_datasets opencv_dnn opencv_plot opencv_xfeatures2d opencv_shape opencv_video opencv_ml opencv_ximgproc opencv_calib3d opencv_features2d opencv_highgui opencv_videoio opencv_flann opencv_xobjdetect opencv_imgcodecs opencv_objdetect opencv_xphoto opencv_imgproc opencv_core
  openpose openpose_3d openpose_calibration openpose_core openpose_face openpose_filestream openpose_gpu openpose_gui openpose_hand openpose_net openpose_pose openpose_producer openpose_thread openpose_tracking openpose_unity openpose_utilities openpose_wrapper
  boost_system boost_filesystem
  openMVG_easyexif openMVG_exif openMVG_fast openMVG_features openMVG_geometry openMVG_image openMVG_kvld openMVG_lInftyComputerVision openMVG_lemon openMVG_linearProgramming openMVG_matching openMVG_matching_image_collection openMVG_multiview openMVG_numeric openMVG_robust_estimation openMVG_sfm openMVG_stlplus openMVG_system
  Common.a IO.a MVS.a Math.a
)

# Link the required libraries
#target_link_libraries(reconstruct ${LIBS})
target_link_libraries(locate_keypoints_test ${LIBS})