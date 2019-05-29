cmake_minimum_required(VERSION 3.10.2)
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
add_executable(reconstruct src/pipeline.cpp src/system.cpp)
#add_executable(locate_keypoints_test src/locate_keypoints.cpp src/system.cpp)

set(
  LIBS 
  boost_system boost_filesystem boost_serialization boost_iostreams
  openMVG_easyexif openMVG_exif openMVG_fast openMVG_features openMVG_geometry openMVG_image openMVG_kvld openMVG_lInftyComputerVision openMVG_lemon openMVG_linearProgramming openMVG_matching openMVG_matching_image_collection openMVG_multiview openMVG_numeric openMVG_robust_estimation openMVG_sfm openMVG_stlplus openMVG_system
  tiff jpeg png
  MVS Common IO Math
)

# Link the required libraries
target_link_libraries(reconstruct ${LIBS})
#target_link_libraries(locate_keypoints_test ${LIBS})
