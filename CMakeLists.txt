cmake_minimum_required( VERSION 2.8 )
project( Mobile_Sensing )

set( CMAKE_BUILD_TYPE "Release" )
MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

# set( CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native " )
# set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 -march=native" )
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

# 添加cmake模块以使用g2o
list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )

# 搜索CSpaese
#find_package(CSparse REQUIRED)

# 搜索cholmod
#find_package(Cholmod REQUIRED)

# 搜索opencv
find_package(OpenCV 3.0 QUIET)
if(NOT OpenCV_FOUND)
   find_package(OpenCV 2.4.3 QUIET)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
   endif()
endif()

# 搜索pcl
find_package( PCL 1.9 REQUIRED )
if( PCL_FOUND )
  # [C/C++]>[General]>[Additional Include Directories]
  include_directories( ${PCL_INCLUDE_DIRS} )
  # [Linker]>[General]>[Additional Library Directories]
  link_directories( ${PCL_LIBRARY_DIRS} )
  # [C/C++]>[Preprocessor]>[Preprocessor Definitions]
  add_definitions( ${PCL_DEFINITIONS} )
endif()

# 搜索Eigen
find_package( Eigen3 REQUIRED )

# 搜索Pangolin
find_package( Pangolin REQUIRED )

# 添加库
include_directories( 
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/include
${Pangolin_INCLUDE_DIRS}
${OPENCV_INCLUDE_DIR}
${PCL_INCLUDE_DIRS}
${EIGEN3_INCLUDE_DIR} 
)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

# 将类添加为库文件
add_library(${PROJECT_NAME} SHARED
src/system.cpp
src/datareader.cpp
src/spanpose.cpp
src/projection_params.cpp
src/mapping.cpp
src/frame_drawer.cpp
src/map_drawer.cpp
src/viewer.cpp
src/depth_ground_remover.cpp
src/image_labelers/abstract_image_labeler.cpp
src/image_labelers/diff_helpers/angle_diff.cpp
src/image_labelers/diff_helpers/line_dist_diff.cpp
src/scan.cpp
src/scan_projection.cpp
src/rich_point.cpp
src/utils.cpp
src/mcgicpIntensity.cpp
)

target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
${PCL_LIBRARIES}
${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
)

# Build examples
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

# add_executable( feature_extraction bin/feature_extraction.cpp  )
# target_link_libraries( feature_extraction ${PROJECT_NAME} )

# add_executable( pose_estimation_2d2d bin/pose_estimation_2d2d.cpp extra.cpp ) # use this if in OpenCV2 
# add_executable( pose_estimation_2d2d bin/pose_estimation_2d2d.cpp )
# target_link_libraries( pose_estimation_2d2d ${PROJECT_NAME} )

# add_executable( triangulation bin/triangulation.cpp extra.cpp) # use this if in opencv2 
# add_executable( triangulation bin/triangulation.cpp )
# target_link_libraries( triangulation ${PROJECT_NAME} )

# add_executable( pose_estimation_3d2d bin/pose_estimation_3d2d.cpp )
# target_link_libraries( pose_estimation_3d2d ${PROJECT_NAME})

# add_executable( pose_estimation_3d3d bin/pose_estimation_3d3d.cpp )
# target_link_libraries( pose_estimation_3d3d ${PROJECT_NAME})

# add_executable( pose_estimation_mcgicp bin/pose_estimation_mcgicp.cpp )
# target_link_libraries( pose_estimation_mcgicp ${PROJECT_NAME})

add_executable( mobile_mapping bin/mobile_mapping.cpp )
target_link_libraries( mobile_mapping ${PROJECT_NAME})

# add_executable( bin_capture bin/bin_capture.cpp )
# target_link_libraries( bin_capture ${PROJECT_NAME})

# add_executable( pcap2bin bin/pcap2bin.cpp )
# target_link_libraries( pcap2bin ${PROJECT_NAME})

# add_executable( txt2txt bin/txt2txt.cpp )
# target_link_libraries( txt2txt ${PROJECT_NAME})

add_executable( depth_image bin/depth_image.cpp )
target_link_libraries( depth_image ${PROJECT_NAME})

add_executable( deskew_eval bin/deskew_eval.cpp )
target_link_libraries( deskew_eval ${PROJECT_NAME})