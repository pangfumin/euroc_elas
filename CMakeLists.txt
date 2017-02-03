SET(PROJECT_NAME euroc_elas)
PROJECT(${PROJECT_NAME})
CMAKE_MINIMUM_REQUIRED (VERSION 2.8.3)

SET(CMAKE_VERBOSE_MAKEFILE OFF)
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/CMakeModules/")



SET(CMAKE_BUILD_TYPE Debug)  


# Set build flags, set ARM_ARCHITECTURE environment variable on Odroid
SET(CMAKE_CXX_FLAGS "-Wall -D_LINUX -D_REENTRANT -march=native -Wno-unused-variable -Wno-unused-but-set-variable -Wno-unknown-pragmas")
IF(DEFINED ENV{ARM_ARCHITECTURE})
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mfpu=neon -march=armv7-a")
ELSE()
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mmmx -msse -msse -msse2 -msse3 -mssse3")
ENDIF()
IF(CMAKE_COMPILER_IS_GNUCC)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
ELSE()
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
ENDIF()
SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -O3 -fsee -fomit-frame-pointer -fno-signed-zeros -fno-math-errno -funroll-loops")

# Add plain cmake packages 
FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(Eigen REQUIRED)
FIND_PACKAGE(Boost COMPONENTS filesystem system thread REQUIRED)
find_package( brisk 2 REQUIRED )



# glog
find_package(Glog REQUIRED)
include_directories(BEFORE ${GLOG_INCLUDE_DIRS}) 



# add Pangolin for visualization
find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


# pcl 
find_package( PCL REQUIRED COMPONENT common io )
include_directories( ${PCL_INCLUDE_DIRS} )
add_definitions( ${PCL_DEFINITIONS} )

# octomap
find_package(octomap REQUIRED)
include_directories(${OCTOMAP_INCLUDE_DIRS})


# Include dirs
INCLUDE_DIRECTORIES(
  include
  elas
  ${Eigen_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${BRISK_INCLUDE_DIRS}
)



# Set sourcefiles
LIST(APPEND SOURCEFILES
src/Struct_definition.cpp
elas/descriptor.cpp
elas/elas.cpp
elas/filter.cpp
elas/matrix.cpp
elas/triangle.cpp
  
)

# Create svo library
ADD_LIBRARY(euroc_elas  ${SOURCEFILES})

TARGET_LINK_LIBRARIES(euroc_elas 
  ${OpenCV_LIBS}
  ${GLOG_LIBRARY}
  ${Boost_LIBRARIES}
  ${PCL_LIBRARIES}
  ${Pangolin_LIBRARIES}
  ${BRISK_LIBRARIES}
  

)

include_directories(occmapping/inc)
# Set sourcefiles
LIST(APPEND Occmapping_SOURCEFILES
occmapping/src/probabilitylookup.cpp
occmapping/src/robustoctree.cpp
occmapping/src/robustoctreenode.cpp
)


# Create occmapping library
ADD_LIBRARY(occmapping  ${Occmapping_SOURCEFILES})

TARGET_LINK_LIBRARIES(occmapping
${OCTOMAP_LIBRARIES}
)




ADD_EXECUTABLE(main src/main.cpp)
TARGET_LINK_LIBRARIES(main euroc_elas)



ADD_EXECUTABLE(disparity2pointcloud tools/disparity2pointcloud.cpp)
TARGET_LINK_LIBRARIES(disparity2pointcloud euroc_elas)



ADD_EXECUTABLE(saveRectifyPGM tools/saveRectifyPGM.cpp)
TARGET_LINK_LIBRARIES(saveRectifyPGM euroc_elas)


ADD_EXECUTABLE(createDPF tools/createDPF.cpp)
TARGET_LINK_LIBRARIES(createDPF euroc_elas)

ADD_EXECUTABLE(playDPF tools/playDPF.cpp)
TARGET_LINK_LIBRARIES(playDPF euroc_elas)



ADD_EXECUTABLE(disparity2Octree tools/disparity2Octree.cpp)
TARGET_LINK_LIBRARIES(disparity2Octree euroc_elas ${OCTOMAP_LIBRARIES})


ADD_EXECUTABLE(disparity2Occmapping tools/disparity2Occmapping.cpp)
TARGET_LINK_LIBRARIES(disparity2Occmapping euroc_elas occmapping ${OCTOMAP_LIBRARIES})



# TESTS
if (0)

ADD_EXECUTABLE(test_feature_align test/test_feature_alignment.cpp)
TARGET_LINK_LIBRARIES(test_feature_align svo)

ADD_EXECUTABLE(test_pipeline test/test_pipeline.cpp)
TARGET_LINK_LIBRARIES(test_pipeline svo)

ADD_EXECUTABLE(test_matcher test/test_matcher.cpp)
TARGET_LINK_LIBRARIES(test_matcher svo)

ADD_EXECUTABLE(test_feature_detection test/test_feature_detection.cpp)
TARGET_LINK_LIBRARIES(test_feature_detection svo)

ADD_EXECUTABLE(test_depth_filter test/test_depth_filter.cpp)
TARGET_LINK_LIBRARIES(test_depth_filter svo)

ADD_EXECUTABLE(test_sparse_img_align test/test_sparse_img_align.cpp)
TARGET_LINK_LIBRARIES(test_sparse_img_align svo)

ADD_EXECUTABLE(test_pose_optimizer test/test_pose_optimizer.cpp)
TARGET_LINK_LIBRARIES(test_pose_optimizer svo) 

endif()
