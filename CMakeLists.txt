SET(PROJECT_NAME euroc_elas)
PROJECT(${PROJECT_NAME})
CMAKE_MINIMUM_REQUIRED (VERSION 2.8.3)

SET(CMAKE_VERBOSE_MAKEFILE OFF)
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/CMakeModules/")

# Set definitions
IF(TRACE)
  ADD_DEFINITIONS(-DSVO_TRACE)
ENDIF()
IF(HAVE_G2O)
  ADD_DEFINITIONS(-DUSE_BUNDLE_ADJUSTMENT)
ENDIF()

IF(DEBUG_OUTPUT)
  ADD_DEFINITIONS(-DSVO_DEBUG_OUTPUT)
ENDIF()



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


# Include dirs
INCLUDE_DIRECTORIES(
  include
  EfficientLargeScaleStereo
  ${Eigen_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${BRISK_INCLUDE_DIRS}
)



# Set sourcefiles
LIST(APPEND SOURCEFILES
src/Struct_definition.cpp
EfficientLargeScaleStereo/descriptor.cpp
EfficientLargeScaleStereo/elas.cpp
EfficientLargeScaleStereo/filter.cpp
EfficientLargeScaleStereo/matrix.cpp
EfficientLargeScaleStereo/triangle.cpp
  
)

# Create svo library
ADD_LIBRARY(euroc_elas  ${SOURCEFILES})

TARGET_LINK_LIBRARIES(euroc_elas 
  ${OpenCV_LIBS}
  ${GLOG_LIBRARY}
  ${Boost_LIBRARIES}
 
  ${Pangolin_LIBRARIES}
  ${BRISK_LIBRARIES}
  

)

ADD_EXECUTABLE(main src/main.cpp)
TARGET_LINK_LIBRARIES(main euroc_elas)



ADD_EXECUTABLE(test test/test.cpp)
TARGET_LINK_LIBRARIES(test euroc_elas)


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
