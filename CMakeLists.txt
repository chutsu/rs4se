CMAKE_MINIMUM_REQUIRED(VERSION 3.0)
PROJECT(rs4se)

# ADD_COMPILE_OPTIONS(
#   -Wall
#   -Wextra
#   -fsanitize=address
#   -fno-omit-frame-pointer
# )
#
# LINK_LIBRARIES(
#   -fsanitize=address
#   -static-libasan # Use -static-libsan for Clang
# )

# PACKAGES
FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(Eigen3 REQUIRED)
FIND_PACKAGE(realsense2 REQUIRED)
INCLUDE_DIRECTORIES(${realsense2_INCLUDE_DIR})
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})
FIND_PACKAGE(
  catkin
  REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  cv_bridge
  image_transport
)
CATKIN_PACKAGE(LIBRARIES ${PROJECT_NAME})
INCLUDE_DIRECTORIES(${catkin_INCLUDE_DIRS}
                    ${realsense2_INCLUDE_DIR})

# EXECUTABLES
SET(DEPS
  ${realsense2_LIBRARY}
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
)

ADD_EXECUTABLE(intel_d435i src/intel_d435i.cpp)
TARGET_LINK_LIBRARIES(intel_d435i ${DEPS})

ADD_EXECUTABLE(test_rs4se src/test_rs4se.cpp)
TARGET_LINK_LIBRARIES(test_rs4se ${DEPS})
