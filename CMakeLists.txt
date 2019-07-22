CMAKE_MINIMUM_REQUIRED(VERSION 3.0)
PROJECT(rs4se)

# PACKAGES
FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(Eigen3 REQUIRED)
FIND_PACKAGE(OpenGL REQUIRED)
FIND_PACKAGE(assimp REQUIRED)
FIND_PACKAGE(glm REQUIRED)
FIND_PACKAGE(glfw3 REQUIRED)
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})
INCLUDE_DIRECTORIES(${OPENGL_INCLUDE_DIR})

# -- Imgui
INCLUDE_DIRECTORIES(dep/imgui dep/imgui/examples)
ADD_DEFINITIONS(-DIMGUI_IMPL_OPENGL_LOADER_GLAD)
ADD_LIBRARY(
  imgui
  dep/imgui/imgui.cpp
  dep/imgui/imgui_draw.cpp
  dep/imgui/imgui_demo.cpp
  dep/imgui/imgui_widgets.cpp
)
ADD_LIBRARY(
  imgui_impl
  dep/imgui/examples/imgui_impl_glfw.cpp
  dep/imgui/examples/imgui_impl_opengl3.cpp
)

# -- GLAD
LINK_DIRECTORIES(dep/glad/build)
INCLUDE_DIRECTORIES(dep/glad/include)

# -- Catkin
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
INCLUDE_DIRECTORIES(${catkin_INCLUDE_DIRS})

# EXECUTABLES
SET(DEPS
  realsense2
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
  glad
  glfw
  dl
  ${ASSIMP_LIBRARIES}
  ${OPENGL_gl_LIBRARY}
  imgui
  imgui_impl
)

ADD_EXECUTABLE(intel_d435i src/intel_d435i.cpp)
TARGET_LINK_LIBRARIES(intel_d435i ${DEPS})

ADD_EXECUTABLE(gui src/gui.cpp)
TARGET_LINK_LIBRARIES(gui ${DEPS})
