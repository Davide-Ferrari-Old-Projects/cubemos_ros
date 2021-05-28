cmake_minimum_required(VERSION 2.8.3)
project(cubemos_ros LANGUAGES CXX)
set (CMAKE_CXX_STANDARD 11)

find_package(CUBEMOS_SKELETON_TRACKING
			REQUIRED
			CONFIG
			PATHS "/opt/cubemos/skeleton_tracking/samples/../lib/cmake/CUBEMOS_SKELETON_TRACKING"
			)

set(OpenCV_DIR "/opt/cubemos/skeleton_tracking/samples/dep/opencv/lib/cmake/opencv4")
message("PATH: ${OpenCV_DIR}")
find_package(OpenCV REQUIRED CONFIG)
message("OpenCV version: ${OpenCV_VERSION}")

find_package(realsense2 
			REQUIRED 
			CONFIG 
			PATHS "/opt/cubemos/skeleton_tracking/samples//dep/realsense/lib/cmake/realsense2"
			)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  geometry_msgs
)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN3_DEFINITIONS})

add_message_files(
  FILES
  cubemos_body.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_executable(${PROJECT_NAME} src/cubemos.cpp src/filters/KalmanLin.cpp src/filters/KalmanRot.cpp src/filters/MediumFilt.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME} cubemos_skeleton_tracking)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
target_link_libraries(${PROJECT_NAME} ${realsense2_LIBRARY})
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(${PROJECT_NAME}2 src/cubemos2.cpp)
target_link_libraries(${PROJECT_NAME}2 ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}2 cubemos_skeleton_tracking)
target_link_libraries(${PROJECT_NAME}2 ${OpenCV_LIBS})
target_link_libraries(${PROJECT_NAME}2 ${realsense2_LIBRARY})
add_dependencies(${PROJECT_NAME}2 ${${PROJECT_NAME}_EXPORTED_TARGETS})

