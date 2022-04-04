cmake_minimum_required(VERSION 3.0.2)
project(object_detection_ros)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  visualization_msgs
  tf2_ros
  message_generation
  cv_bridge
  image_transport
  sensor_msgs
  tf2_geometry_msgs
)


find_package(realsense2 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Torch REQUIRED)


add_message_files(
  FILES
  arrbbox.msg
  bbox.msg
  Object.msg
  ObjectArray.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)


## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES object_detection_ros
  CATKIN_DEPENDS  message_generation   
                  geometry_msgs
                  roscpp
                  visualization_msgs
                  tf2_ros
                  message_generation
                  cv_bridge
                  image_transport
                  sensor_msgs
#  DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(vision_pipeline src/visionpipeline.cpp)
target_link_libraries(vision_pipeline  ${realsense2_LIBRARY} ${OpenCV_LIBRARIES} ${TORCH_LIBRARIES})

add_executable(depth_node src/depth.cpp)
add_executable(object_detection_from_depth src/detectornode.cpp src/object_detection_node.cpp)
add_executable(object_handler_node src/object_handler_node.cpp)

add_dependencies(depth_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(object_detection_from_depth ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(object_handler_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Specify libraries to link a library or executable target against
target_link_libraries(depth_node
  ${catkin_LIBRARIES}
  vision_pipeline
)
target_link_libraries(object_detection_from_depth
  ${catkin_LIBRARIES}
  vision_pipeline
)

target_link_libraries(object_handler_node
  ${catkin_LIBRARIES}
  vision_pipeline
)


