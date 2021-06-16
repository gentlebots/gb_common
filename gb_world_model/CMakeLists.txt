cmake_minimum_required(VERSION 3.5)
project(gb_world_model)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros2_knowledge_graph REQUIRED)
find_package(plansys2_domain_expert REQUIRED)
find_package(plansys2_problem_expert REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(vision_msgs REQUIRED)

set(dependencies
  rclcpp
  ros2_knowledge_graph
  plansys2_domain_expert
  plansys2_problem_expert
  behaviortree_cpp_v3
  vision_msgs
)

include_directories(
  include
)
add_library(gb_get_pose_from_wp_bt_node SHARED src/behavior_tree_nodes/GetPoseFromWp.cpp)
list(APPEND plugin_libs gb_get_pose_from_wp_bt_node)

add_library(gb_get_pose_from_object_bt_node SHARED src/behavior_tree_nodes/GetPoseFromObject.cpp)
list(APPEND plugin_libs gb_get_pose_from_object_bt_node)

add_library(gb_get_pick_object_bt_node SHARED src/behavior_tree_nodes/GetPickObject.cpp)
list(APPEND plugin_libs gb_get_pick_object_bt_node)

add_library(gb_get_placing_pose_bt_node SHARED src/behavior_tree_nodes/GetPlacingPose.cpp)
list(APPEND plugin_libs gb_get_placing_pose_bt_node)

foreach(bt_plugin ${plugin_libs})
  ament_target_dependencies(${bt_plugin} ${dependencies})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

add_library(${PROJECT_NAME} SHARED
  src/gb_world_model/WorldModelNode.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

add_executable(world_model_main
  src/world_model_main.cpp
)
ament_target_dependencies(world_model_main ${dependencies})
target_link_libraries(world_model_main ${PROJECT_NAME})

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

install(TARGETS
  ${PROJECT_NAME}
  world_model_main
  ${plugin_libs}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_index_cpp REQUIRED)

  add_subdirectory(tests)
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})

ament_package()