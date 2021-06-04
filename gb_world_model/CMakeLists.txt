cmake_minimum_required(VERSION 3.5)
project(gb_world_model)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros2_knowledge_graph REQUIRED)
find_package(ros2_knowledge_graph_plugins REQUIRED)
find_package(plansys2_domain_expert REQUIRED)
find_package(plansys2_problem_expert REQUIRED)
find_package(pluginlib REQUIRED)

set(dependencies
  rclcpp
  ros2_knowledge_graph
  ros2_knowledge_graph_plugins
  plansys2_domain_expert
  plansys2_problem_expert
  pluginlib
)

include_directories(
  include
)

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