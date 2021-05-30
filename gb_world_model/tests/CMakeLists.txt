ament_add_gtest(test_world_model test_world_model.cpp)
ament_target_dependencies(test_world_model ${dependencies})
target_link_libraries(test_world_model ${PROJECT_NAME})