// Copyright 2021 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#include "gb_world_model/WorldModelNode.hpp"

#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include "gtest/gtest.h"



TEST(TerminalTestCase, test_server)
{
  auto test_node = std::make_shared<gb_world_model::WorldModelNode>();
  auto graph = std::make_shared<ros2_knowledge_graph::GraphNode>(test_node);

  rclcpp::executors::SingleThreadedExecutor exe;

  exe.add_node(test_node);

  std::thread t([&]() {
      exe.spin();
    });



  exe.cancel();
  t.join();
}


int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  return RUN_ALL_TESTS();
}
