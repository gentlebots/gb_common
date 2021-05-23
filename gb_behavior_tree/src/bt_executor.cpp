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

#include <string>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/utils/shared_library.h"

#include "rclcpp/rclcpp.hpp"


/* bt_executor: A program that executes a behavior tree
 *
 * Requires two ROS2 params:
 *     - bt_xml_file [string]: The full path of the bheavior tree to execute
 *     - plugins [string[]]: The names of the bt nodes used in the behavior tree
 *
 * It is possible to pass extra parameters by command line (via argv) that will be inserted
 * in the blackboard with the ids arg1, arg2, ..., argN
*/

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto bt_executor_node = rclcpp::Node::make_shared("bt_executor_node");
  bt_executor_node->declare_parameter("bt_xml_file");
  bt_executor_node->declare_parameter("plugins");

  BT::BehaviorTreeFactory factory;
  BT::SharedLibrary loader;

  auto plugin_lib_names = bt_executor_node->get_parameter("plugins").as_string_array();
  for (auto plugin : plugin_lib_names) {
    RCLCPP_INFO_STREAM(bt_executor_node->get_logger(), "plugin: [" << plugin << "]");
    factory.registerFromPlugin(loader.getOSName(plugin));
  }

  BT::Blackboard::Ptr blackboard;

  blackboard = BT::Blackboard::create();
  blackboard->set("node", bt_executor_node);
  int arg_counter = 0;
  for (int i = 1; i < argc; i++) {
    if (argv[i][0] == '-') {
      break;
    }
    std::string argname = "arg" + std::to_string(arg_counter++);
    blackboard->set(argname, argv[i]);
    RCLCPP_INFO_STREAM(
      bt_executor_node->get_logger(),
      "set " << argname << " = [" << argv[i] << "]");
  }

  std::string bt_xml_file;
  bt_xml_file = bt_executor_node->get_parameter("bt_xml_file").value_to_string();
  RCLCPP_INFO_STREAM(bt_executor_node->get_logger(), "bt_xml_file: [" << bt_xml_file << "]");

  BT::Tree tree;
  tree = factory.createTreeFromFile(bt_xml_file, blackboard);

  bool finished = false;
  BT::NodeStatus result = BT::NodeStatus::IDLE;

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(bt_executor_node);

  rclcpp::Rate rate(30);
  while (rclcpp::ok() && !finished) {
    result = tree.rootNode()->executeTick();
    finished = result != BT::NodeStatus::RUNNING;

    rate.sleep();
    executor.spin_some();
  }

  switch (result) {
    case BT::NodeStatus::SUCCESS:
      RCLCPP_INFO(bt_executor_node->get_logger(), "Finished with SUCCESS value");
      break;
    case BT::NodeStatus::IDLE:
      RCLCPP_INFO(bt_executor_node->get_logger(), "Finished with IDLE value");
      break;
    case BT::NodeStatus::RUNNING:
      RCLCPP_INFO(bt_executor_node->get_logger(), "Finished with RUNNING value");
      break;
    case BT::NodeStatus::FAILURE:
      RCLCPP_INFO(bt_executor_node->get_logger(), "Finished with FAILURE value");
      break;
  }

  rclcpp::shutdown();
  return 0;
}
