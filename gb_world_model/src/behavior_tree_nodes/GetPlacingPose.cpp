// Copyright 2019 Intelligent Robotics Lab
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

#include "ros2_knowledge_graph/GraphNode.hpp"
#include "gb_world_model/behavior_tree_nodes/GetPlacingPose.hpp"
#include "ros2_knowledge_graph/graph_utils.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"

namespace gb_world_model
{

GetPlacingPose::GetPlacingPose(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{
  auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  graph_ = ros2_knowledge_graph::GraphFactory::getInstance(node);
}

void
GetPlacingPose::halt()
{
}

BT::NodeStatus
GetPlacingPose::tick()
{
  std::string object;
  getInput<std::string>("object", object);

  std::cerr << "Looking for object [" << object << "] placing pose" << std::endl;

  auto object_graph_node = graph_->get_node(object);

  if (!object_graph_node.has_value()) { 
    std::cerr << "Object Node [" << object << "] not found" << std::endl;
    return BT::NodeStatus::FAILURE;
  }

  auto object_class = ros2_knowledge_graph::get_property<std::string>(object_graph_node.value(), "class");

  if (!object_class.has_value()) {
    std::cerr << "[" << object << "] property \"class\" not found" << std::endl;
    return BT::NodeStatus::FAILURE;
  }

  std::string object_class_value = object_class.value();

  std::string node_stores_value = "";

  for (auto node : graph_->get_nodes()) {
      auto node_stores = ros2_knowledge_graph::get_property<std::string>(node, "stores");
      if (node_stores.has_value()) {
        node_stores_value = node_stores.value();
        setOutput("ob_pose", node_stores_value);
        return BT::NodeStatus::SUCCESS;
      }
  }

  std::cerr << "Couldn't find objects with \"class\" property equal to ["
       << object_class_value << "] in the graph" << std::endl;
  return BT::NodeStatus::FAILURE;

}

}  // namespace gb_world_model

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<gb_world_model::GetPlacingPose>("GetPlacingPose");
}
