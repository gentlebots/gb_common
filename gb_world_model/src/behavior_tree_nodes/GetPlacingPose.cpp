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

  // 1.- Find object class 
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

  // 2.- Find a node that stores found class 
  std::string stores_prop_value = "";
  std::string storage_node_name = "";

  for (auto node : graph_->get_nodes()) {
      auto node_stores_prop = ros2_knowledge_graph::get_property<std::string>(node, "stores");
      if (node_stores_prop.has_value()) {
        stores_prop_value = node_stores_prop.value();
        if (stores_prop_value == object_class_value) {
          storage_node_name = node.node_name;
          //storage_node_name = node.get_node_names()[0];
          break;
        }
      }
  }

  if (storage_node_name == "") {
    std::cerr << "Couldn't find nodes with \"stores\" property equal to ["
         << object_class_value << "] in the graph" << std::endl;
    return BT::NodeStatus::FAILURE;
  }

  // 3.- Get the position of that node 
  auto storage_node = graph_->get_node(storage_node_name);

  if (!storage_node.has_value()) { 
    std::cerr << "Storage Node [" << storage_node_name << "] not found" << std::endl;
    return BT::NodeStatus::FAILURE;
  }

  auto storage_node_pose_property = ros2_knowledge_graph::get_property<geometry_msgs::msg::PoseStamped>(storage_node.value(), "position");

  if (!storage_node_pose_property.has_value()) {
    std::cerr << "[" << storage_node_name << "] property \"position\" not found" << std::endl;
    return BT::NodeStatus::FAILURE;
  }

  // 4.- Set the ouput 
  setOutput("ob_pose", storage_node_pose_property.value());
  return BT::NodeStatus::SUCCESS;

}

}  // namespace gb_world_model

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<gb_world_model::GetPlacingPose>("GetPlacingPose");
}
