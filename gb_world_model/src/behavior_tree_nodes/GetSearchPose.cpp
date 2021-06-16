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
#include "gb_world_model/behavior_tree_nodes/GetSearchPose.hpp"
#include "ros2_knowledge_graph/graph_utils.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"

namespace gb_world_model
{

GetSearchPose::GetSearchPose(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  graph_ = ros2_knowledge_graph::GraphFactory::getInstance(node_);
}

void
GetSearchPose::halt()
{
}

BT::NodeStatus
GetSearchPose::tick()
{
  auto search_node = graph_->get_node("search_area");
  
  if (search_node.has_value()) {
    auto pose = ros2_knowledge_graph::get_property<geometry_msgs::msg::PoseStamped>(
      search_node.value(), "position");
    if (pose.has_value()) {
      geometry_msgs::msg::PoseStamped p_value;
      p_value = pose.value();
      p_value.pose.position.z = 0.0;
      setOutput("wp_pose", p_value);
      return BT::NodeStatus::SUCCESS;
    } else {
      std::cerr << "position prop at [search_area] not found" << std::endl;
      return BT::NodeStatus::FAILURE;
    }
  } else {
    std::cerr << "Node [search_area] not found" << std::endl;
  }
  
  rclcpp::spin_some(node_);
  return BT::NodeStatus::RUNNING;
}

}  // namespace gb_world_model

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<gb_world_model::GetSearchPose>("GetSearchPose");
}
