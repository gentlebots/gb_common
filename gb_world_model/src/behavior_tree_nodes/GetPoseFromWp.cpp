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
#include "gb_world_model/behavior_tree_nodes/GetPoseFromWp.hpp"
#include "ros2_knowledge_graph/graph_utils.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"

namespace gb_world_model
{

GetPoseFromWp::GetPoseFromWp(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf), counter_(0)
{
  auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  graph_ = ros2_knowledge_graph::GraphFactory::getInstance(node);
}

void
GetPoseFromWp::halt()
{
}

BT::NodeStatus
GetPoseFromWp::tick()
{
  std::string wp_id;
  getInput<std::string>("wp_id", wp_id);

  std::cerr << "Getting [" << wp_id << "]" << std::endl;

  auto wp_graph_node = graph_->get_node(wp_id);

  if (wp_graph_node.has_value()) {
    auto pose = ros2_knowledge_graph::get_property<geometry_msgs::msg::PoseStamped>(
      wp_graph_node.value(), "position");

    if (pose.has_value()) {
      std::cerr << "[" << wp_id << "] (" << pose.value().pose.position.x << 
        ", " << pose.value().pose.position.y << ")" << std::endl;
      setOutput("wp_pose", pose.value());
      return BT::NodeStatus::SUCCESS;
    } else {
      std::cerr << "position prop at [" << wp_id << "] not found" << std::endl;
      return BT::NodeStatus::SUCCESS;
    }
  } else {
    std::cerr << "WP Node [" << wp_id << "] not found" << std::endl;
    return BT::NodeStatus::SUCCESS;
  }


}

}  // namespace gb_world_model

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<gb_world_model::GetPoseFromWp>("GetPoseFromWp");
}
