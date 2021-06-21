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
#include "gb_world_model/behavior_tree_nodes/SelectPick.hpp"
#include "ros2_knowledge_graph/graph_utils.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"

namespace gb_world_model
{

SelectPick::SelectPick(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  graph_ = ros2_knowledge_graph::GraphFactory::getInstance(node_);
  robot_ = "jarvis";
}

void
SelectPick::halt()
{
}

BT::NodeStatus
SelectPick::tick()
{
  rclcpp::spin_some(node_);
  std::string object_id;
  auto edges_by_data = graph_->get_edges_from_node_by_data(robot_, "perceived");

  if (edges_by_data.size() == 0)
  {
    std::cerr << " [SelectPick] Error: I found [" << edges_by_data.size() << "] perceived edges from jarvis node. Retrying..." << std::endl;
    for (auto edge : edges_by_data)
    {
      std::cerr << " [" << ros2_knowledge_graph::to_string(edge) << "] " << std::endl;
    }
    return BT::NodeStatus::RUNNING;
  }

  for (auto edge : edges_by_data)
  {
    object_id = edge.target_node_id;
  }

  auto edge_pick_obj = ros2_knowledge_graph::new_edge<std::string>(
    robot_, object_id, "pick");
  graph_->update_edge(edge_pick_obj);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace gb_world_model

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<gb_world_model::SelectPick>("SelectPick");
}
