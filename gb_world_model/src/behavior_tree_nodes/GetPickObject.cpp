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
#include "gb_world_model/behavior_tree_nodes/GetPickObject.hpp"
#include "ros2_knowledge_graph/graph_utils.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"

namespace gb_world_model
{

GetPickObject::GetPickObject(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf), counter_(0)
{
  auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  graph_ = ros2_knowledge_graph::GraphFactory::getInstance(node);
}

void
GetPickObject::halt()
{
}

BT::NodeStatus
GetPickObject::tick()
{
  
  // std::cerr << " [GetPickObject] WARNING: ADDING DEBUG CODE!" << std::endl;

  // //add node jarvis
  // // auto node_1 = ros2_knowledge_graph::new_node("jarvis", "robot");
  // // graph_->update_node(node_1);

  // //add node sugar_box, class food_item
  // auto node_2 = ros2_knowledge_graph::new_node("sugar_box", "food_items");
  
  // //add property class food_item
  // std::cerr << " [GetPickObject] adding class? property!" << std::endl;
  // ros2_knowledge_graph::add_property<std::string>(node_2, "class","food_items");

  // //add property position 
  // geometry_msgs::msg::PoseStamped poseSt;
  // poseSt.pose.position.x = 7.0;  
  // poseSt.pose.position.y = 7.0;
  // poseSt.header.frame_id = "map";
  // std::cerr << " [GetPickObject] adding pose property!" << std::endl;
  // ros2_knowledge_graph::add_property<geometry_msgs::msg::PoseStamped>(node_2, "position",poseSt);
  // std::cerr << " [GetPickObject] updating node!" << std::endl;
  // graph_->update_node(node_2);

  // //add edge pick jarvis sugar_box 
  // std::cerr << " [GetPickObject] creating edge" << std::endl;
  // auto edge_pick = ros2_knowledge_graph::new_edge<std::string>("jarvis", "sugar_box", "pick");
  // std::cerr << " [GetPickObject] updating edge" << std::endl;
  // bool ans = graph_->update_edge(edge_pick);
  // std::cerr << " [GetPickObject] edge was: " << ans << std::endl;
  

  // std::cerr << " [GetPickObject] give it time" << std::endl;
  // rclcpp::spin_some(config().blackboard->get<rclcpp::Node::SharedPtr>("node"));
  // std::cerr << " [GetPickObject] go!" << std::endl;





  std::string object_id;

  auto edges_by_data = graph_->get_edges_from_node_by_data("jarvis", "pick");

  if (edges_by_data.size() > 1 || edges_by_data.size() == 0)
  {
    std::cerr << " [GetPickObject] Error: I found [" << edges_by_data.size() << "] pick edges from jarvis node. I should be exactly 1:" << std::endl;
    for (auto edge : edges_by_data)
    {
      std::cerr << " [" << ros2_knowledge_graph::to_string(edge) << "] " << std::endl;
    }
    return BT::NodeStatus::FAILURE;
  }

  for (auto edge : edges_by_data)
  {
    object_id = edge.target_node_id;
  }

  auto obj_graph_node = graph_->get_node(object_id);

  if (obj_graph_node.has_value()) {
    auto pose = ros2_knowledge_graph::get_property<geometry_msgs::msg::PoseStamped>(
      obj_graph_node.value(), "position");

    if (pose.has_value()) {
      //setOutput("wp_pose", pose.value());
      setOutput("object_id", object_id);
      return BT::NodeStatus::SUCCESS;
    } else {
      std::cerr << "[GetPickObject] [" << object_id << "] does not have position property" << std::endl;
      return BT::NodeStatus::FAILURE;
    }
  } else {
    std::cerr << " [GetPickObject] WP Node [" << object_id << "] not found" << std::endl;
    return BT::NodeStatus::FAILURE;
  }


}

}  // namespace gb_world_model

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<gb_world_model::GetPickObject>("GetPickObject");
}
