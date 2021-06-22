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
#include "gb_world_model/behavior_tree_nodes/GetSearchObject.hpp"
#include "ros2_knowledge_graph/graph_utils.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"

namespace gb_world_model
{

GetSearchObject::GetSearchObject(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  message_sub_ = node_->create_subscription<std_msgs::msg::String>(
    "/message", 1, std::bind(&GetSearchObject::messageCB, this, _1));
}

void
GetSearchObject::halt()
{
}


void 
GetSearchObject::messageCB(const std_msgs::msg::String::SharedPtr msg)
{
  received_msg_ = (msg->data);
}


BT::NodeStatus
GetSearchObject::tick()
{
  

  if (received_msg_ == "")
  {
    return BT::NodeStatus::RUNNING;
  }
  else 
  {
    setOutput("object_id", received_msg_);
      return BT::NodeStatus::SUCCESS;
    return BT::NodeStatus::SUCCESS;
  } 

}

}  // namespace gb_world_model

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<gb_world_model::GetSearchObject>("GetSearchObject");
}
