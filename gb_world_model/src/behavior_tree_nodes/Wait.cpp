// Copyright (c) 2019 Intel Corporation
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
#include "gb_world_model/behavior_tree_nodes/Wait.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"


namespace gb_world_model
{

Wait::Wait(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(name, conf), initialized_(false)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  RCLCPP_INFO(node_->get_logger(), 
    "\"%s\" BtDecoratorNode initialized",
    name.c_str());
}

void
Wait::halt()
{
}

BT::NodeStatus 
Wait::tick()
{
  if (!initialized_) 
  {
    first_tick_ = node_->now();
    initialized_ = true;
  } else if ((node_->now() - first_tick_).seconds() > stoi(getInput<std::string>("seconds").value())) 
  {
    initialized_ = false;
    return BT::NodeStatus::SUCCESS;
  }
  RCLCPP_INFO(node_->get_logger(), "Waiting...");
  return BT::NodeStatus::RUNNING;
}

}  // namespace gb_world_model

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<gb_world_model::Wait>("Wait");
}
