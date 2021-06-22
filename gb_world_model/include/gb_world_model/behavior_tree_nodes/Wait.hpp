// Copyright (c) 2021 Intelligent Robotics Lab
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

#ifndef GB_BEHAVIOR_TREE__BEHAVIOR_TREE_NODES__WAIT_HPP_
#define GB_BEHAVIOR_TREE__BEHAVIOR_TREE_NODES__WAIT_HPP_

#include <string>
#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

namespace gb_world_model
{

class Wait : public BT::ActionNodeBase
{
public:
  Wait(
    const std::string & name,
    const BT::NodeConfiguration & conf);
    
  void halt();
  BT::NodeStatus tick();

  static BT::PortsList providedPorts()
  {
    return BT::PortsList(
    {
      BT::InputPort<std::string>("seconds")
    });
  }

private:
  rclcpp::Node::SharedPtr node_;
  bool initialized_;
  rclcpp::Time first_tick_;
};

}  // namespace gb_world_model

#endif  // GB_BEHAVIOR_TREE__BEHAVIOR_TREE_NODES__WAIT_HPP_
