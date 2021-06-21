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

#ifndef GB_BEHAVIOR_TREE__BEHAVIOR_TREE_NODES__GETSEARCHPOSE_HPP_
#define GB_BEHAVIOR_TREE__BEHAVIOR_TREE_NODES__GETSEARCHPOSE_HPP_

#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "ros2_knowledge_graph/GraphNode.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

namespace gb_world_model
{

class GetSearchPose : public BT::ActionNodeBase
{
public:
  explicit GetSearchPose(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  void halt();
  BT::NodeStatus tick();

  static BT::PortsList providedPorts()
  {
    return BT::PortsList(
      {
        BT::OutputPort<geometry_msgs::msg::PoseStamped>("wp_pose")
      });
  }

private:
  ros2_knowledge_graph::GraphNode * graph_;
  rclcpp::Node::SharedPtr node_;
};

}  // namespace namespace gb_world_model

#endif  // GB_BEHAVIOR_TREE__BEHAVIOR_TREE_NODES__GETSEARCHPOSE_HPP_
