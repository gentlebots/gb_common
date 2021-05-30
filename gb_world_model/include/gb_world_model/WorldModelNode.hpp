// Copyright 2021 Intelligent Robotics Lab
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

#ifndef GB_WORLD_MODEL_WORLDMODELNODE_HPP
#define GB_WORLD_MODEL_WORLDMODELNODE_HPP

#include "ros2_knowledge_graph/GraphNode.hpp"

#include "rclcpp/rclcpp.hpp"

namespace gb_world_model
{

class WorldModelNode : public rclcpp::Node
{
public:
	explicit WorldModelNode();

protected:
  std::shared_ptr<ros2_knowledge_graph::GraphNode> graph_;

  void init_graph_node(
    const std::string & node_name,
    const std::string & parent = "");
};

};  // namespace gb_world_model

#endif  // GB_WORLD_MODEL_WORLDMODELNODE_HPP