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

#include "gb_world_model/WorldModelNode.hpp"

#include "rclcpp/rclcpp.hpp"

namespace gb_world_model
{

WorldModelNode::WorldModelNode()
: Node("world_model")
{
  graph_ = std::make_shared<ros2_knowledge_graph::GraphNode>(get_name());
  graph_->start();

  declare_parameter("world_root");
  init_graph_node(get_parameter("world_root").as_string());
}

void
WorldModelNode::init_graph_node(
  const std::string & node_name,
  const std::string & parent)
{
  declare_parameter(node_name);
  declare_parameter(node_name + ".class");
  declare_parameter(node_name + ".position");
  declare_parameter(node_name + ".dimensions.x");
  declare_parameter(node_name + ".dimensions.y");
  declare_parameter(node_name + ".dimensions.z");
  declare_parameter(node_name + ".is_container");
  declare_parameter(node_name + ".is_navegable");
  declare_parameter(node_name + ".contains");
  declare_parameter(node_name + ".waypoints");

  std::string class_id = "";
  std::vector<double> position = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  std::vector<double> dimensions_x = {0.0, 0.0};
  std::vector<double> dimensions_y = {0.0, 0.0};
  std::vector<double> dimensions_z = {0.0, 0.0};
  bool is_container = false;
  bool is_navegable = false;

  get_parameter_or<std::string>(node_name + ".class", class_id, class_id);
  get_parameter_or<std::vector<double>>(node_name + ".position", position, position);
  get_parameter_or<std::vector<double>>(node_name + ".dimensions.x", dimensions_x, dimensions_x);
  get_parameter_or<std::vector<double>>(node_name + ".dimensions.y", dimensions_y, dimensions_y);
  get_parameter_or<std::vector<double>>(node_name + ".dimensions.z", dimensions_z, dimensions_z);
  get_parameter_or<bool>(node_name + ".is_container", is_container, is_container);
  get_parameter_or<bool>(node_name + ".is_navegable", is_navegable, is_navegable);

  graph_->add_node(
    {node_name, class_id, {
      {"dimensions_x", ros2_knowledge_graph::to_property(dimensions_x)},
      {"dimensions_y", ros2_knowledge_graph::to_property(dimensions_y)},
      {"dimensions_z", ros2_knowledge_graph::to_property(dimensions_z)},
      {"is_container", ros2_knowledge_graph::to_property(is_container)},
      {"is_navegable", ros2_knowledge_graph::to_property(is_navegable)}
    }});

  if (parent != "") {
    std::string pos_as_string = 
      std::to_string(position[0]) + ":" +
      std::to_string(position[1]) + ":" +
      std::to_string(position[2]) + ":" +
      std::to_string(position[3]) + ":" +
      std::to_string(position[4]) + ":" +
      std::to_string(position[5]);
    graph_->add_edge(ros2_knowledge_graph::Edge{pos_as_string, "tf_static", parent, node_name});
  }

  if (is_navegable) {
    std::vector<std::string> waypoints = {};
    get_parameter<std::vector<std::string>>(node_name + ".waypoints", waypoints);

    for (const auto & wp : waypoints) {
      declare_parameter(node_name + "." + wp);
  
      graph_->add_node({wp, "waypoint"});
      std::vector<double> coords = {0.0, 0.0, 0.0};

      get_parameter_or<std::vector<double>>(node_name + "." + wp, coords, coords);

      std::string pos_as_string = 
        std::to_string(coords[0]) + ":" +
        std::to_string(coords[1]) + ":0.0:0.0:0.0:" +
        std::to_string(coords[2]);
      graph_->add_edge(ros2_knowledge_graph::Edge{pos_as_string, "tf_static", node_name, wp});
    }
  }

  if (is_container) {
    std::vector<std::string> contents = {};
    get_parameter_or<std::vector<std::string>>(node_name + ".contains", contents, contents);
    for (const auto content : contents) {
      init_graph_node(content, node_name);
    }
  }
}


};  // namespace gb_world_model
