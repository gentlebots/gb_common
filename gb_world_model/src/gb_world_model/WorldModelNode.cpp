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

#include "ros2_knowledge_graph/graph_utils.hpp"
#include "ros2_knowledge_graph/GraphNode.hpp"

#include "rclcpp/rclcpp.hpp"

namespace gb_world_model
{

WorldModelNode::WorldModelNode()
: Node("world_model")
{
  declare_parameter("world_root");
}

void
WorldModelNode::start()
{
  graph_ =std::make_shared<ros2_knowledge_graph::GraphNode>(
    shared_from_this());

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
  declare_parameter(node_name + ".reference_frame");
  declare_parameter(node_name + ".dimensions.x");
  declare_parameter(node_name + ".dimensions.y");
  declare_parameter(node_name + ".dimensions.z");
  declare_parameter(node_name + ".is_container");
  declare_parameter(node_name + ".is_navegable");
  declare_parameter(node_name + ".contains");
  declare_parameter(node_name + ".waypoints");

  std::string class_id = "";
  std::vector<double> position = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  std::string reference_frame = parent;
  std::vector<double> dimensions_x = {0.0, 0.0};
  std::vector<double> dimensions_y = {0.0, 0.0};
  std::vector<double> dimensions_z = {0.0, 0.0};
  bool is_container = false;
  bool is_navegable = false;

  get_parameter_or<std::string>(node_name + ".class", class_id, class_id);
  get_parameter_or<std::vector<double>>(node_name + ".position", position, position);
  get_parameter_or<std::string>(node_name + ".reference_frame", reference_frame, reference_frame);
  get_parameter_or<std::vector<double>>(node_name + ".dimensions.x", dimensions_x, dimensions_x);
  get_parameter_or<std::vector<double>>(node_name + ".dimensions.y", dimensions_y, dimensions_y);
  get_parameter_or<std::vector<double>>(node_name + ".dimensions.z", dimensions_z, dimensions_z);
  get_parameter_or<bool>(node_name + ".is_container", is_container, is_container);
  get_parameter_or<bool>(node_name + ".is_navegable", is_navegable, is_navegable);


  auto node = ros2_knowledge_graph::new_node(node_name, class_id);
  ros2_knowledge_graph::add_property(node, "reference_frame", reference_frame);
  ros2_knowledge_graph::add_property(node, "dimensions_x", dimensions_x);
  ros2_knowledge_graph::add_property(node, "dimensions_x", dimensions_y);
  ros2_knowledge_graph::add_property(node, "dimensions_z", dimensions_z);
  ros2_knowledge_graph::add_property(node, "is_container", is_container);
  ros2_knowledge_graph::add_property(node, "is_navegable", is_navegable);

  graph_->update_node(node);

  if (parent != "") {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = now();
    pose.header.frame_id = reference_frame;
    pose.pose.position.x = position[0];
    pose.pose.position.y = position[1];
    pose.pose.position.z = position[2];

    tf2::Quaternion q;
    q.setRPY(position[3], position[4], position[5]);

    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    pose.pose.orientation.w = q.w();

    auto edge = ros2_knowledge_graph::new_edge(parent, node_name, pose);
    graph_->update_edge(edge);
    auto edge_contains = ros2_knowledge_graph::new_edge<std::string>(
      parent, node_name, "contains");
    graph_->update_edge(edge_contains);
  }

  if (is_navegable) {
    std::vector<std::string> waypoints = {};
    get_parameter<std::vector<std::string>>(node_name + ".waypoints", waypoints);

    for (const auto & wp : waypoints) {
      declare_parameter(node_name + "." + wp);

      auto node_wp = ros2_knowledge_graph::new_node(wp, "waypoint");
      graph_->update_node(node_wp);
      std::vector<double> coords = {0.0, 0.0, 0.0};

      get_parameter_or<std::vector<double>>(node_name + "." + wp, coords, coords);
      get_parameter_or<std::string>(node_name + ".reference_frame", reference_frame, node_name);

      geometry_msgs::msg::TransformStamped tf_wp;
      tf_wp.header.stamp = now();
      tf_wp.header.frame_id = reference_frame;
      tf_wp.child_frame_id = wp;
      tf_wp.transform.translation.x = coords[0];
      tf_wp.transform.translation.y = coords[1];
      tf_wp.transform.translation.z = 0.0;

      tf2::Quaternion q_wp;
      q_wp.setRPY(0.0, 0.0, coords[2]);

      tf_wp.transform.rotation.x = q_wp.x();
      tf_wp.transform.rotation.y = q_wp.y();
      tf_wp.transform.rotation.z = q_wp.z();
      tf_wp.transform.rotation.w = q_wp.w();

      auto edge_wp = ros2_knowledge_graph::new_edge(node_name, wp, tf_wp, true);
      graph_->update_edge(edge_wp);
      auto edge_has_wp = ros2_knowledge_graph::new_edge<std::string>(
        node_name, wp, "has_wp");
      graph_->update_edge(edge_has_wp);
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
