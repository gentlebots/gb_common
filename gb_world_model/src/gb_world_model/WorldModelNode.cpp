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

using std::placeholders::_1;

WorldModelNode::WorldModelNode()
: Node("world_model")
{
  declare_parameter("world_root");
  declare_parameter("object_classes");
  robot_ = "jarvis";
}

void
WorldModelNode::start()
{
  graph_ =std::make_shared<ros2_knowledge_graph::GraphNode>(
    shared_from_this());

  init_graph_node(get_parameter("world_root").as_string());
  start_object_classes();

  auto node_robot = ros2_knowledge_graph::new_node(robot_, "robot");
  graph_->update_node(node_robot);

  auto edge_roboot_is = ros2_knowledge_graph::new_edge<std::string>(
    get_parameter("world_root").as_string(),
    robot_, "is");
  graph_->update_edge(edge_roboot_is);

  dope_sub_ = create_subscription<vision_msgs::msg::Detection3DArray>(
    "/dope/detected_objects", 100, std::bind(&WorldModelNode::dope_callback, this, _1));

  tfBuffer_ = std::make_shared<tf2::BufferCore>();
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_, shared_from_this(), false);
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
  declare_parameter(node_name + ".stores");
  
  std::vector<std::string> stored_classes = {""};
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
  ros2_knowledge_graph::add_property(node, "dimensions_y", dimensions_y);
  ros2_knowledge_graph::add_property(node, "dimensions_z", dimensions_z);
  ros2_knowledge_graph::add_property(node, "is_container", is_container);
  ros2_knowledge_graph::add_property(node, "is_navegable", is_navegable);
  if (get_parameter(node_name + ".stores", stored_classes)) {
    ros2_knowledge_graph::add_property(node, "stores", stored_classes);
  }

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

  ros2_knowledge_graph::add_property(node, "position", pose);

  graph_->update_node(node);

  if (parent != "") {
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
      std::vector<double> coords = {0.0, 0.0, 0.0};

      get_parameter_or<std::vector<double>>(node_name + "." + wp, coords, coords);
      get_parameter_or<std::string>(node_name + ".reference_frame", reference_frame, "map");


      geometry_msgs::msg::PoseStamped pose_wp;
      pose_wp.header.stamp = now();
      pose_wp.header.frame_id = reference_frame;
      pose_wp.pose.position.x = coords[0];
      pose_wp.pose.position.y = coords[1];
      pose_wp.pose.position.z = 0.0;

      tf2::Quaternion q_wp;
      q_wp.setRPY(0.0, 0.0, coords[2]);

      pose_wp.pose.orientation.x = q_wp.x();
      pose_wp.pose.orientation.y = q_wp.y();
      pose_wp.pose.orientation.z = q_wp.z();
      pose_wp.pose.orientation.w = q_wp.w();

      ros2_knowledge_graph::add_property(node_wp, "position", pose_wp);
      graph_->update_node(node_wp);

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

void
WorldModelNode::start_object_classes()
{
  auto object_classes = get_parameter("object_classes").as_string_array();

  for (const auto & object_class : object_classes) {
    declare_parameter(object_class);
    auto objects = get_parameter(object_class).as_string_array();

    object_classes_[object_class] = objects;
  }
}

void
WorldModelNode::dope_callback(vision_msgs::msg::Detection3DArray::UniquePtr msg)
{
  for (const auto & detection : msg->detections) {
    const std::string & id = detection.results[0].id;

    ros2_knowledge_graph_msgs::msg::Node object_node;
    if (graph_->exist_node(id)) {
      object_node = graph_->get_node(id).value();
    } else {
      std::string object_class = "unknown_objects";
      for (const auto & class_it : object_classes_) {
        if (std::find(class_it.second.begin(), class_it.second.end(), id) !=
          class_it.second.end())
        {
          object_class = class_it.first;
          break;
        }
      }
      object_node = ros2_knowledge_graph::new_node(id, object_class);
      ros2_knowledge_graph::add_property(object_node, "class", object_class);
    }

    geometry_msgs::msg::TransformStamped map2frame_msg;
    tf2::TimePoint perception_ts = tf2::timeFromSec(rclcpp::Time(msg->header.stamp).seconds());

  	std::string error;
		if (tfBuffer_->canTransform("map", msg->header.frame_id, perception_ts, &error)) {
			map2frame_msg = tfBuffer_->lookupTransform("map", msg->header.frame_id, perception_ts);
    } else {
			RCLCPP_ERROR(get_logger(), "Can't transform %s", error.c_str());
			continue;
		}
  
    tf2::Stamped<tf2::Transform> map2frame;
    tf2::Transform map2frame_b;
		tf2::convert(map2frame_msg, map2frame);
    map2frame_b = map2frame;
  
    tf2::Transform frame2perception;
    const auto & position = detection.results[0].pose.pose.position;
    const auto & orientation = detection.results[0].pose.pose.orientation;

    frame2perception.setOrigin(tf2::Vector3(position.x, position.y, position.z));
    frame2perception.setRotation(tf2::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w));

    tf2::Transform map2perception = map2frame_b * frame2perception;

    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.header.stamp = msg->header.stamp;
    pose.pose.position.x = map2perception.getOrigin().x();
    pose.pose.position.y = map2perception.getOrigin().y();
    pose.pose.position.z = map2perception.getOrigin().z();
    pose.pose.orientation.x = map2perception.getRotation().x();
    pose.pose.orientation.y = map2perception.getRotation().y();
    pose.pose.orientation.z = map2perception.getRotation().z();
    pose.pose.orientation.w = map2perception.getRotation().w();

    ros2_knowledge_graph::add_property(object_node, "position", pose);
    graph_->update_node(object_node);
    auto edge_perceived_obj = ros2_knowledge_graph::new_edge<std::string>(
        robot_, id, "perceived");
    graph_->update_edge(edge_perceived_obj);
  }
}


};  // namespace gb_world_model
