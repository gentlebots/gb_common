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

#include <map>
#include <vector>
#include <string>


#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer_interface.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "ros2_knowledge_graph/GraphNode.hpp"

#include "vision_msgs/msg/detection3_d_array.hpp"
#include "vision_msgs/msg/object_hypothesis_with_pose.hpp"

#include "rclcpp/rclcpp.hpp"

namespace gb_world_model
{

class WorldModelNode : public rclcpp::Node
{
public:
	explicit WorldModelNode();
  void start();

protected:
  std::shared_ptr<ros2_knowledge_graph::GraphNode> graph_;

  rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr dope_sub_;
  void dope_callback(vision_msgs::msg::Detection3DArray::UniquePtr msg);

  void init_graph_node(
    const std::string & node_name,
    const std::string & parent = "");
  void start_object_classes();

  std::map<std::string, std::vector<std::string>> object_classes_;

	std::shared_ptr<tf2::BufferCore> tfBuffer_;
	std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::string robot_;
};

};  // namespace gb_world_model

#endif  // GB_WORLD_MODEL_WORLDMODELNODE_HPP
