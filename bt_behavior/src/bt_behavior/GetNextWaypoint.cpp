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

#include <string>
#include <iostream>

#include "bt_behavior/Patrol.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"

#include "geometry_msgs/msg/twist.hpp"

#include "rclcpp/rclcpp.hpp"

namespace bt_behavior
{

using namespace std::chrono_literals;

GetNextWaypoint::GetNextWaypoint(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{
  config().blackboard->get("node", node_);
}

void
GetNextWaypoint::halt()
{
  std::cout << "GNW halt" << std::endl;
}

BT::NodeStatus
GetNextWaypoint::tick()
{
  int ind;
  config().blackboard->get("index", *ind);
  rclcpp::Parameter wp_array = this->get_parameter("waypoint");
  std::vector<std::string> wp_vector = wp_array.as_string_array();
  rclcpp::Parameter wp_param = this->get_parameter(wp_vector[ind]);
  std::vector<double> wp = wp_param.as_double_array(); 
  //aqui hay que leer del yaml pero no tengo ni idea de como se hace
  //hay que leer y quedarte con el wp[ind] para establecerlo como output
  //y lo que hay escrito es que pasamos publicamos en la blackboard el indx+1 para en la siguiente ir al siguiente punto
  ind++;
  config().blackboard->set("index", ind);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace bt_behavior

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<bt_behavior::GetNextWaypoint>("GetNextWaypoint");
}
