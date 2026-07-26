/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020 Shivang Patel
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Shivang Patel
 *
 * Reference tutorial:
 * https://navigation.ros.org/tutorials/docs/writing_new_nav2planner_plugin.html
 *********************************************************************/

#include <cmath>
#include <string>
#include <memory>
#include "nav2_util/node_utils.hpp"

#include "nav2_straightline_planner/straight_line_planner.hpp"

namespace nav2_straightline_planner
{

// planner_server가 이 플러그인을 로드한 직후 딱 한 번 호출.
// 코스트맵/전역 프레임을 캐싱하고, "<name_>.interpolation_resolution"
// 파라미터(기본 0.1m)를 읽어 보간 간격을 확정한다.
void StraightLine::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent.lock();
  name_ = name;
  tf_ = tf;
  costmap_ = costmap_ros->getCostmap();
  global_frame_ = costmap_ros->getGlobalFrameID();

  // Parameter initialization
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".interpolation_resolution", rclcpp::ParameterValue(
      0.1));
  node_->get_parameter(name_ + ".interpolation_resolution", interpolation_resolution_);
}

// 참고: 로그 문구의 "NavfnPlanner"는 이 플래너 이름(StraightLine)이 아니라
// 원본 nav2 튜토리얼에서 그대로 복사돼 남은 문구다(업스트림 원본 그대로 —
// 이 프로젝트에서 새로 만든 오류 아님). cleanup/activate/deactivate 셋 다
// 관리할 리소스가 없어 로그만 남기고 실제로 하는 일은 없다.
void StraightLine::cleanup()
{
  RCLCPP_INFO(
    node_->get_logger(), "CleaningUp plugin %s of type NavfnPlanner",
    name_.c_str());
}

void StraightLine::activate()
{
  RCLCPP_INFO(
    node_->get_logger(), "Activating plugin %s of type NavfnPlanner",
    name_.c_str());
}

void StraightLine::deactivate()
{
  RCLCPP_INFO(
    node_->get_logger(), "Deactivating plugin %s of type NavfnPlanner",
    name_.c_str());
}

// bt_navigator가 경로 계산이 필요할 때마다(최초 진입 + 재계획 주기) 호출하는
// 이 플러그인의 핵심 진입점. 코스트맵 탐색 없이 start->goal을
// interpolation_resolution_ 간격으로 선형 보간만 해서 Path를 만든다.
// cancel_checker는 이 구현에서 쓰지 않음(보간은 즉시 끝나 취소할 여지가 없음).
nav_msgs::msg::Path StraightLine::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  std::function<bool()> /*cancel_checker*/)
{
  nav_msgs::msg::Path global_path;

  // Checking if the goal and start state is in the global frame
  // start/goal이 코스트맵의 전역 프레임(global_frame_)이 아니면 좌표 비교가
  // 무의미하므로 빈 경로를 리턴해 실패로 처리한다(별도 좌표 변환은 안 함).
  if (start.header.frame_id != global_frame_) {
    RCLCPP_ERROR(
      node_->get_logger(), "Planner will only except start position from %s frame",
      global_frame_.c_str());
    return global_path;
  }

  if (goal.header.frame_id != global_frame_) {
    RCLCPP_INFO(
      node_->get_logger(), "Planner will only except goal position from %s frame",
      global_frame_.c_str());
    return global_path;
  }

  global_path.poses.clear();
  global_path.header.stamp = node_->now();
  global_path.header.frame_id = global_frame_;
  // calculating the number of loops for current value of interpolation_resolution_
  // start-goal 직선 거리(hypot)를 interpolation_resolution_로 나눠 스텝 수를
  // 정한다 — 즉 간격이 작을수록(정밀할수록) 포인트 수가 늘어난다.
  int total_number_of_loop = std::hypot(
    goal.pose.position.x - start.pose.position.x,
    goal.pose.position.y - start.pose.position.y) /
    interpolation_resolution_;
  // 스텝 1개당 x/y 증분 — 아래 for문에서 i번째 포인트 = start + 증분*i.
  double x_increment = (goal.pose.position.x - start.pose.position.x) / total_number_of_loop;
  double y_increment = (goal.pose.position.y - start.pose.position.y) / total_number_of_loop;

  // start부터 goal 직전까지 등간격 중간 포인트를 생성. 코스트맵 장애물 검사가
  // 전혀 없다는 점이 이 플래너의 핵심 특징(그래서 "웨이포인트 구간이 이미
  // 안전하다고 알려진 경우"에만 쓰기로 한 것).
  for (int i = 0; i < total_number_of_loop; ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = start.pose.position.x + x_increment * i;
    pose.pose.position.y = start.pose.position.y + y_increment * i;
    pose.pose.position.z = 0.0;
    // 방향(orientation)은 전부 항등 quaternion 고정 — 이 경로의 각 포인트가
    // 실제 주행 헤딩을 담고 있지 않다는 뜻(헤딩은 컨트롤러/별도 로직 몫).
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;
    pose.header.stamp = node_->now();
    pose.header.frame_id = global_frame_;
    global_path.poses.push_back(pose);
  }

  // 보간 루프는 goal 직전까지만 채우므로, 실제 목표 pose(원래 orientation
  // 포함)를 마지막에 그대로 추가해 경로를 마무리한다.
  geometry_msgs::msg::PoseStamped goal_pose = goal;
  goal_pose.header.stamp = node_->now();
  goal_pose.header.frame_id = global_frame_;
  global_path.poses.push_back(goal_pose);

  return global_path;
}

}  // namespace nav2_straightline_planner

// pluginlib이 이 클래스를 nav2_core::GlobalPlanner 플러그인으로 찾을 수 있게
// 등록하는 매크로. 실제 조회 이름("nav2_straightline_planner/StraightLine")은
// 이 매크로가 아니라 같은 패키지의 global_planner_plugin.xml에 정의돼 있다.
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_straightline_planner::StraightLine, nav2_core::GlobalPlanner)
