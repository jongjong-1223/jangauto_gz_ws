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

#ifndef NAV2_STRAIGHTLINE_PLANNER__STRAIGHT_LINE_PLANNER_HPP_
#define NAV2_STRAIGHTLINE_PLANNER__STRAIGHT_LINE_PLANNER_HPP_

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "nav2_core/global_planner.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

namespace nav2_straightline_planner
{

// nav2_core::GlobalPlanner pluginlib 플러그인 구현체.
//
// ## 역할
// - Nav2 planner_server가 런타임에 pluginlib으로 로드하는 전역 경로 플래너.
//   등록 이름은 `global_planner_plugin.xml`에 정의된
//   "nav2_straightline_planner/StraightLine"(jangauto_navigation2의
//   nav2_params.yaml `planner_server.GridBased.plugin`에서 참조).
// - 코스트맵을 탐색(Dijkstra/A* 등)하지 않고 시작점→목표점을 그대로
//   직선으로 잇는 가장 단순한 플래너다(NavfnPlanner/SmacPlanner의 대체 경량판).
//   이 프로젝트에서 쓰는 이유: 이동 구간이 이미 알려진 웨이포인트 직선
//   구간이라 코스트맵 탐색 자체가 불필요하기 때문(nav2_params.yaml 주석 참고).
// - Nav2 lifecycle 노드 규약에 맞춰 configure/activate/deactivate/cleanup
//   4단계로 관리되지만, 이 플래너는 상태가 없어 실제로는 로그만 남긴다.
class StraightLine : public nav2_core::GlobalPlanner
{
public:
  StraightLine() = default;
  ~StraightLine() = default;

  // lifecycle: configure 단계 — 코스트맵/프레임/파라미터(interpolation_resolution)를
  // 멤버 변수에 캐싱한다. 다른 세 메서드보다 먼저, 딱 한 번 호출된다.
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  // lifecycle: cleanup 단계 — 이 플래너는 해제할 리소스가 없어 로그만 남긴다.
  void cleanup() override;

  // lifecycle: activate 단계 — 마찬가지로 로그만 남긴다(상태 없음).
  void activate() override;

  // lifecycle: deactivate 단계 — 마찬가지로 로그만 남긴다(상태 없음).
  void deactivate() override;

  // 시작/목표 pose 사이를 직선으로 보간한 Path를 만들어 리턴한다.
  // bt_navigator가 상태 진입마다(또는 재계획 주기마다) 호출하는 실제 진입점.
  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    std::function<bool()> cancel_checker) override;

private:
  // TF buffer — 현재 구현(createPlan)에서는 실제로 쓰이지 않고 보관만 함
  // (좌표 변환이 필요해지면 쓸 수 있도록 인터페이스 계약상 받아둔 것).
  std::shared_ptr<tf2_ros::Buffer> tf_;

  // 이 플래너를 소유한 lifecycle 노드(costmap/파라미터/로거 접근용).
  nav2_util::LifecycleNode::SharedPtr node_;

  // 전역 코스트맵 — 현재 구현에서는 좌표 유효성 검사에 쓰이지 않고
  // configure() 시점에 캐싱만 해둔다(탐색 없이 직선 보간만 하기 때문).
  nav2_costmap_2d::Costmap2D * costmap_;

  // global_frame_: 코스트맵의 전역 프레임(예: "map") — start/goal이 이
  // 프레임이 아니면 createPlan()이 빈 경로를 리턴한다.
  // name_: pluginlib이 이 플래너 인스턴스에 부여한 이름(파라미터 네임스페이스,
  // 로그 태그로 쓰임).
  std::string global_frame_, name_;

  // 보간 간격(미터). 두 점 사이를 이 간격으로 나눈 만큼 중간 pose를 생성한다.
  // 파라미터 이름: "<name_>.interpolation_resolution", 기본값 0.1.
  double interpolation_resolution_;
};

}  // namespace nav2_straightline_planner

#endif  // NAV2_STRAIGHTLINE_PLANNER__STRAIGHT_LINE_PLANNER_HPP_
