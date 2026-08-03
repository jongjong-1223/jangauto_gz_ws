// Copyright (c) 2020 Shrijit Singh
// Copyright (c) 2020 Samsung Research America
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

#ifndef NAV2_STANLEY_CONTROLLER__STANLEY_LAW_HPP_
#define NAV2_STANLEY_CONTROLLER__STANLEY_LAW_HPP_

#include <cmath>
#include <algorithm>
#include <limits>

// references/stanley_ref/stanley.c(Thrun et al., "Stanley: The robot that won
// the DARPA Grand Challenge", 2006)를 이 워크스페이스로 포팅한 순수 함수 모음.
// - normalizeAngle/crossTrackError/computeSteeringAngle: 자전거모델 전륜
//   조향각 δ = ψe + atan2(k·e, v+k_soft) 계산(레퍼런스의 stanley_compute
//   스텝2-5에 해당, 최근접점 탐색은 호출부인 stanley_controller.cpp에서
//   transformGlobalPlan()이 이미 만들어 둔 base_link 프레임 경로로 수행).
// - TrackCommand/stanleyToTrack: 위 δ를 궤도(트랙) 차량의 좌/우 속도로
//   매핑 — min_radius(피벗 한계) 클램프, max_track_speed 비율유지 포화까지
//   레퍼런스 stanley_to_track() 원본 그대로.
namespace nav2_stanley_controller
{

namespace stanley_law
{

/**
 * @brief 각도를 (-π, π] 범위로 정규화.
 */
inline double normalizeAngle(double angle)
{
  while (angle > M_PI) {angle -= 2.0 * M_PI;}
  while (angle < -M_PI) {angle += 2.0 * M_PI;}
  return angle;
}

/**
 * @brief 부호 있는 횡방향 오차(cross-track error) 계산.
 * 양수면 차량이 경로 왼쪽에 있음(경로가 차량 기준 오른쪽으로 보임).
 * @param path_x, path_y, path_yaw  최근접 경로점 위치/접선방향
 * @param veh_x, veh_y  차량(가상 전륜) 위치
 */
inline double crossTrackError(
  double path_x, double path_y, double path_yaw,
  double veh_x, double veh_y)
{
  const double dx = path_x - veh_x;
  const double dy = path_y - veh_y;
  const double nx = -std::sin(path_yaw);
  const double ny = std::cos(path_yaw);
  return dx * nx + dy * ny;
}

/**
 * @brief Stanley 조향각 δ = ψe + atan2(k·e, |v|+k_soft), ±max_steer_rad로 클램프.
 * @param psi_e  헤딩 오차(경로 접선 - 차량 헤딩, 정규화됨)
 * @param cross_track_error  crossTrackError() 결과
 * @param speed  차량 속도(부호 무관, 내부에서 절대값 사용)
 */
inline double computeSteeringAngle(
  double psi_e, double cross_track_error, double speed,
  double k, double k_soft, double max_steer_rad)
{
  const double cte_correction = std::atan2(k * cross_track_error, std::fabs(speed) + k_soft);
  double steer = psi_e + cte_correction;
  steer = std::clamp(steer, -max_steer_rad, max_steer_rad);
  return steer;
}

/** @brief stanleyToTrack()이 산출하는 좌/우 트랙 속도 및 진단 정보. */
struct TrackCommand
{
  double v_left = 0.0;
  double v_right = 0.0;
  double turning_radius = 0.0;
  bool radius_clamped = false;
  bool speed_scaled = false;
};

// tan(δ)가 발산하지 않을 만큼 δ가 충분히 작을 때 직선 주행으로 취급하는 임계값[rad].
constexpr double kStraightThreshold = 1e-4;

/**
 * @brief 자전거모델 조향각 δ와 전진속도 v_fwd를 궤도차량 좌/우 속도로 변환.
 *
 * 1. |δ|가 매우 작으면 직진(v_left=v_right=v_fwd)으로 처리.
 * 2. 자전거모델 회전반경 R_nom = wheelbase / tan(δ) 산출(부호=δ의 부호).
 * 3. |R|을 max(min_radius, track_width/2)로 하한 클램프(안쪽 트랙이 역회전할
 *    만큼 좁은 반경은 기본적으로 금지 — track_width/2가 물리적 최소치).
 * 4. 차동구동 기구학으로 외측/내측 트랙 속도 산출:
 *    v_outer = v_fwd·(R+B/2)/R, v_inner = v_fwd·(R-B/2)/R.
 * 5. 어느 한쪽이 max_track_speed를 넘으면 좌우 비율을 유지한 채 비례 축소.
 */
inline TrackCommand stanleyToTrack(
  double steer_rad, double v_fwd,
  double wheelbase, double track_width,
  double min_radius, double max_track_speed)
{
  TrackCommand cmd;

  const double half_b = track_width / 2.0;
  const double r_min = std::max(min_radius, half_b);

  if (std::fabs(steer_rad) < kStraightThreshold) {
    cmd.v_left = v_fwd;
    cmd.v_right = v_fwd;
    cmd.turning_radius = std::numeric_limits<double>::max();
    return cmd;
  }

  const double tan_delta = std::tan(steer_rad);
  const double r_nom = wheelbase / tan_delta;
  double r_abs = std::fabs(r_nom);
  const bool turn_left = r_nom > 0.0;

  if (r_abs < r_min) {
    r_abs = r_min;
    cmd.radius_clamped = true;
  }
  cmd.turning_radius = r_abs;

  const double ratio_outer = (r_abs + half_b) / r_abs;
  const double ratio_inner = (r_abs - half_b) / r_abs;

  const double v_outer = v_fwd * ratio_outer;
  const double v_inner = v_fwd * ratio_inner;

  if (turn_left) {
    cmd.v_right = v_outer;
    cmd.v_left = v_inner;
  } else {
    cmd.v_left = v_outer;
    cmd.v_right = v_inner;
  }

  const double max_abs = std::max(std::fabs(cmd.v_left), std::fabs(cmd.v_right));
  if (max_abs > max_track_speed && max_abs > 0.0) {
    const double scale = max_track_speed / max_abs;
    cmd.v_left *= scale;
    cmd.v_right *= scale;
    cmd.speed_scaled = true;
  }

  return cmd;
}

}  // namespace stanley_law

}  // namespace nav2_stanley_controller

#endif  // NAV2_STANLEY_CONTROLLER__STANLEY_LAW_HPP_
