# 온실 다각형 하나에 대한 ㄹ자 커버리지 경로 후보 하나(가까운 두둑부터 또는 먼 두둑부터).
# GenerateCoveragePath 액션의 result와 /jangauto_mission/selected_coverage_path
# (앱이 고른 경로, latched)에서 공용으로 쓴다.

Waypoint[] waypoints    # 전체 지점(진입 -> 작업 -> 헤드랜드 이동 -> ... -> 종료) 순서대로
float64 rect_length     # 최대 내접 사각형 길이 (yaw 방향)
float64 rect_width      # 최대 내접 사각형 폭
float64 work_len        # 헤드랜드 제외 실제 작업 길이
int32 n_ridges          # 두둑(작업 줄) 개수
string first_row_side   # "near"(y_min쪽 두둑부터) 또는 "far"(y_max쪽 두둑부터).
                         # 두 후보 모두 출발 헤드랜드(x_min쪽)에서 +yaw 방향으로 출발하며,
                         # 어느 두둑을 먼저 도는지만 다르다.
geometry_msgs/Point32[4] start_headland_corners  # 출발쪽 헤드랜드 사각형 꼭짓점(map 프레임, z=0)
geometry_msgs/Point32[4] far_headland_corners    # 반대쪽 헤드랜드 사각형 꼭짓점(map 프레임, z=0)
# 두 코너 배열은 near/far 후보 간 동일한 값(사각형 자체는 후보와 무관, 회전좌표계 기준 CCW 순서:
# [(x_min,y_min),(x_work,y_min),(x_work,y_max),(x_min,y_max)]를 yaw만큼 회전).
