# 온실 다각형 하나에 대한 ㄹ자 커버리지 경로 후보 하나(좌측 시작 또는 우측 시작).
# GenerateCoveragePath 액션의 result와 /jangauto_mission/selected_coverage_path
# (앱이 고른 경로, latched)에서 공용으로 쓴다.

Waypoint[] waypoints    # 전체 지점(진입 -> 작업 -> 헤드랜드 이동 -> ... -> 종료) 순서대로
float64 rect_length     # 최대 내접 사각형 길이 (yaw 방향)
float64 rect_width      # 최대 내접 사각형 폭
float64 work_len        # 헤드랜드 제외 실제 작업 길이
int32 n_ridges          # 두둑(작업 줄) 개수
string start_side       # "left" 또는 "right"
