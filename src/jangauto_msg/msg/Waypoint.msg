# ㄹ자(보우스트로피던) 커버리지 경로의 한 지점.
# coverage_path_action_server.py가 생성해 CoveragePath.waypoints로 담아 보낸다.

float64 x                # map 프레임 x좌표 (m)
float64 y                # map 프레임 y좌표 (m)
float64 yaw              # 다음 지점으로 향하는 진행 방향 (rad). 마지막 지점은 직전 값 유지
float64 turn_angle       # 이 지점에서 다음 구간으로 출발하기 전 필요한 제자리회전량 (rad, -pi~pi)
float64 dist_to_next     # 다음 지점까지 거리 (m). 마지막 지점은 0
string kind              # start/work_start/work_end/turn_out/turn_in/end
uint8 row_index          # 몇 번째 두둑(작업 줄)인지 (0부터)
