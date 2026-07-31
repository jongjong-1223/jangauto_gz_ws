#!/usr/bin/env python3
"""ㄹ자(보우스트로피던) 커버리지 경로 생성 순수 함수 모듈 (ROS 비의존).

## 역할
- 온실 다각형 + 변별 안전거리 + 로봇 반경으로부터 ㄹ자 경로를 계산하는
  알고리즘 본체. `coverage_path_action_server.py`가 이 모듈을 호출해 ROS
  액션으로 감싸 노출한다 — 이 파일 자체는 numpy/matplotlib만 쓰고 rclpy에
  의존하지 않아, 액션 서버 없이도 단독으로 테스트/디버깅 가능하다.

## 파이프라인 (`run_pipeline`)
1. 다각형 + 변별 안전거리 + 로봇 외접원 반지름 -> 로봇 진입 한계선(유효 맵)
2. 유효 맵을 yaw 방향으로 회전 -> 축 정렬 좌표계 변환
3. 축 정렬 좌표계에서 격자화(rasterize) 후 최대 내접 직사각형 탐색
   (histogram + stack 방식), 다각형 무게중심에 가장 가까운 후보 선택
4. 사각형을 원래 좌표계로 회전 복원
5. 사각형 크기에서 헤드랜드 길이를 빼 실제 작업 길이 산출, 두둑 간격으로
   두둑 개수 산출
6. 두둑 라인 + ㄹ자 경로(좌측/우측 시작 두 버전) 생성, 원래 좌표계로 복원

## 주의
다각형 오프셋은 변을 따라 평행 이동한 뒤 인접 변끼리 교차시키는 방식이라
볼록/완만한 오목 다각형에서는 잘 동작하지만, 반사각이 크고 안전거리가 큰
경우 자기교차가 생길 수 있다(Clipper 등 검증된 라이브러리 대체 여지 있음).
"""

import numpy as np
from matplotlib.path import Path


# ---------------------------------------------------------------------------
# 다각형 오프셋
# ---------------------------------------------------------------------------

def signed_area(vertices):
    """양수면 CCW, 음수면 CW."""
    x = np.array([p[0] for p in vertices])
    y = np.array([p[1] for p in vertices])
    x2 = np.roll(x, -1)
    y2 = np.roll(y, -1)
    return 0.5 * np.sum(x * y2 - x2 * y)


def offset_polygon(vertices, edge_offsets):
    """각 변(edge_offsets[i] = vertices[i]->vertices[i+1] 변)을 안쪽으로
    edge_offsets[i]만큼 이동한 새 변들의 교차점으로 새 다각형을 만든다."""
    verts = list(vertices)
    offs = list(edge_offsets)

    # 항상 CCW로 통일(CCW 기준일 때 내부가 진행방향의 왼쪽이 됨).
    if signed_area(verts) < 0:
        verts = verts[::-1]
        offs = offs[::-1]
        # 방향이 뒤집히면 각 변의 인덱스도 한 칸 밀어줘야 원래 변-거리 매칭 유지됨.
        offs = [offs[(i - 1) % len(offs)] for i in range(len(offs))]

    n = len(verts)
    shifted_lines = []
    for i in range(n):
        p1 = np.array(verts[i], dtype=float)
        p2 = np.array(verts[(i + 1) % n], dtype=float)
        edge_dir = p2 - p1
        edge_dir = edge_dir / np.linalg.norm(edge_dir)
        # CCW 다각형에서 내부로 향하는 법선 = 진행방향을 +90도 회전.
        inward_normal = np.array([-edge_dir[1], edge_dir[0]])
        d = offs[i]
        shifted_lines.append((p1 + inward_normal * d, p2 + inward_normal * d))

    def line_intersect(a1, a2, b1, b2):
        """직선 a1-a2와 b1-b2의 교점(무한 직선 기준)."""
        d1 = a2 - a1
        d2 = b2 - b1
        denom = d1[0] * d2[1] - d1[1] * d2[0]
        if abs(denom) < 1e-9:
            return (a1 + b1) / 2  # 거의 평행하면 두 점 평균으로 대체(예외 처리)
        t = ((b1[0] - a1[0]) * d2[1] - (b1[1] - a1[1]) * d2[0]) / denom
        return a1 + d1 * t

    new_verts = []
    for i in range(n):
        prev_line = shifted_lines[(i - 1) % n]
        curr_line = shifted_lines[i]
        pt = line_intersect(prev_line[0], prev_line[1], curr_line[0], curr_line[1])
        new_verts.append(tuple(pt))

    return new_verts


def polygon_centroid(vertices):
    """면적 기준 무게중심."""
    x = np.array([p[0] for p in vertices])
    y = np.array([p[1] for p in vertices])
    x2 = np.roll(x, -1)
    y2 = np.roll(y, -1)
    cross = x * y2 - x2 * y
    a = 0.5 * np.sum(cross)
    cx = np.sum((x + x2) * cross) / (6 * a)
    cy = np.sum((y + y2) * cross) / (6 * a)
    return np.array([cx, cy])


def rotate_points(points, angle_rad):
    c, s = np.cos(angle_rad), np.sin(angle_rad)
    r = np.array([[c, -s], [s, c]])
    pts = np.array(points)
    return pts @ r.T


# ---------------------------------------------------------------------------
# 격자화 + 최대 내접 직사각형 (histogram + stack)
# ---------------------------------------------------------------------------

def rasterize_polygon(vertices, cell_size):
    xs = [p[0] for p in vertices]
    ys = [p[1] for p in vertices]
    minx, maxx = min(xs), max(xs)
    miny, maxy = min(ys), max(ys)

    nx = max(2, int(np.ceil((maxx - minx) / cell_size)))
    ny = max(2, int(np.ceil((maxy - miny) / cell_size)))

    xs_grid = minx + (np.arange(nx) + 0.5) * cell_size
    ys_grid = miny + (np.arange(ny) + 0.5) * cell_size
    xx, yy = np.meshgrid(xs_grid, ys_grid)
    points = np.column_stack([xx.ravel(), yy.ravel()])

    path = Path(vertices)
    inside = path.contains_points(points)
    grid = inside.reshape(ny, nx)

    return grid, minx, miny, cell_size


def all_maximal_rectangles(grid):
    """이진 행렬에서 나올 수 있는 모든(지역 최대) 직사각형을 수집한다.
    각 원소: (area, row_start, row_end, col_start, col_end) — 그리드 인덱스, inclusive."""
    rows, cols = grid.shape
    height = [0] * cols
    rects = []

    for r in range(rows):
        for c in range(cols):
            height[c] = height[c] + 1 if grid[r, c] else 0

        stack = []  # (start_col, height)
        for c in range(cols + 1):
            h = height[c] if c < cols else 0
            start = c
            while stack and stack[-1][1] > h:
                s_col, s_h = stack.pop()
                width = c - s_col
                area = s_h * width
                rects.append((area, r - s_h + 1, r, s_col, c - 1))
                start = s_col
            stack.append((start, h))

    return rects


def best_inscribed_rectangle(vertices, cell_size, tol=0.99):
    """격자 기반 최대 내접 직사각형 탐색. 최대 면적의 tol 비율 이상인 후보들
    중 다각형 무게중심에 가장 가까운 것 선택. 반환: (x_min, x_max, y_min, y_max)
    — 회전된 좌표계 기준."""
    grid, minx, miny, cs = rasterize_polygon(vertices, cell_size)
    rects = all_maximal_rectangles(grid)
    if not rects:
        raise RuntimeError("내접 사각형을 찾지 못함 (격자 해상도를 높여보기)")

    max_area = max(r[0] for r in rects)
    candidates = [r for r in rects if r[0] >= tol * max_area]

    centroid = polygon_centroid(vertices)

    def rect_world_bounds(r):
        _, r1, r2, c1, c2 = r
        return (minx + c1 * cs, minx + (c2 + 1) * cs, miny + r1 * cs, miny + (r2 + 1) * cs)

    best, best_dist = None, None
    for r in candidates:
        bounds = rect_world_bounds(r)
        center = np.array([(bounds[0] + bounds[1]) / 2, (bounds[2] + bounds[3]) / 2])
        dist = np.linalg.norm(center - centroid)
        if best is None or dist < best_dist:
            best, best_dist = bounds, dist

    return best


# ---------------------------------------------------------------------------
# ㄹ자 경로 생성
# ---------------------------------------------------------------------------

def compute_ridge_lines(x_min, x_max, y_min, y_max, headland_len, ridge_spacing):
    """두둑 개수와 각 두둑 라인의 y좌표, 작업구간 x범위 계산(좌우 대칭 여백)."""
    length = x_max - x_min
    width = y_max - y_min

    work_len = length - 2 * headland_len
    if work_len <= 0:
        raise ValueError("헤드랜드 길이가 너무 커서 작업 구간이 남지 않음")

    n_ridges = int(np.floor(width / ridge_spacing))
    if n_ridges <= 0:
        raise ValueError("두둑 간격이 너무 커서 두둑을 하나도 배치할 수 없음")

    margin = width - n_ridges * ridge_spacing
    ridge_ys = [y_min + margin / 2 + (i + 0.5) * ridge_spacing for i in range(n_ridges)]

    x_work_start = x_min + headland_len
    x_work_end = x_max - headland_len
    ridge_lines = [(y, x_work_start, x_work_end) for y in ridge_ys]

    return ridge_lines, work_len, n_ridges


def build_waypoints(x_min, x_max, y_min, y_max, headland_len, ridge_spacing, start_side="left"):
    """ㄹ자 경로를 라벨 붙은 웨이포인트 시퀀스로 생성(회전 좌표계, 헤드랜드
    중심 기준). start_side="left"면 왼쪽 헤드랜드에서 시작해 오른쪽으로 첫
    작업, "right"면 반대.

    각 웨이포인트 kind:
      - "start"      : 전체 경로의 시작점(첫 헤드랜드 중심)
      - "work_start" : 두둑 작업구간 진입점
      - "work_end"   : 두둑 작업구간 종료점
      - "turn_out"   : 작업 종료 후 도착한 헤드랜드 중심(여기서 진행방향->수직 회전)
      - "turn_in"    : 헤드랜드 내에서 다음 줄로 이동 후 도착(여기서 수직->진행방향 회전)
      - "end"        : 전체 경로의 마지막점(마지막 헤드랜드 중심)
    """
    ridge_lines, work_len, n_ridges = compute_ridge_lines(
        x_min, x_max, y_min, y_max, headland_len, ridge_spacing)
    ridge_ys = [rl[0] for rl in ridge_lines]
    x_work_start = x_min + headland_len
    x_work_end = x_max - headland_len
    left_c = x_min + headland_len / 2
    right_c = x_max - headland_len / 2

    waypoints = []
    going_right = (start_side == "left")

    for i, y in enumerate(ridge_ys):
        entry_c = left_c if going_right else right_c
        exit_c = right_c if going_right else left_c
        ws = x_work_start if going_right else x_work_end
        we = x_work_end if going_right else x_work_start

        kind_entry = "start" if i == 0 else "turn_in"
        waypoints.append({"x": entry_c, "y": y, "kind": kind_entry, "row_index": i})
        waypoints.append({"x": ws, "y": y, "kind": "work_start", "row_index": i})
        waypoints.append({"x": we, "y": y, "kind": "work_end", "row_index": i})

        if i < n_ridges - 1:
            waypoints.append({"x": exit_c, "y": y, "kind": "turn_out", "row_index": i})
        else:
            waypoints.append({"x": exit_c, "y": y, "kind": "end", "row_index": i})

        going_right = not going_right

    return waypoints, ridge_lines, work_len, n_ridges


def enrich_waypoints_world(waypoints_rot, yaw_rad):
    """회전 좌표계 웨이포인트를 원래 좌표계로 변환하고, 각 점에서 다음 점으로
    가기 위한 yaw, 그 점에서 필요한 회전량(turn_angle), 다음 점까지 거리,
    이동 종류(motion_type)를 계산한다."""
    pts_rot = np.array([[wp["x"], wp["y"]] for wp in waypoints_rot])
    pts_world = rotate_points(pts_rot, yaw_rad)

    n = len(waypoints_rot)
    out = []
    prev_yaw = None
    for i in range(n):
        x, y = pts_world[i]
        kind = waypoints_rot[i]["kind"]

        if i < n - 1:
            dx, dy = pts_world[i + 1] - pts_world[i]
            yaw_to_next = float(np.arctan2(dy, dx))
            dist_to_next = float(np.hypot(dx, dy))
        else:
            yaw_to_next = None
            dist_to_next = None

        if prev_yaw is None:
            turn_angle = 0.0
        else:
            diff = yaw_to_next - prev_yaw if yaw_to_next is not None else 0.0
            turn_angle = float((diff + np.pi) % (2 * np.pi) - np.pi)

        out.append({
            "idx": i,
            "x": float(x),
            "y": float(y),
            "kind": kind,
            "row_index": waypoints_rot[i]["row_index"],
            "yaw_to_next_rad": yaw_to_next,
            "turn_angle_rad": turn_angle,
            "dist_to_next": dist_to_next,
        })

        if yaw_to_next is not None:
            prev_yaw = yaw_to_next

    return out


# ---------------------------------------------------------------------------
# 전체 파이프라인
# ---------------------------------------------------------------------------

def run_pipeline(polygon, edge_safety_dist, robot_radius, yaw_deg,
                  ridge_spacing, headland_len, cell_size=0.2):
    """다각형+파라미터 -> 좌측/우측 시작 두 웨이포인트 시퀀스(월드 좌표) +
    사각형/작업길이/두둑개수 요약. `coverage_path_action_server.py`가 호출하는
    유일한 진입점."""
    yaw_rad = np.deg2rad(yaw_deg)

    # 안전거리 + 로봇 반지름을 합쳐서 로봇 중심 진입 한계선을 한 번에 계산.
    combined_offset = [d + robot_radius for d in edge_safety_dist]
    entry_limit = offset_polygon(polygon, combined_offset)

    # yaw 방향 정렬을 위해 -yaw만큼 회전 후, 격자 기반 최대 내접 사각형 탐색.
    rotated_entry = rotate_points(entry_limit, -yaw_rad)
    x_min, x_max, y_min, y_max = best_inscribed_rectangle(rotated_entry, cell_size)

    # 좌/우 두 시작 버전으로 웨이포인트 생성 후 원래 좌표계로 복원.
    waypoints_left_rot, _, work_len, n_ridges = build_waypoints(
        x_min, x_max, y_min, y_max, headland_len, ridge_spacing, start_side="left")
    waypoints_right_rot, _, _, _ = build_waypoints(
        x_min, x_max, y_min, y_max, headland_len, ridge_spacing, start_side="right")

    waypoints_left_world = enrich_waypoints_world(waypoints_left_rot, yaw_rad)
    waypoints_right_world = enrich_waypoints_world(waypoints_right_rot, yaw_rad)

    return {
        "waypoints_start_left": waypoints_left_world,
        "waypoints_start_right": waypoints_right_world,
        "rect_L": x_max - x_min,
        "rect_W": y_max - y_min,
        "work_len": work_len,
        "n_ridges": n_ridges,
    }
