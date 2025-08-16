import math, time
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from typing import Optional
from sklearn.cluster import DBSCAN
import os,logging, cv2

# =========================
# 유틸
# =========================
def _deg2rad(d: float) -> float:
    return d * math.pi / 180.0

def _rad2deg(d: float) -> float:
    return d / math.pi * 180.0

def pure_pursuit(alpha, wheel_base_cm=30, Ld_cm=30):
    alpha=_deg2rad(alpha)
    Ld_m=Ld_cm/100
    wheel_base_m=wheel_base_cm/100
    kappa = 2.0 * math.sin(alpha) / Ld_m
    delta = math.atan(wheel_base_m * kappa)
    return _rad2deg(delta)

def _build_candidate_angles_sorted(max_steer_deg: float, step_deg: float) -> np.ndarray:
    """
    -max..+max 단조 증가로 생성 (예: -35, -34, ..., +35)
    연속 자유 블록을 찾기 위해 정렬된 각도 배열이 필요.
    """
    n = int(round(max_steer_deg / step_deg))
    angles = np.arange(-n, n + 1, dtype=float) * step_deg
    angles = np.clip(angles, -max_steer_deg, +max_steer_deg)
    return angles

# =========================
# 레이-원 교차 (내부 단위 m)
# 좌표계: x 오른쪽+, y 위쪽+, 전방 +y
# 각도: 0°=+y, 오른쪽(시계) +
# =========================
def _first_hit_distance_for_ray_m(
    theta_deg: float,
    L_m: float,
    centers_xy_m: np.ndarray,  # (N,2) (x,y) in meters
    radius_m: float
) -> Optional[float]:
    """(0,0)에서 조향각 theta_deg로 길이 L_m 레이를 쏠 때 최초 교차거리(m). 없으면 None."""
    if centers_xy_m.size == 0:
        return None

    th = _deg2rad(theta_deg)
    vx = L_m * math.sin(th)   # 전방 +y 기준
    vy = L_m * math.cos(th)

    a = L_m * L_m
    cx = centers_xy_m[:, 0]
    cy = centers_xy_m[:, 1]
    # 정석: a t^2 + b t + c = 0,   b = -2 v·(C-P)
    b = -2.0 * (vx * cx + vy * cy)
    c = (cx * cx + cy * cy) - (radius_m * radius_m)

    disc = b * b - 4.0 * a * c
    valid = disc >= 0.0
    if not np.any(valid):
        return None

    sqrt_disc = np.zeros_like(disc)
    sqrt_disc[valid] = np.sqrt(disc[valid])

    denom = 2.0 * a
    t1 = np.full_like(disc, np.inf, dtype=float)
    t2 = np.full_like(disc, np.inf, dtype=float)
    t1[valid] = (-b[valid] - sqrt_disc[valid]) / denom
    t2[valid] = (-b[valid] + sqrt_disc[valid]) / denom

    cand = []
    m1 = (t1 >= 0.0) & (t1 <= 1.0)
    if np.any(m1): cand.append(t1[m1])
    m2 = (t2 >= 0.0) & (t2 <= 1.0)
    if np.any(m2): cand.append(t2[m2])
    if not cand: return None

    t_min = float(np.min(np.concatenate(cand)))
    return L_m * t_min  # m

# =========================
# 자유 블록의 중앙각 (block_center)
# =========================
def _pick_block_center_around_nearest_to_zero(free_mask: np.ndarray, candidates_deg: np.ndarray) -> Optional[float]:
    """
    자유각 중 |각도|가 최소인 각도를 포함하는 '연속 자유 블록'의 중앙각 반환.
    예: 자유가 [-30..-14]면 -22 반환.
    """
    free_idx = np.where(free_mask)[0]
    if free_idx.size == 0:
        return None
    i0 = free_idx[np.argmin(np.abs(candidates_deg[free_idx]))]  # |각도| 최소 자유각 인덱스
    l = i0
    while l - 1 >= 0 and free_mask[l - 1]:
        l -= 1
    r = i0
    while r + 1 < len(free_mask) and free_mask[r + 1]:
        r += 1
    return 0.5 * (candidates_deg[l] + candidates_deg[r])

# =========================
# 메인: 각도 선택 (입력은 cm)
# =========================
def pick_steering_angle(
    obstacles_xy_cm: np.ndarray,   # (N,2) — (x,y) cm
    obstacle_radius_cm: float = 25.0,  # 라바콘+여유 반경 (cm)
    max_steer_deg: float = 40.0,
    lookahead_cm: float = 300.0,       # 레이 길이 (cm)
    candidate_step_deg: float = 1.0,
    min_clear_cm: float = 0.0          # 이 거리 이상 비면 자유로 간주(0=완전 무충돌만)
) -> float:
    # cm -> m
    centers_m = obstacles_xy_cm / 100.0
    r_m = obstacle_radius_cm / 100.0
    L_m = lookahead_cm / 100.0
    clear_thresh_m = max(0.0, min_clear_cm / 100.0)

    # 정렬된 각도 축
    candidates_deg = _build_candidate_angles_sorted(max_steer_deg, candidate_step_deg)
    n = len(candidates_deg)

    free_mask = np.zeros(n, dtype=bool)
    free_angles = []
    best_clearance = (-1.0, 0.0)  # (최대 최초충돌거리[m], 각도)

    for i, ang in enumerate(candidates_deg):
        hit = _first_hit_distance_for_ray_m(ang, L_m, centers_m, r_m)
        if hit is None:
            free_mask[i] = True
            free_angles.append(ang)
        else:
            if clear_thresh_m > 0.0 and hit >= clear_thresh_m:
                free_mask[i] = True
                free_angles.append(ang)
            if hit > best_clearance[0]:
                best_clearance = (hit, ang)

    if np.any(free_mask):
        chosen = _pick_block_center_around_nearest_to_zero(free_mask, candidates_deg)
        if chosen is None:
            # 예외적으로 블록 중앙을 못 찾으면 0°에 가장 가까운 자유각
            chosen = float(min(free_angles, key=lambda a: abs(a)))
    else:
        # 자유각이 하나도 없으면 가장 멀리 비는 각도
        chosen = best_clearance[1]

    # 조향 한계
    chosen = float(max(-max_steer_deg, min(max_steer_deg, chosen)))
    return chosen

# =========================
# 라바콘 클러스터링 & 원 업데이트(중복 제거)
# =========================
def cluster_labacon(ranges_m, graph=None, ax=None, circle_radius_cm=25):
    """
    ranges_m: LiDAR range 배열 (m)
    graph: cluster center 점(Line2D) 핸들 (set_data로 갱신)
    ax: matplotlib axes
    return: labacon_list (N,2) in cm
    """
    ranges = np.array(ranges_m, dtype=float)
    # 0.2~1.5 m 범위만 사용 (나머지는 0 처리)
    valid = (~np.isnan(ranges)) & (~np.isinf(ranges)) & (ranges > 0.2) & (ranges < 1.5)
    ranges[~valid] = 0.0

    angles = np.linspace(0, 2*np.pi, len(ranges)) - np.pi/2
    x_cm = ranges * np.cos(angles) * 100.0
    y_cm = ranges * np.sin(angles) * 100.0
    pts = np.stack([x_cm, y_cm], axis=1)

    # DBSCAN으로 클러스터링
    dbscan = DBSCAN(eps=3, min_samples=3)  # eps=3cm, min_samples=3
    labels = dbscan.fit_predict(pts)

    # --- 이전에 그린 라바콘 원 제거 ---
    if ax:
        if hasattr(ax, "_cone_patches"):
            for p in ax._cone_patches:
                p.remove()
        ax._cone_patches = []

    labacon_list = np.empty((0, 2))
    cx_list, cy_list = [], []

    for k in set(labels):
        if k > 0:
            cx = x_cm[labels == k]
            cy = y_cm[labels == k]
            if cx.size:
                mx, my = float(np.mean(cx)), float(np.mean(cy))
                cx_list.append(mx); cy_list.append(my)
                labacon_list = np.append(labacon_list, [[mx, my]], axis=0)

                c = Circle((mx, my), circle_radius_cm, fill=False,
                           color='red', linestyle='--', linewidth=1.5)
                if ax:
                    ax.add_patch(c)
                    ax._cone_patches.append(c)

    # 중심점 산점도 갱신
    if graph: graph.set_data(cx_list, cy_list)
    return labacon_list

# =========================
# 조향 각도선: 한 번 만들고 set_data로 갱신
# =========================
def draw_angle_line(ax, angle_deg, length_cm=300, line=None, **style):
    """
    angle_deg: pick_steering_angle에서 나온 조향각(도 단위)
               0°=정면(+y), +는 오른쪽(시계), -는 왼쪽(반시계)
    length_cm: 선 길이 (cm)
    line: 기존 Line2D 핸들(없으면 새로 생성)
    style: matplotlib plot 스타일 인자(color, linewidth 등)
    return: Line2D 핸들
    """
    th = math.radians(angle_deg)
    end_x = length_cm * math.sin(th)
    end_y = length_cm * math.cos(th)
    xs, ys = [0.0, end_x], [0.0, end_y]

    if line is None:
        if "color" not in style: style["color"] = "orange"
        if "linewidth" not in style: style["linewidth"] = 2.5
        if "alpha" not in style: style["alpha"] = 0.9
        line, = ax.plot(xs, ys, **style)
    else:
        line.set_data(xs, ys)
    return line

def change2drive_angle(angle):
    '''
    angle 0~5 : 0도~5도
    angle 15~35  : 10도~30도
    값 1당 1도씩 변함
    angle 5~15 5도~10도
    값 1당 0.5도씩 변함
    '''
    if abs(angle)<=5: return angle
    elif abs(angle)<=10:
        if angle>0: return 2*angle-5
        else: return 2*angle+5
    else:
        if angle>0: return angle+5
        else: return angle-5

def compute_lookahead_cm(speed, s_min=4, s_max=10, Lmin_cm=25, Lmax_cm=70, gamma=1.0):
    u = (speed - s_min) / (s_max - s_min)
    u = u ** gamma
    return Lmin_cm + (Lmax_cm - Lmin_cm) * u
    

def create_save_folder(p_date="20250815"):
    try: base_dir = os.path.dirname(os.path.abspath(__file__))
    except NameError: base_dir = os.getcwd()

    # 현재 날짜 폴더명
    base_folder_name = p_date

    # 중복되지 않는 폴더명 찾기
    folder_name = base_folder_name
    counter = 0
    while os.path.exists(os.path.join(base_dir, folder_name)):
        counter += 1
        folder_name = f"{base_folder_name}({counter})"

    # 최종 경로
    folder_path = os.path.join(base_dir, folder_name)

    # 폴더 및 파일 생성
    os.makedirs(os.path.join(folder_path))
    return folder_path

# =========================
# 노드 클래스
# =========================
class Labacon_node:
    def __init__(self, debug_bool=False, save_bool=False):
        self.lookahead_length=70
        self.debug_bool=debug_bool
        self.save_bool=save_bool

        if debug_bool:
            plt.ion()
            self.fig, self.ax = plt.subplots(figsize=(8, 8))
            self.ax.set_xlim(-150, 150)
            self.ax.set_ylim(-150, 150)
            self.ax.grid(True, linestyle=':', linewidth=0.5)
            self.ax.set_xlabel("x (cm) → 오른쪽 +")
            self.ax.set_ylabel("y (cm) → 위쪽 +")
            self.cluster_points, = self.ax.plot([], [], 'bo', markersize=5, label='clusters')
            self.angle_line = None  # 각도선 핸들
            self.ax.legend(loc='upper right')
        
        if self.save_bool:
            self.img_path=create_save_folder()
            self.image_idx = 1

            angle_path = os.path.join(self.img_path, "angle.txt")
            self.angle_path = open(angle_path, "w")

    def labacon_main(self, obj):
        start=time.time()
        ranges = obj.ranges
        if self.debug_bool: labacon_list = cluster_labacon(ranges, self.cluster_points, self.ax)
        else: labacon_list = cluster_labacon(ranges)

        angle = pick_steering_angle(
            obstacles_xy_cm=labacon_list,
            lookahead_cm=self.lookahead_length,
        )

        # 각도선 업데이트(없으면 생성)
        if self.debug_bool:
            self.angle_line = draw_angle_line(self.ax, angle, length_cm=self.lookahead_length, line=self.angle_line)
            # 화면 갱신
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()
            
        lookahead_cm=compute_lookahead_cm(obj.speed)
        pp_angle=pure_pursuit(angle, wheel_base_cm=30, Ld_cm=lookahead_cm)

        if self.save_bool:
            '''
            canvas = self.fig.canvas   # 캔버스 연결
            canvas.draw()                # 렌더링
            buf = canvas.tostring_rgb()
            w, h = self.fig.canvas.get_width_height()
            img = np.frombuffer(buf, dtype=np.uint8).reshape(h, w, 3)

            img_bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            cv2.imwrite(f"{self.img_path}/{self.image_idx:03d}.png", img_bgr)
            self.image_idx+=1
            '''

            # self.fig.savefig(f"{self.img_path}/{self.image_idx:03d}", dpi=150, bbox_inches='tight')
            end=time.time()
            gap=end-start
            self.angle_path.write(f"{angle} {pp_angle} {gap}\n")
            self.angle_path.flush()
        
        return pp_angle