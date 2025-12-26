import render_map
import pygame
import numpy as np
import math


# === 配置迷宫地图 ===
mazeconfig = render_map.MazeMapConfig(rows=12, cols=12, cell_size=90, loop_percent=0, start_point=(1,1), goal_point=(12,12))
maze, maze_data, renderer = render_map.MazeMapConfig.create_scene(mazeconfig)

# === 配置动态障碍 ===
# 低速障碍物个数，速度范围，高速障障碍物个数，速度范围, 半径大小

# 给每个动态障碍物分配唯一编号

# obstacle_states = render_map.get_obstacle_states(dynamic_obstacles)

# === 配置机器人 ===
# 位置，初始速度，初始方向，半径
# N7car = render_map.N7carRobot(
#     x=maze_data.start_pixel[0],
#     y=maze_data.start_pixel[1],
#     speed=2,
#     direction=(1,0),
#     radius=15
# )

# === 配置环境渲染器 ===
# scene_renderer = render_map.SceneRenderer(maze_data, mazeconfig, dynamic_obstacles, N7car)


# === 进行动态避障 ===
class VO_Avoidance:
            
    def __init__(self, robot, v_max, obstacles, path_goal, inflate_ratio, vision_range, maze_data):
        self.robot = robot
        self.obstacles = obstacles
        self.vision_range = vision_range  # 视距范围
        self.inflate_ratio = inflate_ratio  # 膨胀系数
        self.path_goal = path_goal  # 规划路径点列表
        self.path_idx = 0  # 当前目标路径点索引
        self.last_velocity = np.zeros(2)  # 历史速度惯性
        self.a_max = 10.0  # 默认最大加速度
        self.v_max = v_max  # 最大速度
        self.maze_data = maze_data  # 迷宫数据，包含walls

    def set_speed_limit(self, v_max):
        self.v_max = float(v_max)

    def set_acc_limit(self, a_max):
        self.a_max = float(a_max)

    def is_velocity_in_VO(self, v, VO):
        """
        判断速度 v 是否在 VO 区域内
        v: (vx, vy)
        VO: {'apex': (vx, vy), 'left_leg': (lx, ly), 'right_leg': (rx, ry)}
        """
        rel_v = (v[0] - VO['apex'][0], v[1] - VO['apex'][1])
        left = (VO['left_leg'][0] - VO['apex'][0], VO['left_leg'][1] - VO['apex'][1])
        right = (VO['right_leg'][0] - VO['apex'][0], VO['right_leg'][1] - VO['apex'][1])
        # 叉积判断 rel_v 是否在 left 和 right 之间
        cross1 = left[0]*rel_v[1] - left[1]*rel_v[0]
        cross2 = rel_v[0]*right[1] - rel_v[1]*right[0]
        return cross1 >= 0 and cross2 >= 0

    def construct_obstacle_VO(self, robot_state, obs_state, robot_radius=15, obs_radius=10):
        """
        构造动态障碍物的VO
        修正：确保VO区危险区朝向障碍物，安全区远离障碍物
        """
        px, py = robot_state['x'], robot_state['y']
        ox, oy = obs_state.x, obs_state.y
        ovx = obs_state.speed * obs_state.direction[0]
        ovy = obs_state.speed * obs_state.direction[1]
        rel_pos = (ox - px, oy - py)
        dist = math.hypot(rel_pos[0], rel_pos[1])
        R = robot_radius + obs_radius * self.inflate_ratio
        if dist < 1e-6 or R > dist:
            theta = math.pi
        else:
            theta = math.asin(R / dist)
        dir_angle = math.atan2(rel_pos[1], rel_pos[0])
        # 修正：交换左右腿定义，保证危险区朝向障碍物
        left_angle = dir_angle - theta
        right_angle = dir_angle + theta
        left_leg = (math.cos(left_angle), math.sin(left_angle))
        right_leg = (math.cos(right_angle), math.sin(right_angle))
        apex = (ovx, ovy)
        return {
            'apex': apex,
            'left_leg': (apex[0] + left_leg[0], apex[1] + left_leg[1]),
            'right_leg': (apex[0] + right_leg[0], apex[1] + right_leg[1])
        }

    def filter_obstacles_in_vision(self, obstacles, robot_state):
        """
        返回在视距范围内且与机器人之间无墙阻挡的障碍物列表
        """
        rx, ry = robot_state['x'], robot_state['y']
        considered_obs = []
        def ccw(A, B, C):
            return (C[1]-A[1])*(B[0]-A[0]) > (B[1]-A[1])*(C[0]-A[0])
        def segments_intersect(A, B, C, D):
            return ccw(A, C, D) != ccw(B, C, D) and ccw(A, B, C) != ccw(A, B, D)
        walls = getattr(self.maze_data, 'walls', [])
        for obs in obstacles:
            # 支持 obs 为对象或字典
            if hasattr(obs, 'x') and hasattr(obs, 'y'):
                ox, oy = obs.x, obs.y
            elif isinstance(obs, dict) and 'x' in obs and 'y' in obs:
                ox, oy = obs['x'], obs['y']
            else:
                continue
            dist = math.hypot(ox - rx, oy - ry)
            if dist <= self.vision_range:
                blocked = False
                for wall in walls:
                    # wall: ((x1, y1), (x2, y2))
                    (x1, y1), (x2, y2) = wall
                    if segments_intersect((rx, ry), (ox, oy), (x1, y1), (x2, y2)):
                        blocked = True
                        break
                if not blocked:
                    considered_obs.append(obs)
        return considered_obs
    def visualize_velocity_space(self, screen, robot_state, VOs, num_angles=72, num_radii=20):
        """
        在速度空间采样，显示哪些速度是安全的，哪些是危险的
        修正：采样速度v应以机器人为参考，判定时用v与障碍物速度的相对关系
        """
        cx, cy = int(robot_state['x']), int(robot_state['y'])
        vmax = self.v_max
        for r in np.linspace(0.2, vmax, num_radii):
            for angle in np.linspace(0, 2 * np.pi, num_angles, endpoint=False):
                vx = math.cos(angle) * r
                vy = math.sin(angle) * r
                v = np.array([vx, vy])
                in_vo = False
                for vo in VOs:
                    rel_v = (v[0] - vo['apex'][0], v[1] - vo['apex'][1])
                    left = (vo['left_leg'][0] - vo['apex'][0], vo['left_leg'][1] - vo['apex'][1])
                    right = (vo['right_leg'][0] - vo['apex'][0], vo['right_leg'][1] - vo['apex'][1])
                    cross1 = left[0]*rel_v[1] - left[1]*rel_v[0]
                    cross2 = rel_v[0]*right[1] - rel_v[1]*right[0]
                    if cross1 >= 0 and cross2 >= 0:
                        in_vo = True
                        break
                # 颜色透明度处理（50%），圆点变细（半径2）
                base_color = (255, 0, 0) if in_vo else (0, 255, 0)
                color = (*base_color, 50)  # RGBA, 50为非常高透明度
                px = int(cx + vx * 8)
                py = int(cy + vy * 8)
                surf = pygame.Surface((4,4), pygame.SRCALPHA)
                pygame.draw.circle(surf, color, (2,2), 2)
                screen.blit(surf, (px-2, py-2))
    def build_VOs(self, obstacles, robot_state):
        VOs = []
        considered_obs = self.filter_obstacles_in_vision(obstacles, robot_state)
        for obs in considered_obs:
            vo = self.construct_obstacle_VO(robot_state, obs)
            VOs.append(vo)
        print(f"N7car坐标: ({robot_state['x']:.1f}, {robot_state['y']:.1f})")
        collision_flag = False
        if considered_obs:
            print("考虑进VO的动态障碍物坐标:")
            for obs in considered_obs:
                min_dist = math.hypot(obs.x - robot_state['x'], obs.y - robot_state['y'])
                print(f"  障碍物{getattr(obs, 'oid', '?')}: ({obs.x:.1f}, {obs.y:.1f})，此时距离{min_dist:.1f}")
                if min_dist < 18 :
                    print("!!! 警告: 障碍物过近，可能发生碰撞 !!!")
                    collision_flag = True
        else:
            print("视距内无动态障碍物被考虑进VO，当前速度为{}".format(robot_state['speed']))
        # 新增：如有碰撞，调用回调
        if collision_flag and hasattr(self, 'collision_callback'):
            self.collision_callback()
        return VOs
    
    def select_velocity_outside_VOs(self, v_desired, VOs, danger_away_dir=None):
        if self.filter_obstacles_in_vision(self.obstacles, self.robot.get_N7car_state()) != []:
            vmax = self.v_max * 0.8  # 视距内有障碍物时降低速度上限
        else:
            vmax = self.v_max
        last_v = self.last_velocity if hasattr(self, 'last_velocity') else np.zeros(2)
        # 危险时优先尝试最大加速度远离障碍物（只受加速度和速度限制）
        if danger_away_dir is not None:
            v_far = last_v + danger_away_dir * self.a_max
            v_far_norm = np.linalg.norm(v_far)
            if v_far_norm > vmax:
                v_far = v_far / v_far_norm * vmax
            acc = np.linalg.norm(v_far - last_v)
            if acc <= self.a_max + 1e-6 and not any(self.is_velocity_in_VO(v_far, vo) for vo in VOs):
                self.last_velocity = v_far
                return v_far
        # 如果没有VO，直接返回目标速度（但要满足加速度和最大速度约束）
        if not VOs or len(VOs) == 0:
            acc = np.linalg.norm(v_desired - last_v)
            v_norm = np.linalg.norm(v_desired)
            if acc <= self.a_max + 1e-6 and v_norm <= vmax + 1e-6:
                self.last_velocity = v_desired
                return v_desired
            # 超过加速度/速度限制，按加速度极限方向逼近
            direction = v_desired - last_v
            if np.linalg.norm(direction) > 1e-6:
                v = last_v + direction / np.linalg.norm(direction) * min(self.a_max, np.linalg.norm(direction))
            else:
                v = last_v
            v_norm = np.linalg.norm(v)
            if v_norm > vmax:
                v = v / v_norm * vmax
            self.last_velocity = v
            return v
        candidates = []
        # 提高分辨率：半径20档，角度72档（每5度）
        for r in np.linspace(0.2, vmax, 20):
            for angle in np.linspace(0, 2 * np.pi, 72, endpoint=False):
                v = np.array([np.cos(angle), np.sin(angle)]) * r
                acc = np.linalg.norm(v - last_v)
                v_norm = np.linalg.norm(v)
                if acc > self.a_max + 1e-6 or v_norm > vmax + 1e-6:
                    continue
                if not any(self.is_velocity_in_VO(v, vo) for vo in VOs):
                    candidates.append(v)
        if candidates:
            alpha = 0.6  # 目标速度权重
            beta = 0.2   # 目标方向权重
            gamma = 0.9  # 惯性权重
            def score(v):
                norm_v = np.linalg.norm(v)
                norm_vd = np.linalg.norm(v_desired)
                norm_last = np.linalg.norm(last_v)
                dist_score = np.linalg.norm(v - v_desired)
                if norm_v > 1e-6 and norm_vd > 1e-6:
                    dir_score = np.arccos(np.clip(np.dot(v, v_desired) / (norm_v * norm_vd), -1, 1))
                else:
                    dir_score = 0
                if norm_v > 1e-6 and norm_last > 1e-6:
                    inertia_score = np.arccos(np.clip(np.dot(v, last_v) / (norm_v * norm_last), -1, 1))
                else:
                    inertia_score = 0
                return alpha * dist_score + beta * dir_score + gamma * inertia_score
            best = min(candidates, key=score)
            self.last_velocity = best
            return best
        # 没有可行速度，优先尝试最大加速度远离障碍物
        if danger_away_dir is not None:
            v = last_v + danger_away_dir * self.a_max
            v_norm = np.linalg.norm(v)
            if v_norm > vmax:
                v = v / v_norm * vmax
            self.last_velocity = v
            return v
        # 否则，强制加速度极限逼近目标速度
        direction = v_desired - last_v
        if np.linalg.norm(direction) > 1e-6:
            v = last_v + direction / np.linalg.norm(direction) * min(self.a_max, np.linalg.norm(direction))
        else:
            v = last_v
        v_norm = np.linalg.norm(v)
        if v_norm > vmax:
            v = v / v_norm * vmax
        self.last_velocity = v
        return v
    
    def compute_new_velocity(self, target_point, current_position):
        """
        计算N7car朝向目标点的合理速度，并避开所有VO，集成最大加速度和最大速度约束，危险时优先远离障碍物
        """
        # 1. 计算目标方向
        direction = np.array(target_point) - np.array(current_position)
        distance = np.linalg.norm(direction)
        if distance > 1e-3:
            direction_normalized = direction / distance
        else:
            direction_normalized = np.zeros_like(direction)

        v_desired = direction_normalized * self.v_max

        # 2. 获取机器人当前状态（dict）
        robot_state = self.robot.get_N7car_state()

        # 3. 检查最小距离约束，危险时优先远离
        min_dist = None
        nearest_obs = None
        for obs in self.obstacles:
            obs_dist = math.hypot(obs.x - robot_state['x'], obs.y - robot_state['y'])
            safe_dist = self.robot.radius + obs.radius + 5
            if min_dist is None or obs_dist < min_dist:
                min_dist = obs_dist
                nearest_obs = obs
        danger_away_dir = None
        if nearest_obs is not None:
            obs_dist = min_dist
            safe_dist = self.robot.radius + nearest_obs.radius + 5
            if obs_dist < safe_dist:
                robot_pos = np.array([robot_state['x'], robot_state['y']])
                obs_pos = np.array([nearest_obs.x, nearest_obs.y])
                away_dir = robot_pos - obs_pos
                if np.linalg.norm(away_dir) > 1e-3:
                    away_dir = away_dir / np.linalg.norm(away_dir)
                else:
                    away_dir = np.array([1.0, 0.0])
                danger_away_dir = away_dir

        # 4. 构造所有VO（包括动态障碍物）
        VOs = self.build_VOs(self.obstacles, robot_state)

        # 5. 搜索可行速度，危险时优先最大加速度远离，否则正常采样
        v_new = self.select_velocity_outside_VOs(v_desired, VOs, danger_away_dir)
        return v_new
    
    def update(self):
        robot_state = self.robot.get_N7car_state()
        current_position = (robot_state['x'], robot_state['y'])
        # 支持嵌套路径点列表（如[[p1,...,p9],[p10,...,p18],...]），每个子列表为一个粗网格的细分点
        # 新策略：遍历当前子列表所有点，筛选VO外的安全点，选与下一个子列表中心点欧氏距离最小的点作为目标点
        if not self.path_goal or self.path_idx >= len(self.path_goal):
            return
        current_segment = self.path_goal[self.path_idx]
        # 计算当前VO遮蔽区
        robot_state = self.robot.get_N7car_state()
        VOs = self.build_VOs(self.obstacles, robot_state)
        # 获取下一个子列表的中心点（自适应点集大小，取几何中心附近的点）
        def get_center_point(segment):
            if not segment:
                return None
            arr = np.array(segment)
            center = np.mean(arr, axis=0)
            # 找到距离均值最近的点
            return min(segment, key=lambda p: np.linalg.norm(np.array(p) - center))
        if self.path_idx + 1 < len(self.path_goal):
            next_segment = self.path_goal[self.path_idx + 1]
            ref_point = get_center_point(next_segment)
        else:
            ref_point = get_center_point(current_segment)

        # 筛选VO外的安全点
        safe_points = []
        for pt in current_segment:
            v = np.array(pt) - np.array([robot_state['x'], robot_state['y']])
            # 判断该点对应的速度是否在所有VO外
            if not any(self.is_velocity_in_VO(v, vo) for vo in VOs):
                safe_points.append(pt)
        # 如果没有安全点，降级为所有点都可选
        if not safe_points:
            safe_points = current_segment
        # 选择与参考点欧氏距离最小的安全点
        target_point = min(safe_points, key=lambda p: np.linalg.norm(np.array(p) - np.array(ref_point)))

        # 判断是否到达当前目标点
        if np.linalg.norm(np.array(current_position) - np.array(target_point)) < 3:
            self.path_idx += 1
            if self.path_idx >= len(self.path_goal):
                self.robot.speed = 0
                return
            # 进入下一个子列表，重新筛选目标点
            current_segment = self.path_goal[self.path_idx]
            # 重新计算VO和参考点
            robot_state = self.robot.get_N7car_state()
            VOs = self.build_VOs(self.obstacles, robot_state)
            if self.path_idx + 1 < len(self.path_goal):
                next_segment = self.path_goal[self.path_idx + 1]
                if len(next_segment) >= 5:
                    ref_point = next_segment[4]
                else:
                    ref_point = next_segment[len(next_segment)//2]
            else:
                if len(current_segment) >= 5:
                    ref_point = current_segment[4]
                else:
                    ref_point = current_segment[len(current_segment)//2]
            safe_points = []
            for pt in current_segment:
                v = np.array(pt) - np.array([robot_state['x'], robot_state['y']])
                if not any(self.is_velocity_in_VO(v, vo) for vo in VOs):
                    safe_points.append(pt)
            if not safe_points:
                safe_points = current_segment
            target_point = min(safe_points, key=lambda p: np.linalg.norm(np.array(p) - np.array(ref_point)))
            self.target_point = target_point
            robot_state = self.robot.get_N7car_state()
            VOs = self.build_VOs(self.obstacles, robot_state)
            if hasattr(self, 'scene_renderer') and self.scene_renderer is not None:
                # 速度空间安全区可视化
                self.visualize_velocity_space(self.scene_renderer.screen, robot_state, VOs)
                self.target_point = target_point
            print("N7car目标点:", self.target_point)

        new_velocity = self.compute_new_velocity(target_point, current_position)
        speed = np.linalg.norm(new_velocity)
        if speed > 1e-3:
            direction = new_velocity / speed
        else:
            direction = (0,0)
        self.robot.speed = speed
        self.robot.direction = direction
        self.last_velocity = new_velocity

# avoidance = VO_Avoidance(N7car, dynamic_obstacles, inflate_ratio=5.5)
       
# === 开始渲染 ===
# clock = pygame.time.Clock()
# running = True
# pygame.font.init() 
# while running:
#     for event in pygame.event.get():
#         if event.type == pygame.QUIT:
#             running = False
#     # 更新动态障碍物
#     for obs in dynamic_obstacles:
#         obs.update(maze_data.walls)
#     avoidance.update()
#     N7car.update()
#     clock.tick(30)
#     # 统一渲染
#     scene_renderer.draw()
#     pygame.display.flip()
    
# scene_renderer.quit()