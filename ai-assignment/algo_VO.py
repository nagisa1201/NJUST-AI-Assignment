import render_map
import pygame
import numpy as np
import math


# === 配置迷宫地图 ===
mazeconfig = render_map.MazeMapConfig(rows=12, cols=12, cell_size=90, loop_percent=0, start_point=(1,1), goal_point=(12,12))
maze, maze_data, renderer = render_map.MazeMapConfig.create_scene(mazeconfig)

# === 配置动态障碍 ===
# 低速障碍物个数，速度范围，高速障碍物个数，速度范围, 半径大小
dynamic_obstacles = render_map.generate_dynamic_obstacles(
    num_slow=20, num_fast=18, slow_speed_range=(0.5,1),fast_speed_range=(2,3),
    map_width=mazeconfig.map_size[0], map_height=mazeconfig.map_size[1],radius=10,
    maze_walls=maze_data.walls
)
# 给每个动态障碍物分配唯一编号
for idx, obs in enumerate(dynamic_obstacles):
    obs.oid = idx

# obstacle_states = render_map.get_obstacle_states(dynamic_obstacles)

# === 配置机器人 ===
# 位置，初始速度，初始方向，半径
N7car = render_map.N7carRobot(
    x=maze_data.start_pixel[0],
    y=maze_data.start_pixel[1],
    speed=2,
    direction=(1,0),
    radius=15
)

# === 配置环境渲染器 ===
scene_renderer = render_map.SceneRenderer(maze_data, mazeconfig, dynamic_obstacles, N7car)


# === 进行动态避障 ===
class VO_Avoidance:
    def __init__(self, robot, obstacles, path_goal, inflate_ratio):
        self.robot = robot
        self.obstacles = obstacles
        self.vision_range = 100  # 视距范围
        self.inflate_ratio = inflate_ratio  # 膨胀系数
        self.path_goal = path_goal  # 规划路径点列表
        self.path_idx = 0  # 当前目标路径点索引
        self.last_velocity = np.zeros(2)  # 历史速度惯性
        
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
        robot_state: {'x', 'y', 'speed', 'direction'}
        obs_state: {'x', 'y', 'speed', 'direction'}
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
        left_angle = dir_angle + theta
        right_angle = dir_angle - theta
        left_leg = (math.cos(left_angle), math.sin(left_angle))
        right_leg = (math.cos(right_angle), math.sin(right_angle))
        apex = (ovx, ovy)
        return {
            'apex': apex,
            'left_leg': (apex[0] + left_leg[0], apex[1] + left_leg[1]),
            'right_leg': (apex[0] + right_leg[0], apex[1] + right_leg[1])
        }

    def build_VOs(self, obstacles, robot_state):
        VOs = []
        px, py = robot_state['x'], robot_state['y']
        considered_obs = []
        rx, ry = robot_state['x'], robot_state['y']
        for obs in obstacles:
            dist = math.hypot(obs.x - rx, obs.y - ry)
            if dist <= self.vision_range:
                vo = self.construct_obstacle_VO(robot_state, obs)
                VOs.append(vo)
                considered_obs.append(obs)
        print(f"N7car坐标: ({robot_state['x']:.1f}, {robot_state['y']:.1f})")
        if considered_obs:
            print("考虑进VO的动态障碍物坐标:")
            for obs in considered_obs:
                min_dist = math.hypot(obs.x - robot_state['x'], obs.y - robot_state['y'])
                print(f"  障碍物{getattr(obs, 'oid', '?')}: ({obs.x:.1f}, {obs.y:.1f})，此时距离{min_dist:.1f}")
                if min_dist < 18 :
                    print("!!! 警告: 障碍物过近，可能发生碰撞 !!!")
        else:
            print("视距内无动态障碍物被考虑进VO，当前速度为{}".format(robot_state['speed']))
        return VOs
    
    def select_velocity_outside_VOs(self, v_desired, VOs):
        vmax = 5.0
        # 如果没有VO，直接返回目标速度
        if not VOs or len(VOs) == 0:
            self.last_velocity = v_desired
            return v_desired
        candidates = []
        # 提高分辨率：半径20档，角度72档（每5度）
        for r in np.linspace(0.2, vmax, 20):
            for angle in np.linspace(0, 2 * np.pi, 72, endpoint=False):
                v = np.array([np.cos(angle), np.sin(angle)]) * r
                if not any(self.is_velocity_in_VO(v, vo) for vo in VOs):
                    candidates.append(v)

        # 只有在所有速度都不可行时才考虑静止
        if not candidates and not any(self.is_velocity_in_VO(np.zeros(2), vo) for vo in VOs):
            candidates.append(np.zeros(2))
        if candidates:
            # 历史速度惯性权重
            alpha = 0.6  # 目标速度权重
            beta = 0.2   # 目标方向权重
            gamma = 0.9  # 惯性权重
            last_v = self.last_velocity if hasattr(self, 'last_velocity') else np.zeros(2)
            def score(v):
                # 与目标速度接近、方向接近、与历史速度夹角小
                norm_v = np.linalg.norm(v)
                norm_vd = np.linalg.norm(v_desired)
                norm_last = np.linalg.norm(last_v)
                # 目标速度距离
                dist_score = np.linalg.norm(v - v_desired)
                # 目标方向夹角
                if norm_v > 1e-6 and norm_vd > 1e-6:
                    dir_score = np.arccos(np.clip(np.dot(v, v_desired) / (norm_v * norm_vd), -1, 1))
                else:
                    dir_score = 0
                # 与历史速度夹角
                if norm_v > 1e-6 and norm_last > 1e-6:
                    inertia_score = np.arccos(np.clip(np.dot(v, last_v) / (norm_v * norm_last), -1, 1))
                else:
                    inertia_score = 0
                return alpha * dist_score + beta * dir_score + gamma * inertia_score
            best = min(candidates, key=score)
            self.last_velocity = best
            return best
        else:
            self.last_velocity = np.zeros(2)
            return np.zeros(2)
    
    def compute_new_velocity(self, target_point, current_position):
        """
        计算N7car朝向目标点的合理速度，并避开所有VO，集成最小距离约束和紧急制动
        target_point: (x, y) 目标点像素坐标
        current_position: (x, y) 当前N7car像素坐标
        返回: 新速度向量 (vx, vy)
        """
        # 1. 计算目标方向
        direction = np.array(target_point) - np.array(current_position)
        distance = np.linalg.norm(direction)
        if distance > 1e-3:
            direction_normalized = direction / distance
        else:
            direction_normalized = np.zeros_like(direction)

        vmax = 5.0
        v_desired = direction_normalized * vmax

        # 2. 获取机器人当前状态（dict）
        robot_state = self.robot.get_N7car_state()

        # 3. 检查最小距离约束和紧急制动
        min_dist = None
        nearest_obs = None
        for obs in self.obstacles:
            obs_dist = math.hypot(obs.x - robot_state['x'], obs.y - robot_state['y'])
            safe_dist = self.robot.radius + obs.radius + 5
            if min_dist is None or obs_dist < min_dist:
                min_dist = obs_dist
                nearest_obs = obs
        # 优先远离最近障碍物
        if nearest_obs is not None:
            obs_dist = min_dist
            safe_dist = self.robot.radius + nearest_obs.radius + 5
            if obs_dist < safe_dist:
                # 采样所有可行速度，优先选与障碍物连线夹角>120度的速度
                robot_pos = np.array([robot_state['x'], robot_state['y']])
                obs_pos = np.array([nearest_obs.x, nearest_obs.y])
                away_dir = robot_pos - obs_pos
                if np.linalg.norm(away_dir) > 1e-3:
                    away_dir = away_dir / np.linalg.norm(away_dir)
                else:
                    away_dir = np.array([1.0, 0.0])
                # 采样最大速度远离障碍物
                vmax = 5.0  # 与N7car最大速度一致
                v_far = away_dir * vmax
                # 先判断最大速度远离是否可行
                VOs = self.build_VOs(self.obstacles, robot_state)
                if not any(self.is_velocity_in_VO(v_far, vo) for vo in VOs):
                    print(f"[紧急规避] N7car与障碍物{getattr(nearest_obs, 'oid', '?')} 距离{obs_dist:.1f} < 安全距离{safe_dist}，最大速度远离")
                    return v_far
                # 若最大速度不可行，再采样夹角>120度的速度
                candidates = []
                for r in np.linspace(0.2, vmax, 10):
                    for angle in np.linspace(0, 2 * np.pi, 36, endpoint=False):
                        v = np.array([np.cos(angle), np.sin(angle)]) * r
                        if np.dot(v, away_dir) / (np.linalg.norm(v) * np.linalg.norm(away_dir) + 1e-6) < -0.5:
                            if not any(self.is_velocity_in_VO(v, vo) for vo in VOs):
                                candidates.append(v)
                if candidates:
                    best = max(candidates, key=lambda v: np.dot(v, away_dir))
                    print(f"N7car 当前速度{robot_state['speed']:.1f}，与障碍物{getattr(nearest_obs, 'oid', '?')} 距离{obs_dist:.1f} < 安全距离{safe_dist}，当前障碍物速度为{nearest_obs.speed:.1f}，采样远离速度")
                    print(f"[紧急规避] N7car与障碍物{getattr(nearest_obs, 'oid', '?')} 距离{obs_dist:.1f} < 安全距离{safe_dist}，采样远离")
                    return best
                else:
                    print(f"N7car 当前速度{robot_state['speed']:.1f}，与障碍物{getattr(nearest_obs, 'oid', '?')} 距离{obs_dist:.1f} < 安全距离{safe_dist}，无可行远离速度")
                    print(f"[紧急制动] N7car与障碍物{getattr(nearest_obs, 'oid', '?')} 距离{obs_dist:.1f} < 安全距离{safe_dist}，无路可逃急停")
                    return np.zeros(2)

        # 4. 构造所有VO（包括墙体和动态障碍物）
        VOs = self.build_VOs(self.obstacles, robot_state)

        # 5. 搜索可行速度，选取最接近v_desired的
        v_new = self.select_velocity_outside_VOs(v_desired, VOs)

        return v_new
    
    def update(self):
        robot_state = self.robot.get_N7car_state()
        current_position = (robot_state['x'], robot_state['y'])
        # 路径点引导：依次跟踪path_goal中的点
        if not self.path_goal or self.path_idx >= len(self.path_goal):
            return
        target_point = self.path_goal[self.path_idx]
        # 判断是否到达当前目标点
        if np.linalg.norm(np.array(current_position) - np.array(target_point)) < max(5, self.robot.radius * 0.7):
            self.path_idx += 1
            if self.path_idx >= len(self.path_goal):
                self.robot.speed = 0
                return
            target_point = self.path_goal[self.path_idx]
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