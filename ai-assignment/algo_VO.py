import render_map
import pygame
import numpy as np
import math


# === 配置迷宫地图 ===
mazeconfig = render_map.MazeMapConfig(rows=12, cols=12, cell_size=90, loop_percent=80, start_point=(1,1), goal_point=(8,8))
maze, maze_data, renderer = render_map.MazeMapConfig.create_scene(mazeconfig)

# === 配置动态障碍 ===
# 低速障碍物个数，速度范围，高速障碍物个数，速度范围, 半径大小
dynamic_obstacles = render_map.generate_dynamic_obstacles(
    num_slow=20, num_fast=20, slow_speed_range=(0.5,1),fast_speed_range=(1.5,3),
    map_width=mazeconfig.map_size[0], map_height=mazeconfig.map_size[1],radius=10,
    maze_walls=maze_data.walls
)
obstacle_states = render_map.get_obstacle_states(dynamic_obstacles)
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
    def __init__(self, robot, obstacles, maze_walls):
        self.robot = robot
        self.obstacles = obstacles
        self.maze_walls = maze_walls
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
        R = robot_radius + obs_radius
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

    def construct_wall_VO(self, robot_state, wall, robot_radius=15):
        """
        构造静态墙体的VO
        wall: ((x1, y1), (x2, y2))
        """
        px, py = robot_state['x'], robot_state['y']
        (x1, y1), (x2, y2) = wall
        dx, dy = x2 - x1, y2 - y1
        if dx == 0 and dy == 0:
            closest = (x1, y1)
        else:
            t = ((px - x1) * dx + (py - y1) * dy) / (dx*dx + dy*dy)
            t = max(0, min(1, t))
            closest = (x1 + t * dx, y1 + t * dy)
        rel_pos = (closest[0] - px, closest[1] - py)
        dist = math.hypot(rel_pos[0], rel_pos[1])
        R = robot_radius
        if dist < 1e-6 or R > dist:
            theta = math.pi
        else:
            theta = math.asin(R / dist)
        dir_angle = math.atan2(rel_pos[1], rel_pos[0])
        left_angle = dir_angle + theta
        right_angle = dir_angle - theta
        left_leg = (math.cos(left_angle), math.sin(left_angle))
        right_leg = (math.cos(right_angle), math.sin(right_angle))
        apex = (0, 0)
        return {
            'apex': apex,
            'left_leg': left_leg,
            'right_leg': right_leg
        }
    def build_VOs(self, obstacles, walls, robot_state):
        VOs = []
        for wall in walls:
            vo = self.construct_wall_VO(robot_state, wall)
            VOs.append(vo)
        for obs in obstacles:
            vo = self.construct_obstacle_VO(robot_state, obs)
            VOs.append(vo)
        return VOs
    def select_velocity_outside_VOs(self, v_desired, VOs):
        # 简单示例：如果v_desired在任何VO内，则减速
        for vo in VOs:
            if self.is_velocity_in_VO(v_desired, vo):
                return v_desired * 0.5  # 简单减速策略
        return v_desired
    
    def compute_new_velocity(self, target_point, current_position):
        """
        计算N7car朝向目标点的合理速度，并避开所有VO
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

        vmax = 2.0
        v_desired = direction_normalized * min(vmax, distance * 0.5)

        # 2. 获取机器人当前状态（dict）
        robot_state = self.robot.get_N7car_state()

        # 3. 构造所有VO（包括墙体和动态障碍物）
        VOs = self.build_VOs(self.obstacles, self.maze_walls, robot_state)

        # 4. 搜索可行速度，选取最接近v_desired的
        v_new = self.select_velocity_outside_VOs(v_desired, VOs)

        return v_new
    
    def update(self):
        robot_state = self.robot.get_N7car_state()
        current_position = (robot_state['x'], robot_state['y'])
        target_point = maze_data.goal_pixel
        new_velocity = self.compute_new_velocity(target_point, current_position)
        speed = np.linalg.norm(new_velocity)
        if speed > 1e-3:
            direction = new_velocity / speed
        else:
            direction = (0,0)
        self.robot.speed = speed
        self.robot.direction = direction

avoidance = VO_Avoidance(N7car, dynamic_obstacles, maze_data.walls)
       
# === 开始渲染 ===
clock = pygame.time.Clock()
running = True
pygame.font.init() 
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
    # 更新动态障碍物
    for obs in dynamic_obstacles:
        obs.update(maze_data.walls)
    avoidance.update()
    N7car.update()
    clock.tick(30)
    # 统一渲染
    scene_renderer.draw()
    pygame.display.flip()
    
scene_renderer.quit()