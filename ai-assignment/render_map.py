import pygame
import pyamaze
from typing import Tuple
from dataclasses import dataclass
import random
import math

# ===Define of MazeMap and Mazemap generation===
clock = pygame.time.Clock()
#Define of Mazemap
@dataclass
class MazeMapConfig:
    rows:int
    cols:int
    cell_size:int
    loop_percent:int
    start_point:Tuple[int,int] = None
    goal_point:Tuple[int,int] = None
    def __post_init__(self):
        self.map_size = (self.cols * self.cell_size ,self.rows * self.cell_size)
    
#Define of Mazemap Generator
class MazeMapGenerator:
    def __init__(self, config:MazeMapConfig):
        self.config = config
        self.maze = pyamaze.maze(config.rows, config.cols)
    
    def generate_maze(self):
        self.maze.CreateMaze(loopPercent=self.config.loop_percent)
        return self.maze
    
@dataclass
class MazeDataForPygame:
    walls: list  # [(start, end), ...]
    start_pixel: tuple
    goal_pixel: tuple
    

#Define of Maze Renderer(using pygame)
class MazeRenderer:
    def __init__(self, maze_data, mazemap_config):
        self.maze_data = maze_data
        self.mazemap_config = mazemap_config
        width =  self.mazemap_config.cols * self.mazemap_config.cell_size
        height = self.mazemap_config.rows * self.mazemap_config.cell_size
        if width >1200 or height>1200:
            raise Exception("Maze size too large for display!!!!EXIST NOW")
        pygame.init()
        self.screen = pygame.display.set_mode((width, height))
        pygame.display.set_caption("Maze V0.1")

    def draw(self):
        self.screen.fill((255, 255, 255))  # 白色背景
        # 画墙
        for wall in self.maze_data.walls:
            start, end = wall
            pygame.draw.line(self.screen, (0, 0, 0), start, end, 2)
        # 画起点终点
        pygame.draw.circle(self.screen, (0, 0, 255), self.maze_data.start_pixel, 8)
        pygame.draw.circle(self.screen, (255, 0, 0), self.maze_data.goal_pixel, 8)
        pygame.display.flip()

    def quit(self):
        pygame.quit()

# Convert pyamaze maze to MazeDataForPygame
def convert_maze_to_pygame(maze, mazemap_config, cell_size=40):
    walls = []
    for (r, c), directions in maze.maze_map.items():
        x = (c - 1) * cell_size
        y = (r - 1) * cell_size
        # 四个角点
        tl = (x, y)
        tr = (x + cell_size, y)
        bl = (x, y + cell_size)
        br = (x + cell_size, y + cell_size)
        # 墙壁
        if directions['N'] == 0:
            walls.append((tl, tr))
        if directions['S'] == 0:
            walls.append((bl, br))
        if directions['W'] == 0:
            walls.append((tl, bl))
        if directions['E'] == 0:
            walls.append((tr, br))
    # 起点终点，只用 maze.start 和 maze.goal
    start_pixel = ((mazemap_config.start_point[1] - 0.5) * cell_size, (mazemap_config.start_point[0] - 0.5) * cell_size)
    goal_pixel = ((mazemap_config.goal_point[1] - 0.5) * cell_size, (mazemap_config.goal_point[0] - 0.5) * cell_size)
    return MazeDataForPygame(walls, start_pixel, goal_pixel)

#== Define of Maze Generation and Rendering Process==
mazemap_config = MazeMapConfig(start_point=(1,1), goal_point=(8, 8), rows=8, cols=8, cell_size=90, loop_percent=0)
map_size = mazemap_config.map_size
print(f"Maze Map Size: {map_size}")
maze_generator = MazeMapGenerator(mazemap_config)
maze_data = maze_generator.generate_maze()
maze_data_for_pygame = convert_maze_to_pygame(maze_data, mazemap_config=mazemap_config, cell_size=mazemap_config.cell_size)
maze_walls = maze_data_for_pygame.walls
mazerenderer = MazeRenderer(maze_data_for_pygame,mazemap_config)



#== Define of Dynamic Obstacle ===
class DynamicObstacle:
    def __init__(self, x, y, radius, speed, direction, obstacle_type, slow_speed_range=(1,3), fast_speed_range=(4,8)):
        self.x = x  # 像素坐标
        self.y = y
        self.radius = radius  # 障碍物大小
        self.speed = speed
        self.direction = direction  # 单位向量 (dx, dy)
        self.type = obstacle_type  # 'fast' or 'slow'
        self.slow_timer = 0  # 衰减计时
        self.slowing = False  # 是否处于减速/恢复过程
        self.slow_start_speed = speed  # 衰减起始速度
        self.slow_elapsed = 0  # 已经过的帧数
        self.slow_speed_range = slow_speed_range
        self.fast_speed_range = fast_speed_range
        self.w = 0.5 # 惯性系数

    def get_away_direction(self, next_pos, wall):
        # 计算圆心到墙最近点的向量，取反即为远离墙的方向（归一化+随机扰动）
        (x, y) = next_pos
        (x1, y1), (x2, y2) = wall
        # 线段长度为0
        if x1 == x2 and y1 == y2:
            dx = x - x1
            dy = y - y1
        else:
            t = ((x - x1) * (x2 - x1) + (y - y1) * (y2 - y1)) / ((x2 - x1) ** 2 + (y2 - y1) ** 2)
            t = max(0, min(1, t))
            closest_x = x1 + t * (x2 - x1)
            closest_y = y1 + t * (y2 - y1)
            dx = x - closest_x
            dy = y - closest_y
        # 归一化
        norm = math.hypot(dx, dy)
        if norm == 0:
            # 退化情况，随机方向
            base_dir = self.random_direction()
        else:
            base_dir = (dx / norm, dy / norm)
        # 加入小扰动，避免卡死
        angle = math.atan2(base_dir[1], base_dir[0])
        jitter = random.uniform(-math.pi/2, math.pi/2)  # ±90度扰动
        new_angle = angle + jitter
        return (math.cos(new_angle), math.sin(new_angle))
    
    def random_direction(self):
        angle = random.uniform(0, 2 * math.pi)
        return (math.cos(angle), math.sin(angle))
    
    def update(self, maze_walls):
        # 计算新位置
        new_x = self.x + self.direction[0] * self.speed
        new_y = self.y + self.direction[1] * self.speed

        
        # 检测碰撞标志位
        collided = False

        # 墙体碰撞检测（可用线段与圆碰撞算法）
        collided_wall = None
        for wall in maze_walls:
            if self.collide_with_wall((new_x, new_y), wall):
                collided = True
                collided_wall = wall
                break

        if collided:
            # 选择远离墙面的随机二维方向，避免主轴横跳
            self.direction = self.get_away_direction((new_x, new_y), collided_wall)
            # 不更新位置，停在原地
        else:
            self.x = new_x
            self.y = new_y
        
        if self.type == 'slow':
            # 随机改变方向和速度
            if random.random() < math.exp(-5*self.speed):
                self.direction = self.random_direction()
            if random.random() < 0.05:
                self.speed = self.w * self.speed + (1 - self.w) * random.uniform(*self.fast_speed_range)
        # 高速障碍物较少改变
        else:   
            if random.random() < math.exp(-5*self.speed):
                self.direction = self.random_direction()
            if random.random() < 0.05:
                self.speed = self.w * self.speed + (1 - self.w) * random.uniform(*self.fast_speed_range)
    def collide_with_wall(self, next_pos, wall):
        # next_pos: (x, y) 圆心预测位置
        (x, y) = next_pos
        (x1, y1), (x2, y2) = wall
        # 线段长度为0
        if x1 == x2 and y1 == y2:
            dist = ((x - x1) ** 2 + (y - y1) ** 2) ** 0.5
            return dist < self.radius
        # 投影参数t
        t = ((x - x1) * (x2 - x1) + (y - y1) * (y2 - y1)) / ((x2 - x1) ** 2 + (y2 - y1) ** 2)
        t = max(0, min(1, t))
        closest_x = x1 + t * (x2 - x1)
        closest_y = y1 + t * (y2 - y1)
        dist = ((x - closest_x) ** 2 + (y - closest_y) ** 2) ** 0.5
        return dist < self.radius
    

class DynamicObstacleRenderer:
    def __init__(self, obstacles, map_width, map_height):
        self.obstacles = obstacles
        self.map_width = map_width
        self.map_height = map_height
        pygame.init()
        self.screen = pygame.display.set_mode((map_width, map_height))
        pygame.display.set_caption("Dynamic Obstacles V0.1")
    def draw(self):
        self.screen.fill((255, 255, 255))  # 白色背景
        for obs in self.obstacles:
            pygame.draw.circle(self.screen, (255, 0, 0), (int(obs.x), int(obs.y)), obs.radius)
        pygame.display.flip()
    def quit(self):
        pygame.quit()

def generate_dynamic_obstacles(
    num_slow, num_fast,
    slow_speed_range,
    fast_speed_range,
    map_width=700, map_height=700,
    radius=10
):
    obstacles = []
    def is_valid_position(x, y, radius, maze_walls):
    # 检查(x, y)为圆心，半径为radius时，是否与任意墙体重叠
        for wall in maze_walls:
            (x1, y1), (x2, y2) = wall
            # 线段与圆的最小距离
            if x1 == x2 and y1 == y2:
                dist = ((x - x1) ** 2 + (y - y1) ** 2) ** 0.5
            else:
                t = ((x - x1) * (x2 - x1) + (y - y1) * (y2 - y1)) / ((x2 - x1) ** 2 + (y2 - y1) ** 2)
                t = max(0, min(1, t))
                closest_x = x1 + t * (x2 - x1)
                closest_y = y1 + t * (y2 - y1)
                dist = ((x - closest_x) ** 2 + (y - closest_y) ** 2) ** 0.5
            if dist < radius + 1:  # +1是安全裕度
                return False
        return True
    def random_direction():
        angle = random.uniform(0, 2 * math.pi)
        return (math.cos(angle), math.sin(angle))
    for _ in range(num_slow):
        for _ in range(20):
            x = random.uniform(radius, map_width - radius)
            y = random.uniform(radius, map_height - radius)
            if is_valid_position(x, y, radius, maze_walls):
                break
        speed = random.uniform(*slow_speed_range)
        direction = random_direction()
        obstacles.append(DynamicObstacle(x, y, radius, speed, direction, 'slow', slow_speed_range=slow_speed_range, fast_speed_range=fast_speed_range))
    for _ in range(num_fast):
        for _ in range(20):
            x = random.uniform(radius, map_width - radius)
            y = random.uniform(radius, map_height - radius)
            if is_valid_position(x, y, radius, maze_walls):
                break
        speed = random.uniform(*fast_speed_range)
        direction = random_direction()
        obstacles.append(DynamicObstacle(x, y, radius, speed, direction, 'fast', slow_speed_range=slow_speed_range, fast_speed_range=fast_speed_range))
    return obstacles

#== Define of the N7car Robot ==
# N7 represents Nagisa and 7415
class N7carRobot:
    def __init__(self, x, y, speed, direction, radius=15):
        self.x = x
        self.y = y
        self.radius = radius
        self.speed = speed
        self.direction = direction
        self.goal = False # 是否到达目标点

    def update(self, maze_walls):
        # 计算新位置
        self.x += self.direction[0] * self.speed
        self.y += self.direction[1] * self.speed

    def draw(self):
        pygame.draw.circle(self.screen, (255, 255, 255), (int(self.x), int(self.y)), self.radius)

#== Define the Dynamic Scene Renderer ==
class SceneRenderer:
    def __init__(self, maze_data, mazemap_config, dynamic_obstacles, N7car):
        self.maze_data = maze_data
        self.mazemap_config = mazemap_config
        self.dynamic_obstacles = dynamic_obstacles
        self.N7car = N7car  # 之后可以添加N7car机器人实例
        width = self.mazemap_config.cols * self.mazemap_config.cell_size
        height = self.mazemap_config.rows * self.mazemap_config.cell_size
        if width > 1200 or height > 1200:
            raise Exception("Maze size too large for display!!!!EXIST NOW")
        pygame.init()
        self.screen = pygame.display.set_mode((width, height))
        pygame.display.set_caption("Maze & Obstacles")

    def draw(self):
        self.screen.fill((255, 255, 255))  # 白色背景
        # 画墙
        for wall in self.maze_data.walls:
            start, end = wall
            pygame.draw.line(self.screen, (0, 0, 0), start, end, 2)
        # 画起点终点
        pygame.draw.circle(self.screen, (0, 0, 255), self.maze_data.start_pixel, 8)
        pygame.draw.circle(self.screen, (255, 0, 0), self.maze_data.goal_pixel, 8)
        # 画动态障碍物
        for obs in self.dynamic_obstacles:
            color = (255, 0, 0) if obs.type == 'fast' else (0, 128, 255)
            pygame.draw.circle(self.screen, color, (int(obs.x), int(obs.y)), obs.radius)
        pygame.draw.circle(self.screen, (0, 0, 0), (int(self.N7car.x), int(self.N7car.y)), self.N7car.radius)
        pygame.display.flip()

    def quit(self):
        pygame.quit()
#=== init the dynamic obstacles ===
dynamic_obstacles = generate_dynamic_obstacles(
    num_slow=8, num_fast=3, slow_speed_range=(0.1,0.3),fast_speed_range=(1,2),map_width=map_size[0], map_height=map_size[1],radius=10
)

N7car = N7carRobot(
    x=maze_data_for_pygame.start_pixel[0],
    y=maze_data_for_pygame.start_pixel[1],
    speed=2,
    direction=(1,0),
    radius=15
)
#=== init the scene ===
scene_renderer = SceneRenderer(maze_data_for_pygame, mazemap_config, dynamic_obstacles, N7car)
# 每帧或每次需要时都可以这样获取
obstacle_states = [
    {
        'x': obs.x,
        'y': obs.y,
        'speed': obs.speed,
        'direction': obs.direction
    }
    for obs in dynamic_obstacles
]
#=== start the render loop ===
running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
    # 更新动态障碍物
    for obs in dynamic_obstacles:
        obs.update(maze_walls)
    obstacle_states = [
    {
        'x': obs.x,
        'y': obs.y,
        'speed': obs.speed,
        'direction': obs.direction
    }
    for obs in dynamic_obstacles
    ]
    print(obstacle_states)
    clock.tick(30)
    # 统一渲染
    scene_renderer.draw()
scene_renderer.quit()
