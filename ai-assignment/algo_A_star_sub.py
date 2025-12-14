'''
Author: Nagisa 2964793117@qq.com
Date: 2025-12-13 18:37:10
LastEditors: Nagisa 2964793117@qq.com
LastEditTime: 2025-12-14 23:14:37
FilePath: /NJUST-AI-Assignment/ai-assignment/algo_A_star_sub.py
Description: A*路径规划后，将原始粗网格路径点细分为高分辨率的网格路径点，并返回列表的列表元组结构。
'''

import render_map
import math 
import pygame
import numpy as np
import heapq
from typing import Tuple, List, Dict, Union

# ===================== 配置细化因子 =====================
# 每个原始粗网格会被等分成 FACTOR x FACTOR 个细网格
# 例如 FACTOR=3，则地图分辨率提高 3 倍。
SUBDIVISION_FACTOR = 3 
# ======================================================


# 首先配置迷宫地图
mazeconfig = render_map.MazeMapConfig(rows=12, cols=12, cell_size=90, loop_percent=80, start_point=(1,1), goal_point=(8,8))
maze, maze_data, renderer = render_map.MazeMapConfig.create_scene(mazeconfig)


# ===================== 坐标转换工具类 (包含细网格转换) =====================
class PointTF:
    """封装了像素坐标和网格坐标之间转换的静态方法，包含粗细网格转换。"""
    
    @staticmethod
    def pixel_to_grid(x: float, y: float, cell_size: int = mazeconfig.cell_size) -> Tuple[int, int]:
        """将像素坐标 (x, y) 转换为 1-based 粗网格 (row, col)"""
        col = int(x / cell_size) + 1
        row = int(y / cell_size) + 1
        return max(1, min(mazeconfig.rows, row)), max(1, min(mazeconfig.cols, col))

    @staticmethod
    def grid_to_pixel_center(r: int, c: int, cell_size: int = mazeconfig.cell_size) -> Tuple[float, float]:
        """将 1-based 粗网格 (row, col) 转换为像素中心点 (x, y)"""
        x = (c - 0.5) * cell_size
        y = (r - 0.5) * cell_size
        return x, y
        
    @staticmethod
    def subgrid_to_pixel_center(r_sub: int, c_sub: int, maze_config, factor: int = SUBDIVISION_FACTOR) -> Tuple[float, float]:
        """将 1-based 细网格 (r_sub, c_sub) 转换为像素中心点 (x, y)"""
        cell_size_sub = maze_config.cell_size / factor
        x = (c_sub - 0.5) * cell_size_sub
        y = (r_sub - 0.5) * cell_size_sub
        return x, y
        
    @staticmethod
    def subdivide_grid_to_fine_points(r_orig: int, c_orig: int, factor: int) -> List[Tuple[int, int]]:
        """
        将一个粗网格坐标 (r_orig, c_orig) 映射到其包含的 FACTOR x FACTOR 个细网格坐标。
        """
        fine_points = []
        # 计算该粗网格在细网格坐标系中的起始索引（1-based）
        r_start = (r_orig - 1) * factor + 1
        c_start = (c_orig - 1) * factor + 1
        
        # 遍历该粗网格包含的所有细网格
        for dr in range(factor):
            for dc in range(factor):
                r_sub = r_start + dr
                c_sub = c_start + dc
                fine_points.append((r_sub, c_sub))
                
        return fine_points

    @staticmethod
    def subgrid_to_origrid(r_sub: int, c_sub: int, factor: int) -> Tuple[int, int]:
        """将细网格坐标 (r_sub, c_sub) 转换为原始粗网格坐标 (r_orig, c_orig)"""
        # (r_sub - 1) // factor 得到 0-based 索引，+ 1 得到 1-based 索引
        r_orig = (r_sub - 1) // factor + 1
        c_orig = (c_sub - 1) // factor + 1
        return r_orig, c_orig

    @staticmethod
    def origrid_to_subgrid_center(r_orig: int, c_orig: int, factor: int) -> Tuple[int, int]:
        """将原始粗网格 (r_orig, c_orig) 的中心点（细网格坐标）"""
        # (factor + 1) // 2 得到中心点的偏移量（1-based）
        center_offset = (factor + 1) // 2 
        r_sub = (r_orig - 1) * factor + center_offset
        c_sub = (c_orig - 1) * factor + center_offset
        return r_sub, c_sub


# ===================== A* 算法的实现（静态避障） =====================
class AStarInit:
    def __init__(self, maze_data):
        start_grid = PointTF.pixel_to_grid(*maze_data.start_pixel)
        goal_grid = PointTF.pixel_to_grid(*maze_data.goal_pixel)
        self.start_grid = start_grid
        self.goal_grid = goal_grid

class AStarStaticPlanner:
    def __init__(self, maze_map, maze_config):
        self.maze_map = maze_map.maze_map 
        self.rows = maze_config.rows
        self.cols = maze_config.cols

    def heuristic(self, a: Tuple[int, int], b: Tuple[int, int]) -> int:
        """启发函数：曼哈顿距离 (Manhattan Distance)"""
        r1, c1 = a
        r2, c2 = b
        return abs(r1 - r2) + abs(c1 - c2)

    def get_neighbors(self, current: Tuple[int, int]) -> List[Tuple[int, int]]:
        """获取当前单元格 (r, c) 的可通行邻居"""
        r, c = current
        neighbors = []
        moves = {
            'N': (-1, 0), 'S': (1, 0), 'E': (0, 1), 'W': (0, -1)
        }
        current_cell_data = self.maze_map.get(current)
        if not current_cell_data:
            return []

        for direction, (dr, dc) in moves.items():
            if current_cell_data.get(direction) == 1:
                nr, nc = r + dr, c + dc
                if 1 <= nr <= self.rows and 1 <= nc <= self.cols:
                    neighbors.append((nr, nc))
        return neighbors

    def find_path(self, start_grid: Tuple[int, int], goal_grid: Tuple[int, int]) -> List[List[Tuple[int, int]]]:
        """
        使用 A* 算法寻找路径 (粗网格)。
        返回：List[List[Tuple[int, int]]] 结构，其中每个内部列表是原粗网格细分后的所有细网格坐标。
        """
        # A* 规划部分（粗网格）
        open_set = []
        heapq.heappush(open_set, (0, start_grid))
        g_score = { (r, c): float('inf') for r in range(1, self.rows + 1) for c in range(1, self.cols + 1) }
        g_score[start_grid] = 0
        came_from = {}
        
        orig_path = []
        
        while open_set:
            current_f, current_node = heapq.heappop(open_set)

            if current_node == goal_grid:
                # 找到目标，重建粗网格路径
                temp_node = current_node
                while temp_node != start_grid:
                    orig_path.append(temp_node)
                    temp_node = came_from[temp_node]
                orig_path.append(start_grid)
                orig_path = orig_path[::-1] # 从起点到终点

                # 路径细分和重组逻辑 (满足用户返回格式要求)
                subdivided_path: List[List[Tuple[int, int]]] = []
                
                for r_orig, c_orig in orig_path:
                    # 获取该粗网格包含的所有细网格坐标
                    fine_points = PointTF.subdivide_grid_to_fine_points(
                        r_orig, c_orig, SUBDIVISION_FACTOR
                    )
                    subdivided_path.append(fine_points)
                    
                return subdivided_path

            for neighbor in self.get_neighbors(current_node):
                tentative_g_score = g_score[current_node] + 1 
                if tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current_node
                    g_score[neighbor] = tentative_g_score
                    f_score = tentative_g_score + self.heuristic(neighbor, goal_grid)
                    heapq.heappush(open_set, (f_score, neighbor))
        return [] 


# ===================== A* 测试对象实例化 (修正) ===================== 
class AstarTest:
    """
    封装了 A* 路径规划、移动逻辑和路径渲染的类。
    """
    def __init__(self, robot, maze_data, maze_map, maze_config, scene_renderer):
        self.robot = robot
        self.maze_data = maze_data
        self.maze_config = maze_config
        self.scene_renderer = scene_renderer
        self.factor = SUBDIVISION_FACTOR
        
        # 规划器和路径初始化
        self.planner = AStarStaticPlanner(maze_map, maze_config)
        self.start_grid = PointTF.pixel_to_grid(*maze_data.start_pixel)
        self.goal_grid = PointTF.pixel_to_grid(*maze_data.goal_pixel)
        
        # self.sub_path 存储 A* 返回的 List[List[Tuple[int, int]]]
        self.sub_path: List[List[Tuple[int, int]]] = []
        # self.flat_path 存储用于机器人追踪的路径点（每个粗网格的中心细网格点）
        self.flat_path: List[Tuple[int, int]] = []
        self.is_planning_complete = False

    def start_planning_and_movement(self):
        """执行规划、路径细化并准备开始移动"""
        print(f"AstarTest: Planning path from {self.start_grid} to {self.goal_grid}...")
        
        current_grid = PointTF.pixel_to_grid(self.robot.x, self.robot.y)
        
        # 1. A* 算法找到并返回细分后的路径 (List[List[Tuple]])
        self.sub_path = self.planner.find_path(current_grid, self.goal_grid)
        
        if self.sub_path:
            self.is_planning_complete = True
            
            # 2. 将细分路径转换为扁平化路径（只追踪每个粗网格的中心细网格点）
            self.flat_path = []
            processed_orig_grids = set()
            
            for fine_points_in_cell in self.sub_path:
                # 获取该粗网格的任意一个细网格点，用于反推粗网格坐标
                r_sub_any, c_sub_any = fine_points_in_cell[0]
                
                # 反向转换找到粗网格坐标
                r_orig, c_orig = PointTF.subgrid_to_origrid(r_sub_any, c_sub_any, self.factor)
                
                if (r_orig, c_orig) not in processed_orig_grids:
                    # 找到该粗网格对应的中心细网格点作为航路点
                    center_subgrid = PointTF.origrid_to_subgrid_center(r_orig, c_orig, self.factor)
                    self.flat_path.append(center_subgrid)
                    processed_orig_grids.add((r_orig, c_orig))


            # 移除当前位置的中心细网格点（如果它在路径上）
            if self.flat_path:
                current_subgrid_center = PointTF.origrid_to_subgrid_center(*current_grid, self.factor)
                if self.flat_path[0] == current_subgrid_center:
                    self.flat_path.pop(0)

            print(f"AstarTest: Path found with {len(self.sub_path)} coarse steps.")
            print(f"AstarTest: Flat tracking path generated with {len(self.flat_path)} fine steps. Ready to move.")
        else:
            self.is_planning_complete = False
            print("AstarTest: Error: Could not find a static coarse path.")

    def update(self):
        """在主循环中调用此方法来执行路径跟踪和移动"""
        # 机器人跟踪的是 self.flat_path
        if not self.is_planning_complete or not self.flat_path:
            self.robot.speed = 0
            self.robot.goal = True
            self.robot.update()
            return

        # 1. 目标点检查和更新 (目标点是细网格坐标)
        next_subgrid = self.flat_path[0]
        target_x, target_y = PointTF.subgrid_to_pixel_center(*next_subgrid, self.maze_config, factor=self.factor)
        
        dist_to_target = math.hypot(self.robot.x - target_x, self.robot.y - target_y)
        
        if dist_to_target < self.robot.radius / (self.factor * 2):
            self.flat_path.pop(0)
            
            if not self.flat_path:
                self.robot.goal = True
                self.robot.speed = 0
                return
                
            # 更新下一个目标
            if self.flat_path:
                next_subgrid = self.flat_path[0]
                target_x, target_y = PointTF.subgrid_to_pixel_center(*next_subgrid, self.maze_config, factor=self.factor)
            else:
                return 

        # 2. 计算方向和移动
        dx = target_x - self.robot.x
        dy = target_y - self.robot.y
        norm = math.hypot(dx, dy)
        
        if norm > 0:
            self.robot.direction = (dx / norm, dy / norm)
        else:
            self.robot.direction = (0, 0)
            
        self.robot.update()

    def draw_path(self):
        """绘制细化后的路径"""
        if not self.sub_path:
            return
            
        # 1. 绘制粗网格点中心连线 (细化路径)
        path_pixels = [(self.robot.x, self.robot.y)] + \
                      [PointTF.subgrid_to_pixel_center(*grid, self.maze_config, factor=self.factor) 
                       for grid in self.flat_path]

        for i in range(len(path_pixels) - 1):
            p1 = path_pixels[i]
            p2 = path_pixels[i+1]
            # 蓝色线段连接每个粗网格的中心细网格点
            pygame.draw.line(self.scene_renderer.screen, (0, 191, 255), p1, p2, 2)
            
        # 2. 绘制细分后的所有细网格点 (橙色小圆点，用于可视化细分结构)
        # 这样您可以清楚地看到每个粗网格是如何被细分成 FACTOR*FACTOR 个细网格的。
        for fine_points_in_cell in self.sub_path:
            for r_sub, c_sub in fine_points_in_cell:
                x_center, y_center = PointTF.subgrid_to_pixel_center(r_sub, c_sub, self.maze_config, factor=self.factor)
                pygame.draw.circle(self.scene_renderer.screen, (255, 165, 0), 
                                   (int(x_center), int(y_center)), 2)
           
#============================== 结束 ===================================#


# === 配置动态障碍 ===
# 低速障碍物个数，速度范围，高速障碍物个数，速度范围, 半径大小
dynamic_obstacles = render_map.generate_dynamic_obstacles(
    num_slow=20, num_fast=20, slow_speed_range=(0.5,1),fast_speed_range=(1.5,3),
    map_width=mazeconfig.map_size[0], map_height=mazeconfig.map_size[1],radius=10,
    maze_walls=maze_data.walls
)

# 配置机器人
Nagisa_robot = render_map.N7carRobot(
    x=maze_data.start_pixel[0],
    y=maze_data.start_pixel[1],
    speed=2,
    direction=(1,0),
    radius=15
)

# 配置环境渲染器
scene_renderer = render_map.SceneRenderer(maze_data, mazeconfig, dynamic_obstacles, Nagisa_robot)


# -------------------- A* 测试对象实例化 --------------------
astar_test_module = AstarTest(Nagisa_robot, maze_data, maze, mazeconfig, scene_renderer)
astar_test_module.start_planning_and_movement()
# ------------------------------------------------------------


# -------------------- 主循环和渲染 --------------------
clock = pygame.time.Clock()
running = True
pygame.font.init()

while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
            
    for obs in dynamic_obstacles:
        obs.update(maze_data.walls)

    Nagisa_robot.update()

    # -------------A* 测试，地图绘制和机器人移动------------------
    astar_test_module.update()
    scene_renderer.draw()
    astar_test_module.draw_path() 

    # 显式绘制机器人，确保它位于最顶层，覆盖路径
    try:
        Nagisa_robot.draw() 
    except AttributeError:
        pass 

    pygame.display.flip()
    clock.tick(30) # 控制帧率

scene_renderer.quit()