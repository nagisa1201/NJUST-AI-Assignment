'''
Author: Nagisa 2964793117@qq.com
Date: 2025-12-13 18:37:10
LastEditors: Nagisa 2964793117@qq.com
LastEditTime: 2025-12-15 11:58:13
FilePath: /NJUST-AI-Assignment/ai-assignment/algo_D_star_lite.py
Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koroFileHeader/wiki/%E9%5D%A3%E7%BD%AE
'''
'''
    使用A_star算法进行静态避障的基础算法
'''

import render_map
import math 
import pygame
import numpy as np
import heapq
from typing import Tuple, List, Optional

# 首先配置迷宫地图
mazeconfig = render_map.MazeMapConfig(rows=12, cols=12, cell_size=90, loop_percent=80, start_point=(1,1), goal_point=(8,8))
maze, maze_data, renderer = render_map.MazeMapConfig.create_scene(mazeconfig)




# ===================== 坐标转换工具类 =====================
class PointTF:
    """封装了像素坐标和网格坐标之间转换的静态方法"""
    @staticmethod
    def pixel_to_grid(x: float, y: float, cell_size: int = mazeconfig.cell_size) -> Tuple[int, int]:
        """将像素坐标 (x, y) 转换为 1-based 网格 (row, col)"""
        # X 对应 Col, Y 对应 Row
        col = int(x / cell_size) + 1
        row = int(y / cell_size) + 1
        # 确保在地图范围内
        return max(1, min(mazeconfig.rows, row)), max(1, min(mazeconfig.cols, col))

    @staticmethod
    def grid_to_pixel_center(r: int, c: int, cell_size: int = mazeconfig.cell_size) -> Tuple[float, float]:
        """将 1-based 网格 (row, col) 转换为像素中心点 (x, y)"""
        x = (c - 0.5) * cell_size
        y = (r - 0.5) * cell_size
        return x, y
    
# ===================== A* 算法的实现 =====================
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
        self.maze_config = maze_config # 存储配置，用于初始化细化器
            
        # 实例化 PathRefiner,在此调整细化参数
        self.path_refiner = PathRefiner(
                subdivision_factor=8, 
                cell_size=self.maze_config.cell_size
        )
        
    def heuristic(self, a: Tuple[int, int], b: Tuple[int, int]) -> int:
        """启发函数：曼哈顿距离 (Manhattan Distance)"""
        r1, c1 = a
        r2, c2 = b
        # 适用于网格路径的优秀启发函数
        return abs(r1 - r2) + abs(c1 - c2)

    def get_neighbors(self, current: Tuple[int, int]) -> List[Tuple[int, int]]:
        """获取当前单元格 (r, c) 的可通行邻居"""
        r, c = current
        neighbors = []
        # 定义可能的移动方向：(dr, dc) 和 pyamaze 的方向标识
        moves = {
            'N': (-1, 0), 
            'S': (1, 0), 
            'E': (0, 1), 
            'W': (0, -1)
        }
        # 检查当前单元格在 pyamaze 中的连接信息
        current_cell_data = self.maze_map.get(current)
        if not current_cell_data:
            return []

        for direction, (dr, dc) in moves.items():
            # pyamaze 中，值为 1 表示可通行
            if current_cell_data.get(direction) == 1:
                nr, nc = r + dr, c + dc
                # 检查新位置是否在地图范围内
                if 1 <= nr <= self.rows and 1 <= nc <= self.cols:
                    neighbors.append((nr, nc))       
        return neighbors

    def find_path(self, start_grid: Tuple[int, int], goal_grid: Tuple[int, int]) -> List[Tuple[int, int]]:
        """
        使用 A* 算法寻找路径
        :param start_grid: 起始网格坐标 (r, c)
        :param goal_grid: 目标网格坐标 (r, c)
        :return: 包含 (r, c) 路径点的列表 (从起点到终点)
        """
        # 优先队列: (f_score, node)
        open_set = []
        heapq.heappush(open_set, (0, start_grid))

        # g_score: 从起点到某点的实际成本
        g_score = {}
        # 初始化所有单元格的 g 值为无穷大，避免字典查找错误
        for r in range(1, self.rows + 1):
            for c in range(1, self.cols + 1):
                g_score[(r, c)] = float('inf')
        
        g_score[start_grid] = 0
        # came_from: 记录最短路径回溯
        came_from = {}
        
        while open_set:
            # 弹出 f_score 最小的节点
            current_f, current_node = heapq.heappop(open_set)

            if current_node == goal_grid:
                # 找到目标，重建路径
                path = []
                temp_node = current_node
                while temp_node != start_grid:
                    path.append(temp_node)
                    temp_node = came_from[temp_node]
                path.append(start_grid)
                return path[::-1] # 返回反转后的路径 (从起点到终点)

            for neighbor in self.get_neighbors(current_node):
                # 两个相邻单元格之间的移动成本为 1 (静态网格)
                tentative_g_score = g_score[current_node] + 1 
                if tentative_g_score < g_score[neighbor]:
                    # 发现了一条更好的路径
                    came_from[neighbor] = current_node
                    g_score[neighbor] = tentative_g_score
                    f_score = tentative_g_score + self.heuristic(neighbor, goal_grid)
                    # 将邻居节点推入优先队列
                    heapq.heappush(open_set, (f_score, neighbor))
        return [] # 未找到路径
    

    def find_and_refine_path(self, start_grid: Tuple[int, int], goal_grid: Tuple[int, int]) -> List[List[Tuple[float, float]]]:
        """
        首先使用 A* 寻找粗粒度路径，然后将其细化为元组列表的列表。
        
        :param start_grid: 起始网格坐标 (r, c)
        :param goal_grid: 目标网格坐标 (r, c)
        :return: 细化后的路径段列表: List[List[Tuple[float, float]]]
        """
        # 调用原始的 A* 寻路方法获取粗粒度路径
        coarse_path = self.find_path(start_grid, goal_grid)
        
        if not coarse_path:
            return []

        # 调用内部的 PathRefiner 进行细化
        fine_path_segments = self.path_refiner.subdivide_path(coarse_path)
        
        return fine_path_segments
    
# ===================== 路径细化工具类 (仅用于数据转换/渲染) =====================
class PathRefiner:
    """
    负责将粗粒度的网格路径 (r, c) 转换为细粒度的像素路径 (x, y)，
    并将路径中的每个粗网格等分为 N*N 个子网格的中心点。
    这个类仅用于数据转换，不影响 A* 寻路和机器人移动逻辑。
    """
    def __init__(self, subdivision_factor: int = 3, cell_size: int = mazeconfig.cell_size):
        # 每条边细分的份数 (例如 3x3 子网格)
        self.subdivision_factor = subdivision_factor
        # 原始网格大小
        self.original_cell_size = cell_size
        # 细分后每个子网格的大小
        self.sub_cell_size = self.original_cell_size / self.subdivision_factor

    def subdivide_path(self, coarse_path: List[Tuple[int, int]]) -> List[List[Tuple[float, float]]]:
        """
        将粗粒度的网格路径 (r, c) 转换为细粒度的像素路径 (x, y) 段列表 (元组列表的列表)。

        :param coarse_path: A* 规划出的网格路径点列表 (r, c)。
        :return: 路径段列表: List[List[Tuple[float, float]]]
        """
        if not coarse_path:
            return []

        fine_segments: List[List[Tuple[float, float]]] = []

        # 遍历 A* 路径中的每一个网格单元
        for r_grid, c_grid in coarse_path:
            # 找到该网格单元的左上角像素坐标 (0-based)
            x_start = (c_grid - 1) * self.original_cell_size
            y_start = (r_grid - 1) * self.original_cell_size
            
            segment: List[Tuple[float, float]] = []

            # 在该网格内部生成 N*N 个子网格的中心点
            for r_sub in range(self.subdivision_factor):
                for c_sub in range(self.subdivision_factor):
                    # 偏移 = (序号 + 0.5) * 子网格大小
                    dx = (c_sub + 0.5) * self.sub_cell_size
                    dy = (r_sub + 0.5) * self.sub_cell_size
                    
                    # 子网格中心点的全局像素坐标
                    x_fine = x_start + dx
                    y_fine = y_start + dy
                    
                    segment.append((x_fine, y_fine))
            
            # 将该网格的所有细分点作为一个子列表添加到结果中
            fine_segments.append(segment)
        return fine_segments


# ===================== A* 测试演示类 (Test Class) =====================
class AstarTest:
    """
    封装了 A* 路径规划、移动逻辑和路径渲染的类。
    机器人的移动严格按照 A* 粗粒度网格中心点进行。
    """
    def __init__(self, robot, maze_data, maze_map, maze_config, scene_renderer):
        self.robot = robot
        self.maze_data = maze_data
        self.scene_renderer = scene_renderer
        self.maze_config = maze_config
        
        # 规划器和路径初始化
        self.planner = AStarStaticPlanner(maze_map, maze_config)
        
        self.start_grid = PointTF.pixel_to_grid(*maze_data.start_pixel)
        self.goal_grid = PointTF.pixel_to_grid(*maze_data.goal_pixel)
        
        # 粗粒度路径 (网格坐标) - 机器人实际跟踪的路径
        self.path: List[Tuple[int, int]] = []
        # 细粒度路径段 (仅用于渲染和数据输出)
        self.fine_path_segments: List[List[Tuple[float, float]]] = [] 
        self.is_planning_complete = False

    def start_planning_and_movement(self):
        """执行规划、生成细化数据并准备开始移动"""
        print(f"AstarTest: Planning path from {self.start_grid} to {self.goal_grid}...")
        
        current_grid = PointTF.pixel_to_grid(self.robot.x, self.robot.y)
        
        
        # 1. 调用新的方法获取细化路径 (元组列表的列表)
        self.fine_path_segments = self.planner.find_and_refine_path(current_grid, self.goal_grid)


        if self.fine_path_segments:
                    print("\n--- 细化后的路径数据 (fine_path_segments) ---")
                    # 打印前两个网格的细化点作为示例，避免输出过多数据
                    print(f"网格1的细分点 (共 {len(self.fine_path_segments[0])} 个): {self.fine_path_segments[0]}")
                    if len(self.fine_path_segments) > 1:
                        print(f"网格2的细分点 (共 {len(self.fine_path_segments[1])} 个): {self.fine_path_segments[1]}")
                    if len(self.fine_path_segments) > 2:
                        print("...")
                    print(f"总共有 {len(self.fine_path_segments)} 个粗粒度网格的细分数据。")
                    print("---------------------------------------------------\n")
        
        # 2. 重新调用原始 find_path 方法获取粗粒度路径用于移动
        # 注意：这里需要重新调用 planner.find_path，因为 find_and_refine_path 只返回细化路径。
        self.path = self.planner.find_path(current_grid, self.goal_grid)
        
        if self.path:
            self.path.pop(0) # 移除当前位置 (保持原始逻辑)
            self.is_planning_complete = True
            print(f"AstarTest: Path found with {len(self.path)} coarse steps. Fine segments generated ({sum(len(s) for s in self.fine_path_segments)} points).")
        else:
            self.is_planning_complete = False
            print("AstarTest: Error: Could not find a static path.")

    def update(self):
        """在主循环中调用此方法来执行路径跟踪和移动 (使用粗粒度路径)"""
        # --- 原始 update 逻辑：完全恢复为跟踪粗粒度网格中心点 ---
        if not self.is_planning_complete or not self.path:
            self.robot.speed = 0
            self.robot.goal = True
            self.robot.update()
            return

        # 1. 目标点检查和更新
        next_grid = self.path[0]
        target_x, target_y = PointTF.grid_to_pixel_center(*next_grid)
        dist_to_target = math.hypot(self.robot.x - target_x, self.robot.y - target_y)
        
        if dist_to_target < self.robot.radius / 2: # 保持原始的到达阈值
            self.path.pop(0)
            if not self.path:
                self.robot.goal = True
                self.robot.speed = 0
                return
            # 更新下一个目标
            next_grid = self.path[0]
            target_x, target_y = PointTF.grid_to_pixel_center(*next_grid)
            
        # 2. 计算方向和移动
        dx = target_x - self.robot.x
        dy = target_y - self.robot.y
        norm = math.hypot(dx, dy)
        
        if norm > 0:
            self.robot.direction = (dx / norm, dy / norm)
        else:
            self.robot.direction = (0, 0)
            
        self.robot.update()
        # -----------------------------------------------------------------

    def draw_path(self):
        """绘制路径，同时描绘细分后的所有点"""
        
        # 1. 绘制原始粗粒度路径 (用于连接网格中心点) - 保持原始绘制方式
        if self.path:
            # Note: 这里的 self.path 已经 pop 掉了第一个点，所以需要加上机器人当前位置
            path_pixels = [(self.robot.x, self.robot.y)] + [PointTF.grid_to_pixel_center(*grid) for grid in self.path]

            for i in range(len(path_pixels) - 1):
                p1 = path_pixels[i]
                p2 = path_pixels[i+1]
                # 原始粗路径用粗线连接
                pygame.draw.line(self.scene_renderer.screen, (255, 165, 0), p1, p2, 5)

        # 2. 描绘细分后的所有点 (仅用于展示细化数据)
        if self.fine_path_segments:
            # 将 segments 展平为单个列表
            all_fine_points = [point for segment in self.fine_path_segments for point in segment]
            
            for x, y in all_fine_points:
                 # 突出显示细化后的点
                 pygame.draw.circle(self.scene_renderer.screen, (0, 255, 255), (int(x), int(y)), 2)
                 
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
    
    # 动态障碍物更新
    for obs in dynamic_obstacles:
        obs.update(maze_data.walls)

    # Nagisa_robot.update() 已经在 astar_test_module.update() 中调用

    # -----------------A* 测试，地图绘制和机器人移动------------------
    astar_test_module.update()
    scene_renderer.draw()
    astar_test_module.draw_path() # 绘制路径和细分点


    pygame.display.flip()
    clock.tick(30) # 控制帧率

scene_renderer.quit()