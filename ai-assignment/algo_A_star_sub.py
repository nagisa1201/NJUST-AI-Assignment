'''
Author: Nagisa 2964793117@qq.com
Date: 2025-12-14 18:37:10
LastEditors: Nagisa 2964793117@qq.com
LastEditTime: 2025-12-14 22:56:44
FilePath: /NJUST-AI-Assignment/ai-assignment/algo_A_star_subgrid.py
Description: 使用细化网格（Sub-Grid）的 A* 算法，用于静态避障和为动态避障（如VO）提供平滑基础路径。
'''
'''
    使用A_star算法在细化网格（Sub-Grid）上进行静态避障
'''

import render_map
import math 
import pygame
import numpy as np
import heapq
from typing import Tuple, List, Dict

# ===================== 配置细化因子 =====================
# 每个原始粗网格会被等分成 SUBDIVISION_FACTOR x SUBDIVISION_FACTOR 个细网格
SUBDIVISION_FACTOR = 3 
# ======================================================

# 首先配置迷宫地图
mazeconfig = render_map.MazeMapConfig(rows=15, cols=15, cell_size=70, loop_percent=10, start_point=(1,1), goal_point=(15,15))
# 假设 render_map.create_scene 返回 maze, maze_data, renderer
maze, maze_data, renderer = render_map.MazeMapConfig.create_scene(mazeconfig) 
# 

# ===================== 坐标转换工具类 =====================
class PointTF:
    """封装了像素坐标、粗网格坐标和细网格坐标之间转换的静态方法"""
    
    @staticmethod
    def pixel_to_grid(x: float, y: float, mazeconfig, factor: int = 1) -> Tuple[int, int]:
        """
        将像素坐标 (x, y) 转换为 1-based 网格 (row, col)。
        factor=1 是粗网格，factor=SUBDIVISION_FACTOR 是细网格。
        """
        cell_size = mazeconfig.cell_size / factor
        # X 对应 Col, Y 对应 Row
        col = int(x / cell_size) + 1
        row = int(y / cell_size) + 1
        
        max_rows = mazeconfig.rows * factor
        max_cols = mazeconfig.cols * factor
        
        # 确保在地图范围内
        return max(1, min(max_rows, row)), max(1, min(max_cols, col))

    @staticmethod
    def grid_to_pixel_center(r: int, c: int, mazeconfig, factor: int = 1) -> Tuple[float, float]:
        """
        将 1-based 网格 (row, col) 转换为像素中心点 (x, y)。
        factor=1 是粗网格，factor=SUBDIVISION_FACTOR 是细网格。
        """
        cell_size = mazeconfig.cell_size / factor
        x = (c - 0.5) * cell_size
        y = (r - 0.5) * cell_size
        return x, y
    
    @staticmethod
    def subgrid_to_origrid(r_sub: int, c_sub: int, factor: int) -> Tuple[int, int]:
        """将细网格坐标 (r_sub, c_sub) 转换为原始粗网格坐标 (r_orig, c_orig)"""
        r_orig = (r_sub - 1) // factor + 1
        c_orig = (c_sub - 1) // factor + 1
        return r_orig, c_orig

    @staticmethod
    def origrid_to_subgrid_center(r_orig: int, c_orig: int, factor: int) -> Tuple[int, int]:
        """将原始粗网格 (r_orig, c_orig) 的中心点（细网格坐标）"""
        center_offset = (factor + 1) // 2 
        r_sub = (r_orig - 1) * factor + center_offset
        c_sub = (c_orig - 1) * factor + center_offset
        return r_sub, c_sub
    
# ===================== A* 算法的实现 =====================
class AStarInit:
    def __init__(self, maze_data, mazeconfig):
        start_grid = PointTF.pixel_to_grid(*maze_data.start_pixel, mazeconfig)
        goal_grid = PointTF.pixel_to_grid(*maze_data.goal_pixel, mazeconfig)
        self.start_grid = start_grid
        self.goal_grid = goal_grid

class AStarStaticPlanner: # 仿照旧代码的类名，但内部实现是细网格
    def __init__(self, maze_map, maze_config, factor: int = SUBDIVISION_FACTOR):
        # 原始地图数据
        self.original_maze_map: Dict[Tuple[int, int], Dict[str, int]] = maze_map.maze_map
        self.orig_rows = maze_config.rows
        self.orig_cols = maze_config.cols
        self.factor = factor
        
        # 细网格尺寸
        self.sub_rows = self.orig_rows * factor
        self.sub_cols = self.orig_cols * factor

    def heuristic(self, a: Tuple[int, int], b: Tuple[int, int]) -> int:
        """启发函数：曼哈顿距离 (Manhattan Distance)，基于细网格坐标"""
        r1, c1 = a
        r2, c2 = b
        return abs(r1 - r2) + abs(c1 - c2)

    def get_neighbors(self, current: Tuple[int, int]) -> List[Tuple[int, int]]:
        """获取当前细网格单元格 (r_sub, c_sub) 的可通行邻居"""
        r_sub, c_sub = current
        neighbors = []
        # 仅考虑上下左右移动，每个移动成本为 1
        moves = [(-1, 0), (1, 0), (0, 1), (0, -1)]

        for dr_sub, dc_sub in moves:
            nr_sub, nc_sub = r_sub + dr_sub, c_sub + dc_sub
            
            # 检查是否越界
            if not (1 <= nr_sub <= self.sub_rows and 1 <= nc_sub <= self.sub_cols):
                continue
            
            r_orig, c_orig = PointTF.subgrid_to_origrid(r_sub, c_sub, self.factor)
            nr_orig, nc_orig = PointTF.subgrid_to_origrid(nr_sub, nc_sub, self.factor)
            
            is_passable = True
            
            # 只有在跨越粗网格边界时才需要检查墙壁
            if (r_orig, c_orig) != (nr_orig, nc_orig):
                current_cell_data = self.original_maze_map.get((r_orig, c_orig))
                
                if current_cell_data:
                    dr_orig, dc_orig = nr_orig - r_orig, nc_orig - c_orig
                    direction_map = {(-1, 0): 'N', (1, 0): 'S', (0, 1): 'E', (0, -1): 'W'}
                    direction = direction_map.get((dr_orig, dc_orig))
                    
                    # 检查当前粗网格是否有墙壁阻挡向目标粗网格的移动
                    if direction and current_cell_data.get(direction) != 1:
                        is_passable = False
                else:
                    is_passable = False
            
            if is_passable:
                neighbors.append((nr_sub, nc_sub))
                
        return neighbors

    def find_path(self, start_grid: Tuple[int, int], goal_grid: Tuple[int, int]) -> List[Tuple[int, int]]:
        """
        使用 A* 算法在细网格上寻找路径。
        返回的路径就是细化后的路径（细网格坐标序列）。
        
        :param start_grid: 起始细网格坐标 (r_sub, c_sub)
        :param goal_grid: 目标细网格坐标 (r_sub, c_sub)
        :return: 包含 (r_sub, c_sub) 路径点的列表 (从起点到终点)
        """
        open_set = []
        # (f_score, node)
        heapq.heappush(open_set, (0, start_grid))

        # g_score 字典存储从起点到当前细网格节点的实际代价
        g_score = { (r, c): float('inf') for r in range(1, self.sub_rows + 1) 
                                         for c in range(1, self.sub_cols + 1) }
        
        g_score[start_grid] = 0
        # came_from 字典用于路径重建，存储每个细网格节点的前驱节点
        came_from = {}
        
        while open_set:
            current_f, current_node = heapq.heappop(open_set)

            if current_node == goal_grid:
                # 路径重建
                path = []
                temp_node = current_node
                while temp_node != start_grid:
                    path.append(temp_node)
                    temp_node = came_from[temp_node]
                path.append(start_grid)
                return path[::-1] # 反转路径，从起点到终点
                

            for neighbor in self.get_neighbors(current_node):
                # 两个相邻细单元格之间的移动成本为 1 
                tentative_g_score = g_score[current_node] + 1
                
                # 检查是否找到更优路径
                if tentative_g_score < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current_node
                    g_score[neighbor] = tentative_g_score
                    # f_score = g + h
                    f_score = tentative_g_score + self.heuristic(neighbor, goal_grid)
                    heapq.heappush(open_set, (f_score, neighbor))
                    
        return [] # 未找到路径


class AstarTest: 
    """
    封装了细网格 A* 路径规划、移动逻辑和路径渲染的类。
    """
    def __init__(self, robot, maze_data, maze_map, maze_config, scene_renderer):
        self.robot = robot
        self.maze_data = maze_data
        self.maze_config = maze_config
        self.scene_renderer = scene_renderer
        self.factor = SUBDIVISION_FACTOR
        
        # 将原始起点/终点转换为细网格坐标
        goal_orig_grid = PointTF.pixel_to_grid(*maze_data.goal_pixel, maze_config, factor=1)
        
        # 细网格的终点应该设置为原始目标格子的中心点
        self.goal_subgrid = PointTF.origrid_to_subgrid_center(*goal_orig_grid, self.factor)

        # 规划器和路径初始化 (注意：传入了 factor)
        self.planner = AStarStaticPlanner(maze_map, maze_config, self.factor)
        self.sub_path: List[Tuple[int, int]] = []
        self.is_planning_complete = False

        self.passed_orig_grids = set() 

    def start_planning_and_movement(self):
        """执行规划并准备开始移动"""
        
        # 实时获取机器人的当前细网格位置作为起点
        current_subgrid = PointTF.pixel_to_grid(self.robot.x, self.robot.y, self.maze_config, factor=self.factor)
        
        print(f"AstarTest: Planning path from subgrid {current_subgrid} to {self.goal_subgrid}...")
        
        # *** 此处调用 find_path，返回的就是细化后的路径 ***
        self.sub_path = self.planner.find_path(current_subgrid, self.goal_subgrid)
        
        if self.sub_path:
            # 移除路径中的起点，因为机器人已经位于该点
            if self.sub_path and self.sub_path[0] == current_subgrid:
                 self.sub_path.pop(0) 
            self.is_planning_complete = True
            print(f"AstarTest: Path found with {len(self.sub_path)} steps. Ready to move.")
            
            # 初始化通过的粗网格
            current_orig_grid = PointTF.subgrid_to_origrid(current_subgrid[0], current_subgrid[1], self.factor)
            self.passed_orig_grids.add(current_orig_grid)
        else:
            self.is_planning_complete = False
            print("AstarTest: Error: Could not find a static path.")

    def update(self):
        """在主循环中调用此方法来执行路径跟踪和移动"""
        if not self.is_planning_complete or not self.sub_path:
            self.robot.speed = 0
            self.robot.goal = True
            self.robot.update()
            return

        # 1. 目标点检查和更新
        next_subgrid = self.sub_path[0]
        # 使用细网格的坐标到像素中心转换
        target_x, target_y = PointTF.grid_to_pixel_center(*next_subgrid, self.maze_config, factor=self.factor)
        
        # 使用更小的判定距离 (相对机器人半径和细化因子)
        dist_to_target = math.hypot(self.robot.x - target_x, self.robot.y - target_y)
        
        # 机器人接近目标点时切换下一个目标点
        if dist_to_target < self.robot.radius / (self.factor * 2): 
            self.sub_path.pop(0)
            
            # 路径通过判定
            if next_subgrid:
                r_sub, c_sub = next_subgrid
                r_orig, c_orig = PointTF.subgrid_to_origrid(r_sub, c_sub, self.factor)
                self.passed_orig_grids.add((r_orig, c_orig))
            
            if not self.sub_path:
                self.robot.goal = True
                self.robot.speed = 0
                return
                
            # 更新下一个目标
            if self.sub_path:
                next_subgrid = self.sub_path[0]
                target_x, target_y = PointTF.grid_to_pixel_center(*next_subgrid, self.maze_config, factor=self.factor)
            else:
                return # 路径走完

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
        """绘制细网格路径"""
        if not self.sub_path:
            return
            
        # 路径点的像素坐标
        path_pixels = [(self.robot.x, self.robot.y)] + \
                      [PointTF.grid_to_pixel_center(*grid, self.maze_config, factor=self.factor) 
                       for grid in self.sub_path]

        # 绘制细路径
        for i in range(len(path_pixels) - 1):
            p1 = path_pixels[i]
            p2 = path_pixels[i+1]
            # 路径颜色改为细网格的蓝色 (0, 191, 255)
            pygame.draw.line(self.scene_renderer.screen, (0, 191, 255), p1, p2, 2)
            
        # -----------------------------------------------------------------
        # 绘制已通过的粗网格中心 (红色圆点，用于调试) - 已注释掉
        # -----------------------------------------------------------------
        # for r_orig, c_orig in self.passed_orig_grids:
        #    x_center, y_center = PointTF.grid_to_pixel_center(r_orig, c_orig, self.maze_config, factor=1)
        #    pygame.draw.circle(self.scene_renderer.screen, (255, 0, 0), 
        #                       (int(x_center), int(y_center)), 3) 

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

# -------------------- A* 测试对象实例化 --------------------
astar_test_module = AstarTest(Nagisa_robot, maze_data, maze, mazeconfig, renderer)
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
            
    # 1. 更新逻辑
    for obs in dynamic_obstacles:
        obs.update(maze_data.walls)

    Nagisa_robot.update()
    astar_test_module.update()
    
    # 2. 绘制逻辑 (关键：保证机器人位于最上层)
    
    # 绘制地图、墙壁和动态障碍物
    renderer.draw() 
    
    # 在地图之上绘制路径（蓝线）
    astar_test_module.draw_path() 

    # 显式绘制机器人，确保它位于最顶层，覆盖路径
    # 假设 N7carRobot 有一个 draw(screen) 方法，此处传入 screen 对象
    try:
        # 修复：传递 renderer.screen 作为绘图目标
        Nagisa_robot.draw(renderer.screen) 
    except AttributeError:
        # 如果机器人对象没有 draw 方法或者绘制逻辑在 renderer.draw() 内部，
        # 则此处可能报错或不需要。但手动绘制更保险。
        pass 

    pygame.display.flip()
    clock.tick(30) # 控制帧率

renderer.quit()