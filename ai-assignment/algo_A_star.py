'''
Author: Nagisa 2964793117@qq.com
Date: 2025-12-13 18:37:10
LastEditors: Nagisa 2964793117@qq.com
LastEditTime: 2025-12-13 19:46:42
FilePath: /NJUST-AI-Assignment/ai-assignment/algo_D_star_lite.py
Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
'''
'''
    使用A_star算法进行静态避障的基础算法
'''

import render_map
import math 
import pygame
import numpy as np
import heapq
from typing import Tuple, List

# 首先配置迷宫地图
# mazeconfig = render_map.MazeMapConfig(rows=15, cols=15, cell_size=70, loop_percent=10, start_point=(1,1), goal_point=(15,15))
# maze, maze_data, renderer = render_map.MazeMapConfig.create_scene(mazeconfig)




# ===================== 坐标转换工具类 =====================
class PointTF:
    """封装了像素坐标和网格坐标之间转换的静态方法"""
    @staticmethod
    def pixel_to_grid(x: float, y: float, mazeconfig) -> Tuple[int, int]:
        """将像素坐标 (x, y) 转换为 1-based 网格 (row, col)"""
        # X 对应 Col, Y 对应 Row
        col = int(x / mazeconfig.cell_size) + 1
        row = int(y / mazeconfig.cell_size) + 1
        # 确保在地图范围内
        return max(1, min(mazeconfig.rows, row)), max(1, min(mazeconfig.cols, col))

    @staticmethod
    def grid_to_pixel_center(r: int, c: int, mazeconfig) -> Tuple[float, float]:
        """将 1-based 网格 (row, col) 转换为像素中心点 (x, y)"""
        x = (c - 0.5) * mazeconfig.cell_size
        y = (r - 0.5) * mazeconfig.cell_size
        return x, y
    
# ===================== A* 算法的实现（静态避障） =====================
class AStarInit:
    def __init__(self, maze_data, mazeconfig):
        start_grid = PointTF.pixel_to_grid(*maze_data.start_pixel, mazeconfig)
        goal_grid = PointTF.pixel_to_grid(*maze_data.goal_pixel, mazeconfig)
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
                print(path)
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
    
class AstarTest:
    """
    封装了 A* 路径规划、移动逻辑和路径渲染的类。
    可以在需要时调用其方法来启动和控制机器人移动。
    """
    def __init__(self, robot, maze_data, maze_map, maze_config, scene_renderer):
        self.robot = robot
        self.maze_data = maze_data
        self.scene_renderer = scene_renderer
        
        # 规划器和路径初始化
        self.planner = AStarStaticPlanner(maze_map, maze_config)
        self.start_grid = PointTF.pixel_to_grid(*maze_data.start_pixel)
        self.goal_grid = PointTF.pixel_to_grid(*maze_data.goal_pixel)
        self.path: List[Tuple[int, int]] = []
        self.is_planning_complete = False

    def start_planning_and_movement(self):
        """执行规划并准备开始移动"""
        print(f"AstarTest: Planning path from {self.start_grid} to {self.goal_grid}...")
        
        current_grid = PointTF.pixel_to_grid(self.robot.x, self.robot.y)
        self.path = self.planner.find_path(current_grid, self.goal_grid)
        
        if self.path:
            self.path.pop(0) # 移除当前位置
            self.is_planning_complete = True
            print(f"AstarTest: Path found with {len(self.path)} steps. Ready to move.")
        else:
            self.is_planning_complete = False
            print("AstarTest: Error: Could not find a static path.")

    def update(self):
        """在主循环中调用此方法来执行路径跟踪和移动"""
        if not self.is_planning_complete or not self.path:
            self.robot.speed = 0
            self.robot.goal = True
            self.robot.update()
            return

        # 1. 目标点检查和更新
        next_grid = self.path[0]
        target_x, target_y = PointTF.grid_to_pixel_center(*next_grid)
        dist_to_target = math.hypot(self.robot.x - target_x, self.robot.y - target_y)
        
        if dist_to_target < self.robot.radius / 2:
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

    def draw_path(self):
        """绘制路径"""
        if not self.path:
            return
            
        path_pixels = [(self.robot.x, self.robot.y)] + [PointTF.grid_to_pixel_center(*grid) for grid in self.path]

        for i in range(len(path_pixels) - 1):
            p1 = path_pixels[i]
            p2 = path_pixels[i+1]
            pygame.draw.line(self.scene_renderer.screen, (255, 165, 0), p1, p2, 5)
#============================== 结束 ===================================#


# # -------------------- A* 算法初始化 --------------------
# astar_init = AStarInit(maze_data)
# planner = AStarStaticPlanner(maze, mazeconfig)
# path = planner.find_path(astar_init.start_grid, astar_init.goal_grid)
# # ------------------------------------------------------

# # -------------------- A* 测试对象实例化 --------------------
# astar_test_module = AstarTest(Nagisa_robot, maze_data, maze, mazeconfig, scene_renderer)
# astar_test_module.start_planning_and_movement()
# # ------------------------------------------------------------


# # -------------------- 主循环和渲染 --------------------
# clock = pygame.time.Clock()
# running = True
# pygame.font.init()

# while running:
#     for event in pygame.event.get():
#         if event.type == pygame.QUIT:
#             running = False
#     for obs in dynamic_obstacles:
#         obs.update(maze_data.walls)

#     Nagisa_robot.update() # 保持此调用以处理方向和位置的内部逻辑（虽然这里机器人默认静止）

#     # -------------A* 测试，地图绘制和机器人移动------------------
#     astar_test_module.update()
#     scene_renderer.draw()
#     astar_test_module.draw_path() # 绘制路径，可注释掉以查看无路径效果


#     pygame.display.flip()
#     clock.tick(30) # 控制帧率

# scene_renderer.quit()