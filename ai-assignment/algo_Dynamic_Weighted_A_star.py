'''
Author: Nagisa 2964793117@qq.com
Description: DWA* 性能分析
'''

import heapq
import math
import matplotlib.pyplot as plt
from typing import Tuple, List, Dict
import pygame
import render_map
import numpy as np

# ===================== 坐标转换工具类 =====================
class PointTF:
    @staticmethod
    def pixel_to_grid(x: float, y: float, mazeconfig) -> Tuple[int, int]:
        col = int(x / mazeconfig.cell_size) + 1
        row = int(y / mazeconfig.cell_size) + 1
        return max(1, min(mazeconfig.rows, row)), max(1, min(mazeconfig.cols, col))

    @staticmethod
    def grid_to_pixel_center(r: int, c: int, mazeconfig) -> Tuple[float, float]:
        x = (c - 0.5) * mazeconfig.cell_size
        y = (r - 0.5) * mazeconfig.cell_size
        return x, y

# ===================== Dynamic Weighted A* 算法 =====================
class AStarStaticPlanner:
    def __init__(self, maze_map, maze_config, weight: float = 1.0):
        self.maze_map_data = maze_map.maze_map if hasattr(maze_map, 'maze_map') else maze_map
        self.rows = maze_config.rows
        self.cols = maze_config.cols
        self.maze_config = maze_config 
        self.weight = weight  
        self.last_stats = {"nodes_expanded": 0, "path_length": 0}
        self.weight_history = [] 

    def heuristic(self, a: Tuple[int, int], b: Tuple[int, int]) -> int:
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def get_neighbors(self, current: Tuple[int, int]) -> List[Tuple[int, int]]:
        r, c = current
        neighbors = []
        moves = {'N': (-1, 0), 'S': (1, 0), 'E': (0, 1), 'W': (0, -1)}
        current_cell_data = self.maze_map_data.get(current)
        if not current_cell_data: return []
        for direction, (dr, dc) in moves.items():
            if current_cell_data.get(direction) == 1:
                nr, nc = r + dr, c + dc
                if 1 <= nr <= self.rows and 1 <= nc <= self.cols:
                    neighbors.append((nr, nc))       
        return neighbors

    def find_path(self, start_grid: Tuple[int, int], goal_grid: Tuple[int, int], record: bool = False) -> List[Tuple[int, int]]:
        open_set = []
        h_initial = self.heuristic(start_grid, goal_grid)
        heapq.heappush(open_set, (0, start_grid))
        g_score = {start_grid: 0}
        came_from = {}
        nodes_expanded = 0
        if record: self.weight_history = []
        
        while open_set:
            current_f, current_node = heapq.heappop(open_set)
            nodes_expanded += 1
            h_n = self.heuristic(current_node, goal_grid)
            epsilon = self.weight - 1.0
            dynamic_w = 1.0 + epsilon * (h_n / h_initial) if h_initial > 0 else 1.0
            if record: self.weight_history.append(dynamic_w)

            if current_node == goal_grid:
                path = []
                temp = current_node
                while temp in came_from:
                    path.append(temp); temp = came_from[temp]
                path.append(start_grid)
                self.last_stats = {"nodes_expanded": nodes_expanded, "path_length": len(path)}
                return path[::-1]

            for neighbor in self.get_neighbors(current_node):
                tentative_g = g_score[current_node] + 1
                if tentative_g < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current_node
                    g_score[neighbor] = tentative_g
                    h_neighbor = self.heuristic(neighbor, goal_grid)
                    w_neighbor = 1.0 + epsilon * (h_neighbor / h_initial) if h_initial > 0 else 1.0
                    f = tentative_g + w_neighbor * h_neighbor
                    heapq.heappush(open_set, (f, neighbor))
        self.last_stats = {"nodes_expanded": nodes_expanded, "path_length": 0}
        return []

# ===================== 测试与可视化类 =====================
class AstarTest:
    def __init__(self, robot, maze_data, maze_map, maze_config, scene_renderer):
        self.robot = robot
        self.maze_data = maze_data
        self.scene_renderer = scene_renderer
        self.maze_config = maze_config
        self.planner = AStarStaticPlanner(maze_map, maze_config)
        self.start_grid = PointTF.pixel_to_grid(robot.x, robot.y, maze_config)
        self.goal_grid = PointTF.pixel_to_grid(*maze_data.goal_pixel, maze_config)
        self.path: List[Tuple[int, int]] = []
        self.is_planning_complete = False

    def _draw_dual_axis_core(self, ax1, x_vals, nodes, lengths, title):
        """  Figure 1 w的参数扫描 """
        ax1.set_xlabel('Initial Weight ($w_{start}$)', fontsize=9)
        ax1.set_ylabel('Nodes', color='#d62728', fontsize=9)
        ax1.plot(x_vals, nodes, color='#d62728', marker='o', linewidth=1.5, markersize=4)
        ax1.tick_params(axis='y', labelcolor='#d62728')
        ax1.grid(True, linestyle=':', alpha=0.6)

        ax2 = ax1.twinx()
        ax2.set_ylabel('Path', color='#1f77b4', fontsize=9)
        ax2.plot(x_vals, lengths, color='#1f77b4', marker='s', linestyle='--', linewidth=1.2, markersize=4)
        ax2.tick_params(axis='y', labelcolor='#1f77b4')

        valid_len = [l for l in lengths if l > 0]
        if valid_len:
            min_len = min(valid_len)
            best_idx = 0; min_nodes = float('inf')
            for i in range(len(lengths)):
                if lengths[i] > 0 and lengths[i] <= min_len * 1.05 and nodes[i] < min_nodes:
                    min_nodes = nodes[i]; best_idx = i
            sw, sn = x_vals[best_idx], nodes[best_idx]
            ax1.annotate(f'w={sw}', xy=(sw, sn), xytext=(3, 5), textcoords='offset points',
                         bbox=dict(boxstyle="round,pad=0.2", fc="yellow", alpha=0.5), fontsize=8)
        ax1.set_title(title, fontsize=10, fontweight='bold')

    def start_planning_and_movement(self, weight_to_use: float = 4.0):
        plt.style.use('seaborn-v0_8-muted')
        w_range = [1.0, 1.2, 1.5, 2.0, 3.0, 5.0, 8.0, 12.0, 18.0]

        # Figure 1 & 2
        fig12, (ax_f1, ax_f2) = plt.subplots(2, 1, figsize=(10, 10))
        nodes_f1, lengths_f1 = [], []
        for w in w_range:
            self.planner.weight = w; self.planner.find_path(self.start_grid, self.goal_grid)
            nodes_f1.append(self.planner.last_stats["nodes_expanded"]); lengths_f1.append(self.planner.last_stats["path_length"])
        self._draw_dual_axis_core(ax_f1, w_range, nodes_f1, lengths_f1, "Figure 1: Current Maze Performance Scan")

        for w_val in [1.2, 1.5, 2.0, 3.0, 5.0]:
                    self.planner.weight = w_val
                    self.planner.find_path(self.start_grid, self.goal_grid, record=True)
                    history = self.planner.weight_history
                    if history:
                        progress = [i / (len(history)-1) * 100 for i in range(len(history))]
                        ax_f2.plot(progress, history, label=f'w_start={w_val}', linewidth=1.5)
            
        ax_f2.set_title("Figure 2: Weight Decay Process", fontsize=10, fontweight='bold')
        ax_f2.set_xlabel("Search Progress (%)", fontsize=9)
        ax_f2.set_ylabel("Dynamic Weight $w(n)$", fontsize=9)
        ax_f2.grid(True, linestyle=':', alpha=0.6)
        ax_f2.legend(fontsize=8)
        
        plt.tight_layout()
        plt.show()

        fig_ext, axes = plt.subplots(5, 2, figsize=(14, 12))
        for i, s in enumerate([3, 5, 6, 7, 10]):
            n, l = [], []
            t_cfg = render_map.MazeMapConfig(rows=s, cols=s, cell_size=90, loop_percent=10, start_point=(1,1), goal_point=(s-1,s-1))
            t_m, _, _ = render_map.MazeMapConfig.create_scene(t_cfg)
            t_p = AStarStaticPlanner(t_m, t_cfg)
            for w in w_range:
                t_p.weight = w; t_p.find_path((1,1), (s,s))
                n.append(t_p.last_stats["nodes_expanded"]); l.append(t_p.last_stats["path_length"])
            self._draw_dual_axis_core(axes[i, 0], w_range, n, l, f"Size Impact(rows and cols): {s}x{s}")

        for i, difficulty in enumerate([2, 3, 4, 5, 6]):
            n, l = [], []
            t_cfg = render_map.MazeMapConfig(rows=11, cols=11, cell_size=90, loop_percent=10-difficulty, start_point=(1,1), goal_point=(6,6))
            t_m, _, _ = render_map.MazeMapConfig.create_scene(t_cfg)
            t_p = AStarStaticPlanner(t_m, t_cfg)
            for w in w_range:
                t_p.weight = w; t_p.find_path((1,1), (6,6))
                n.append(t_p.last_stats["nodes_expanded"]); l.append(t_p.last_stats["path_length"])
            self._draw_dual_axis_core(axes[i, 1], w_range, n, l, f"Difficulty Impact: {difficulty}")
        
        plt.suptitle("Extended Parameter Analysis", fontsize=14, fontweight='bold')
        plt.tight_layout(rect=[0, 0.03, 1, 0.95]); plt.show()

        self.planner.weight = weight_to_use
        self.path = self.planner.find_path(self.start_grid, self.goal_grid, record=True)
        if self.path: self.path.pop(0); self.is_planning_complete = True

    def update(self):
        if not self.is_planning_complete or not self.path:
            self.robot.speed = 0; self.robot.update(); return
        tx, ty = PointTF.grid_to_pixel_center(*self.path[0], self.maze_config)
        dist = math.hypot(self.robot.x - tx, self.robot.y - ty)
        if dist < self.robot.radius / 2: 
            self.path.pop(0)
            if not self.path: self.robot.goal = True; return
        dx, dy = tx - self.robot.x, ty - self.robot.y
        norm = math.hypot(dx, dy)
        if norm > 0: self.robot.direction = (dx / norm, dy / norm)
        self.robot.update()

    def draw_path(self):
        if self.path:
            pts = [PointTF.grid_to_pixel_center(*g, self.maze_config) for g in self.path]
            for i in range(len(pts)-1):
                pygame.draw.line(self.scene_renderer.screen, (255, 165, 0), pts[i], pts[i+1], 4)

# ===================== Main =====================
if __name__ == "__main__":
    cfg = render_map.MazeMapConfig(rows=10, cols=10, cell_size=60, loop_percent= 50, start_point=(1,1), goal_point=(13,13))
    maze, maze_data, _ = render_map.MazeMapConfig.create_scene(cfg)

    obs = render_map.generate_dynamic_obstacles(
        num_slow=15, num_fast=10, slow_speed_range=(0.5,1), fast_speed_range=(1.5,3),
        map_width=cfg.map_size[0], map_height=cfg.map_size[1], radius=10,
        maze_walls=maze_data.walls
    )

    robot = render_map.N7carRobot(maze_data.start_pixel[0], maze_data.start_pixel[1], 2.5, (1, 0), 12)
    renderer = render_map.SceneRenderer(maze_data, cfg, obs, robot)
    test_module = AstarTest(robot, maze_data, maze, cfg, renderer)
    
    test_module.start_planning_and_movement(weight_to_use=4.0) 

    clock = pygame.time.Clock()
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT: running = False
        for o in obs: o.update(maze_data.walls)
        test_module.update(); renderer.draw(); test_module.draw_path()
        pygame.display.flip(); clock.tick(30)
    pygame.quit()