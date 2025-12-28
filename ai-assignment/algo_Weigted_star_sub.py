'''
Author: Nagisa 2964793117@qq.com
Description: A* 参数扫描
'''

import heapq
import math
import matplotlib.pyplot as plt
from typing import Tuple, List, Dict
import pygame
import render_map
import matplotlib.ticker as ticker

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
    
# ===================== A* 算法的实现 =====================
class AStarStaticPlanner:
    def __init__(self, maze_map, maze_config, weight: float = 1.0):
        self.maze_map = maze_map.maze_map 
        self.rows = maze_config.rows
        self.cols = maze_config.cols
        self.maze_config = maze_config 
        self.weight = weight
        self.last_stats = {"nodes_expanded": 0, "path_length": 0}

        self.path_refiner = PathRefiner(
                mazeconfig=self.maze_config,  
                subdivision_factor=3, 
        )
        
    def heuristic(self, a: Tuple[int, int], b: Tuple[int, int]) -> int:
        r1, c1 = a
        r2, c2 = b
        # 使用曼哈顿距离作为启发函数
        return abs(r1 - r2) + abs(c1 - c2)

    def get_neighbors(self, current: Tuple[int, int]) -> List[Tuple[int, int]]:
        r, c = current
        neighbors = []
        moves = {'N': (-1, 0), 'S': (1, 0), 'E': (0, 1), 'W': (0, -1)}
        current_cell_data = self.maze_map.get(current)
        if not current_cell_data: return []

        for direction, (dr, dc) in moves.items():
            if current_cell_data.get(direction) == 1:
                nr, nc = r + dr, c + dc
                if 1 <= nr <= self.rows and 1 <= nc <= self.cols:
                    neighbors.append((nr, nc))       
        return neighbors

    def find_path(self, start_grid: Tuple[int, int], goal_grid: Tuple[int, int]) -> List[Tuple[int, int]]:
        open_set = []
        heapq.heappush(open_set, (0, start_grid))
        g_score = {start_grid: 0}
        came_from = {}
        nodes_expanded = 0
        
        while open_set:
            current_f, current_node = heapq.heappop(open_set)
            nodes_expanded += 1

            if current_node == goal_grid:
                path = []
                temp = current_node
                while temp in came_from:
                    path.append(temp)
                    temp = came_from[temp]
                path.append(start_grid)
                self.last_stats = {"nodes_expanded": nodes_expanded, "path_length": len(path)}
                return path[::-1]

            for neighbor in self.get_neighbors(current_node):
                tentative_g = g_score[current_node] + 1
                if tentative_g < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current_node
                    g_score[neighbor] = tentative_g
                    # Weighted A*
                    f = tentative_g + self.weight * self.heuristic(neighbor, goal_grid)
                    heapq.heappush(open_set, (f, neighbor))
        return []

    def find_and_refine_path(self, start_grid: Tuple[int, int], goal_grid: Tuple[int, int]) -> List[List[Tuple[float, float]]]:
        coarse_path = self.find_path(start_grid, goal_grid)
        return self.path_refiner.subdivide_path(coarse_path) if coarse_path else []
        
# ===================== 路径细化工具类 =====================
class PathRefiner:
    def __init__(self, mazeconfig, subdivision_factor: int = 3):
        self.subdivision_factor = subdivision_factor
        self.original_cell_size = mazeconfig.cell_size
        self.sub_cell_size = self.original_cell_size / self.subdivision_factor

    def subdivide_path(self, coarse_path: List[Tuple[int, int]]) -> List[List[Tuple[float, float]]]:
        fine_segments = []
        for r_grid, c_grid in coarse_path:
            x_start, y_start = (c_grid - 1) * self.original_cell_size, (r_grid - 1) * self.original_cell_size
            segment = []
            for r_sub in range(self.subdivision_factor):
                for c_sub in range(self.subdivision_factor):
                    dx, dy = (c_sub + 0.5) * self.sub_cell_size, (r_sub + 0.5) * self.sub_cell_size
                    segment.append((x_start + dx, y_start + dy))
            fine_segments.append(segment)
        return fine_segments

# ===================== A* 测试类  =====================
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
        self.fine_path_segments: List[List[Tuple[float, float]]] = [] 
        self.is_planning_complete = False

    def plot_sweep_results(self, sweep_data: List[Dict]):
        """ A* 参数扫描分析图 - 美化版 """
        # 设置全局风格
        plt.style.use('seaborn-v0_8-muted') 
        weights = [d['weight'] for d in sweep_data]
        nodes = [d['nodes_expanded'] for d in sweep_data]
        lengths = [d['path_length'] for d in sweep_data]

        # 打印详细原始数据
        print("\n" + "="*30)
        print(f"{'Weight (w)':<12} | {'Nodes Exp.':<12} | {'Path Len.':<12}")
        print("-"*40)
        for d in sweep_data:
            print(f"{d['weight']:<12.2f} | {d['nodes_expanded']:<12} | {d['path_length']:<12}")
        print("="*30)

        fig, ax1 = plt.subplots(figsize=(12, 7), dpi=100)
        
        # 绘制节点扩展数 - 左轴
        color_nodes = '#d62728' 
        ax1.set_xlabel('Heuristic Weight (w)', fontsize=12, fontweight='bold')
        ax1.set_ylabel('Nodes Expanded (Search Speed)', color=color_nodes, fontsize=12, fontweight='bold')
        line1 = ax1.plot(weights, nodes, color=color_nodes, marker='o', markersize=8, 
                         linewidth=2.5, label='Nodes Expanded (Speed)', zorder=3)
        ax1.fill_between(weights, nodes, color=color_nodes, alpha=0.1)
        ax1.tick_params(axis='y', labelcolor=color_nodes, labelsize=10)
        ax1.grid(True, linestyle='--', alpha=0.6)

        # 绘制路径长度 - 右轴
        ax2 = ax1.twinx()
        color_len = '#1f77b4' 
        ax2.set_ylabel('Path Length (Quality)', color=color_len, fontsize=12, fontweight='bold')
        line2 = ax2.plot(weights, lengths, color=color_len, marker='s', markersize=8, 
                         linewidth=2.5, linestyle='--', label='Path Length (Quality)', zorder=3)
        ax2.tick_params(axis='y', labelcolor=color_len, labelsize=10)

        # 精细化坐标刻度
        ax1.xaxis.set_major_locator(ticker.MultipleLocator(1.0))
        ax1.xaxis.set_minor_locator(ticker.MultipleLocator(0.2))
        plt.xlim(min(weights) - 0.5, max(weights) + 0.5)

        # 寻找最佳权重点   节点数最少，路径长度次之
        min_len = min(lengths)
        best_idx = 0
        min_node_at_best_len = float('inf')
        for i in range(len(lengths)):
            if lengths[i] == min_len and nodes[i] < min_node_at_best_len:
                min_node_at_best_len = nodes[i]
                best_idx = i
        
        sweet_w = weights[best_idx]
        sweet_node = nodes[best_idx]
        
        offset_x = 0.8
        offset_y = max(nodes) * 0.15

        ax1.annotate(f'Sweet Spot (w={sweet_w})', 
                     xy=(sweet_w, sweet_node), 
                     xytext=(sweet_w + offset_x, sweet_node + offset_y),
                     arrowprops=dict(arrowstyle="->", connectionstyle="arc3,rad=.2", color='black', lw=1.5),
                     fontsize=11, fontweight='bold', 
                     bbox=dict(boxstyle="round4,pad=0.5", fc="yellow", alpha=0.3, ec="black"))

        plt.title('Weighted A*: Parameters Sweep', fontsize=15, pad=20)
        
        # 合并并放置图例
        lns = line1 + line2
        labs = [l.get_label() for l in lns]
        ax1.legend(lns, labs, loc='upper right', frameon=True, shadow=True)

        fig.tight_layout()
        print(f"Recommended weight based on scan: w = {sweet_w}")
        print("Close the plot to start Pygame simulation...")
        plt.show()

    def start_planning_and_movement(self, weight_to_use: float = 1.0):
        test_weights = [0.5,1.0, 1.1, 1.2, 1.5, 2.0, 2.5, 3.0, 4.0, 5.0, 7.0, 10.0]
        sweep_results = []
        for w in test_weights:
            self.planner.weight = w
            self.planner.find_path(self.start_grid, self.goal_grid)
            res = {"weight": w, **self.planner.last_stats}
            sweep_results.append(res)
        
        self.plot_sweep_results(sweep_results)

        self.planner.weight = weight_to_use
        self.fine_path_segments = self.planner.find_and_refine_path(self.start_grid, self.goal_grid)
        self.path = self.planner.find_path(self.start_grid, self.goal_grid)
        
        if self.path:
            self.path.pop(0)
            self.is_planning_complete = True
        else:
            print("Path planning failed!")

    def update(self):
        if not self.is_planning_complete or not self.path:
            self.robot.speed = 0
            self.robot.update()
            return

        next_grid = self.path[0]
        target_x, target_y = PointTF.grid_to_pixel_center(*next_grid, self.maze_config)
        dist = math.hypot(self.robot.x - target_x, self.robot.y - target_y)
        
        if dist < self.robot.radius / 2: 
            self.path.pop(0)
            if not self.path:
                self.robot.goal = True
                return
        
        dx, dy = target_x - self.robot.x, target_y - self.robot.y
        norm = math.hypot(dx, dy)
        if norm > 0:
            self.robot.direction = (dx / norm, dy / norm)
        self.robot.update()

    def draw_path(self):
        if self.path:
            path_pixels = [PointTF.grid_to_pixel_center(*grid, self.maze_config) for grid in self.path]
            for i in range(len(path_pixels) - 1):
                pygame.draw.line(self.scene_renderer.screen, (255, 165, 0), path_pixels[i], path_pixels[i+1], 5)
        if self.fine_path_segments:
            for segment in self.fine_path_segments:
                for x, y in segment:
                    pygame.draw.circle(self.scene_renderer.screen, (0, 255, 255), (int(x), int(y)), 2)

# ===================== 主程序运行 =====================
if __name__ == "__main__":
    mazeconfig = render_map.MazeMapConfig(rows=12, cols=12, cell_size=90, loop_percent=80, start_point=(1,1), goal_point=(8,8))
    maze, maze_data, renderer = render_map.MazeMapConfig.create_scene(mazeconfig)

    dynamic_obstacles = render_map.generate_dynamic_obstacles(
        num_slow=15, num_fast=10, slow_speed_range=(0.5,1), fast_speed_range=(1.5,3),
        map_width=mazeconfig.map_size[0], map_height=mazeconfig.map_size[1], radius=10,
        maze_walls=maze_data.walls
    )

    Nagisa_robot = render_map.N7carRobot(
        x=maze_data.start_pixel[0], y=maze_data.start_pixel[1],
        speed=2.5, direction=(1,0), radius=15
    )

    scene_renderer = render_map.SceneRenderer(maze_data, mazeconfig, dynamic_obstacles, Nagisa_robot)


    astar_test_module = AstarTest(Nagisa_robot, maze_data, maze, mazeconfig, scene_renderer)
    astar_test_module.start_planning_and_movement(weight_to_use=2.0) 

    # 仿真主循环
    clock = pygame.time.Clock()
    running = True
    pygame.font.init()

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
        
        for obs in dynamic_obstacles:
            obs.update(maze_data.walls)

        astar_test_module.update()
        scene_renderer.draw()
        astar_test_module.draw_path()

        pygame.display.flip()
        clock.tick(30) 

    scene_renderer.quit()