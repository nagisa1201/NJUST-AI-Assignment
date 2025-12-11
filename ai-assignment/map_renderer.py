import pygame
import pyamaze
from typing import Tuple
from dataclasses import dataclass

# ===Define of MazeMap and Mazemap generation===

#Define of Mazemap
@dataclass
class MazeMapConfig:
    rows:int
    cols:int
    cell_size:int
    loop_percent:int
    start_point:Tuple[int,int] = None
    goal_point:Tuple[int,int] = None
    
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
    rows, cols = maze.rows, maze.cols
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
mazemap_config = MazeMapConfig(start_point=(2,5), goal_point=(2,20), rows=20, cols=20, cell_size=40, loop_percent=50)
maze_generator = MazeMapGenerator(mazemap_config)
maze_data = maze_generator.generate_maze()
maze_data_for_pygame = convert_maze_to_pygame(maze_data, mazemap_config=mazemap_config, cell_size=mazemap_config.cell_size)
renderer = MazeRenderer(maze_data_for_pygame,mazemap_config)

running = True
while running:
    if not renderer.handle_events():
        running = False
    renderer.draw()
renderer.quit()
