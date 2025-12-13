# 地图生成 动态障碍生成 机器人生成 场景渲染
import render_map
# import algo_VO # VO 动态避障算法
import algo_A # A* 静态避障算法
import pygame # 主渲染器

# === 配置迷宫地图 ===
# 格子个数，单元格像素大小，环路百分比，起点，终点
mazeconfig = render_map.MazeMapConfig(rows=12, cols=12, cell_size=90, loop_percent=80, start_point=(1,1), goal_point=(8,8))
maze, maze_data, renderer = render_map.MazeMapConfig.create_scene(mazeconfig)

# === 配置动态障碍 ===
# 低速障碍物个数，速度范围，高速障碍物个数，速度范围, 半径大小
dynamic_obstacles = render_map.generate_dynamic_obstacles(
    num_slow=8, num_fast=3, slow_speed_range=(0.5,1.5),fast_speed_range=(2,5),
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

# === 开始渲染 ===
clock = pygame.time.Clock()
running = True
pygame.font.init() 
print( obstacle_states)
print( N7car.get_N7car_state())
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
    # 更新动态障碍物
    for obs in dynamic_obstacles:
        obs.update(maze_data.walls)
    N7car.update()
    clock.tick(30)
    # 统一渲染
    scene_renderer.draw()
    
scene_renderer.quit()