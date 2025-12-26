# 地图生成 动态障碍生成 机器人生成 场景渲染
import render_map       # 地图渲染模块
import algo_VO as VO    # VO 动态避障算法
import algo_A_star_sub as A # A* 静态避障算法
import pygame           # 主渲染器
import numpy as np

# ======= 用户输入参数 =======
def get_int_input(prompt, minv, maxv):
    while True:
        try:
            v = int(input(prompt))
            if minv <= v <= maxv:
                return v
            else:
                print(f"请输入{minv}-{maxv}之间的整数！")
        except Exception:
            print("输入无效，请重新输入！")

rows = get_int_input("请输入迷宫行数(2-10): ", 2, 10)
cols = get_int_input("请输入迷宫列数(2-10): ", 2, 10)
difficulty = get_int_input("请输入迷宫难度(0-10, 10最难): ", 0, 10)
num_slow = get_int_input("请输入慢速动态障碍物数量: ", 0, 50)
num_fast = get_int_input("请输入快速动态障碍物数量: ", 0, 50)

cell_size = 90
loop_percent = 10 - difficulty  # 10最难->0, 0最简单->10
import random

def random_grid(rows, cols):
    return (random.randint(1, rows), random.randint(1, cols))

def new_goal_point(rows, cols, start_grid):
    while True:
        g = random_grid(rows, cols)
        if g != start_grid:
            return g

# ======= 初始化场景 =======
# 生成迷宫
start_grid = (1, 1)
goal_grid = (rows, cols)
mazeconfig = render_map.MazeMapConfig(rows=rows, cols=cols, cell_size=cell_size, loop_percent=loop_percent, start_point=start_grid, goal_point=goal_grid)
maze, maze_data, renderer = render_map.MazeMapConfig.create_scene(mazeconfig)
# 生成动态障碍物
dynamic_obstacles = render_map.generate_dynamic_obstacles(
    num_slow=num_slow, num_fast=num_fast, slow_speed_range=(0.5,1),fast_speed_range=(1,3),
    map_width=mazeconfig.map_size[0], map_height=mazeconfig.map_size[1],radius=8,
    maze_walls=maze_data.walls
)
obstacle_states = render_map.get_obstacle_states(dynamic_obstacles)
# 生成N7car机器人
N7car = render_map.N7carRobot(
    x=maze_data.start_pixel[0],
    y=maze_data.start_pixel[1],
    direction=(1,0),
    speed=3,
    radius=10
)
# ======= 路径规划 =======
def plan_path_and_set_goal(start_grid, goal_grid):
    print(maze_data)
    astar_init = A.AStarInit(maze_data=maze_data, mazeconfig=mazeconfig)
    astar_init.start_grid = start_grid
    astar_init.goal_grid = goal_grid
    planner = A.AStarStaticPlanner(maze, mazeconfig)
    path_grid = planner.find_path(start_grid, goal_grid)
    path_pix = [A.PointTF.grid_to_pixel_center(r, c, mazeconfig) for (r, c) in path_grid]
    path_pix_refined = planner.find_and_refine_path(start_grid, goal_grid)
    return path_grid, path_pix, path_pix_refined

current_grid = A.PointTF.pixel_to_grid(N7car.x, N7car.y, mazeconfig)
path_grid, path_pix, path_pix_refined = plan_path_and_set_goal(current_grid, goal_grid)

collision_count = 0
def collision_callback():
    global collision_count
    collision_count += 1

vo_planner = VO.VO_Avoidance(
    robot=N7car,
    obstacles=dynamic_obstacles,
    v_max=5,
    inflate_ratio=1.5,
    path_goal=path_pix_refined,
    vision_range=100,
    maze_data=maze_data,
)

vo_planner.set_acc_limit(5)
vo_planner.collision_callback = collision_callback
vo_planner.path_idx = 0
scene_renderer = render_map.SceneRenderer(maze_data, mazeconfig, dynamic_obstacles, N7car)


# ======= 开始渲染 =======
clock = pygame.time.Clock()
running = True
pygame.font.init() 
print(obstacle_states)
print(N7car.get_N7car_state())

paused = False
font = pygame.font.SysFont(None, 48)
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_SPACE:
                paused = not paused
            elif event.key == pygame.K_q:
                running = False
    if paused:
        continue
    # 更新动态障碍物
    for obs in dynamic_obstacles:
        obs.update(maze_data.walls)
    vo_planner.update()
    # 绘制碰撞次数
    collision_text = font.render(f"collision times: {collision_count}", True, (255,0,0))
    scene_renderer.screen.blit(collision_text, (20, 20))
    N7car.update()
    # 检查是否到达目标点
    if vo_planner.path_idx >= len(vo_planner.path_goal):
        # 重新随机目标点并规划路径，起点为N7car当前位置
        new_goal = new_goal_point(rows, cols, None)
        goal_grid = new_goal
        # 获取N7car当前位置（像素转网格）
        current_grid = A.PointTF.pixel_to_grid(N7car.x, N7car.y, mazeconfig)
        path_grid, path_pix, path_pix_refined = plan_path_and_set_goal(current_grid, goal_grid)
        vo_planner.path_goal = path_pix_refined
        vo_planner.path_idx = 0
        # 更新迷宫终点
        mazeconfig.goal_point = goal_grid
        maze_data.goal_pixel = A.PointTF.grid_to_pixel_center(*goal_grid, mazeconfig)
    clock.tick(30)
    scene_renderer.draw()
    scene_renderer.screen.blit(collision_text, (20, 20))
    if vo_planner.path_idx < len(vo_planner.path_goal):
        robot_state = N7car.get_N7car_state()
        VOs = vo_planner.build_VOs(vo_planner.obstacles, robot_state)
        # 速度空间安全区可视化（分辨率更细，圆点更细，半透明）
        vo_planner.visualize_velocity_space(scene_renderer.screen, robot_state, VOs, num_angles=20, num_radii=20)
    if hasattr(vo_planner, 'last_target_point'):
        pygame.draw.circle(scene_renderer.screen, (255,0, 0), (int(vo_planner.last_target_point[0]), int(vo_planner.last_target_point[1])), 8)
    pygame.display.flip()

scene_renderer.quit()