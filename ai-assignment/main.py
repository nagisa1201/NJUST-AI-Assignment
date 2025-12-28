# 地图生成 动态障碍生成 机器人生成 场景渲染
import render_map       # 地图渲染模块
import algo_VO as VO    # VO 动态避障算法
import algo_improved_A_star as A # A* 改良算法
import pygame           # 主渲染器
import time 


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

v_max_input = get_int_input("请输入N7car最大速度(1-10)，建议为5: ", 1, 10)
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
# N7 represents Nagisa and 7415
N7car = render_map.N7carRobot(
    x=maze_data.start_pixel[0],
    y=maze_data.start_pixel[1],
    direction=(1,0),
    speed=3,  # 这里3只是一个占位符，在VO里面立马就被修改了
    radius=10
)
# ======= 路径规划 =======
def plan_path_and_set_goal(start_grid, goal_grid):
    astar_init = A.AStarInit(maze_data=maze_data, mazeconfig=mazeconfig)
    astar_init.start_grid = start_grid
    astar_init.goal_grid = goal_grid
    planner = A.AStarStaticPlanner(maze, mazeconfig)
    path_grid = planner.find_path(start_grid, goal_grid)
    path_pix = [A.PointTF.grid_to_pixel_center(r, c, mazeconfig) for (r, c) in path_grid]
    path_pix_refined = planner.find_and_refine_path(start_grid, goal_grid)
    return path_grid, path_pix, path_pix_refined, planner

current_grid = A.PointTF.pixel_to_grid(N7car.x, N7car.y, mazeconfig)
path_grid, path_pix, path_pix_refined, current_planner = plan_path_and_set_goal(current_grid, goal_grid)

# 统计碰撞次数
collision_count = 0
def collision_callback():
    global collision_count
    collision_count += 1

# ======= 初始化VO避障器 =======
vo_planner = VO.VO_Avoidance(
    robot=N7car,
    obstacles=dynamic_obstacles,
    v_max=v_max_input,
    inflate_ratio=1.2,
    path_goal=path_pix_refined,
    vision_range=100,
    maze_data=maze_data,
)

vo_planner.collision_callback = collision_callback
vo_planner.path_idx = 0
scene_renderer = render_map.SceneRenderer(maze_data, mazeconfig, dynamic_obstacles, N7car)


# ======= 开始渲染 =======
def draw_astar_scores(screen, planner, mazeconfig):
    """渲染函数"""
    if not planner or not hasattr(planner, 'last_g_scores'):
        return
    score_font = pygame.font.SysFont('Arial', 12)
    f_font = pygame.font.SysFont('Arial', 14, bold=True)
    for node, g in planner.last_g_scores.items():
        if g == float('inf'): continue
        
        f = planner.last_f_scores.get(node, 0)
        h = f - g
        r, c = node
        
        cx, cy = A.PointTF.grid_to_pixel_center(r, c, mazeconfig)
        
        f_surface = f_font.render(f"F:{int(f)}", True, (150, 0, 0)) # 红色
        gh_surface = score_font.render(f"g:{int(g)} h:{int(h)}", True, (60, 60, 60)) # 灰色
        
        screen.blit(f_surface, (cx - f_surface.get_width()//2, cy - 18))
        screen.blit(gh_surface, (cx - gh_surface.get_width()//2, cy + 2))


clock = pygame.time.Clock()
running = True
pygame.font.init() 

paused = False
font = pygame.font.SysFont(None, 28)
running_time = 0.0
last_time = time.time()
success_count = 0

while running:
    current_time = time.time()
    delta_time = current_time - last_time
    last_time = current_time
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_SPACE:
                paused = not paused
            elif event.key == pygame.K_q:
                running = False
    if paused:
        global_clock = time.time()
        continue

    running_time += delta_time
    for obs in dynamic_obstacles:
        obs.update(maze_data.walls)
    vo_planner.update()
   
    collision_text = font.render(f"collision times: {collision_count}", True, (255,0,0))
    clock_text = font.render(f"running time:{running_time:.1f}", True, (255,0,0))
    success_count_text = font.render(f"success times:{success_count}", True, (255,0,0))

    N7car.update()

    if vo_planner.path_idx >= len(vo_planner.path_goal):

        new_goal = new_goal_point(rows, cols, None)
        success_count += 1
        goal_grid = new_goal
        # 获取N7car当前位置
        current_grid = A.PointTF.pixel_to_grid(N7car.x, N7car.y, mazeconfig)
        path_grid, path_pix, path_pix_refined, current_planner = plan_path_and_set_goal(current_grid, goal_grid)
        vo_planner.path_goal = path_pix_refined
        vo_planner.path_idx = 0
        # 更新迷宫终点
        mazeconfig.goal_point = goal_grid
        maze_data.goal_pixel = A.PointTF.grid_to_pixel_center(*goal_grid, mazeconfig)
    clock.tick(30)
    scene_renderer.draw()
    if 'current_planner' in locals():
        draw_astar_scores(scene_renderer.screen, current_planner, mazeconfig)

    scene_renderer.screen.blit(collision_text, (20, 20))
    scene_renderer.screen.blit(success_count_text, (20, 40))
    scene_renderer.screen.blit(clock_text, (20, 60))
    if vo_planner.path_idx < len(vo_planner.path_goal):
        robot_state = N7car.get_N7car_state()
        VOs = vo_planner.build_VOs(vo_planner.obstacles, robot_state)

        vo_planner.visualize_velocity_space(scene_renderer.screen, robot_state, VOs, num_angles=20, num_radii=20)
    if hasattr(vo_planner, 'last_target_point'):
        pygame.draw.circle(scene_renderer.screen, (255,0, 0), (int(vo_planner.last_target_point[0]), int(vo_planner.last_target_point[1])), 8)
    pygame.display.flip()

scene_renderer.quit()