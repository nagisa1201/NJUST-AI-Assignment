from canvas import Canvas
from algorithms import GaussianRandomMovement, ImprovedPotentialFieldNavigation  # 替换为改进版算法
from ui import UIRenderer
from interact import InteractionHandler

def main():
    # 1. 初始化画布
    canvas = Canvas(width=1200, height=800)

    # 2. 初始化算法（使用改进版势场法）
    dynamic_movement_algo = GaussianRandomMovement(base_speed=2, sigma=0.5)
    robot_navigation_algo = ImprovedPotentialFieldNavigation(
        max_speed=3, repulsion_strength=6, safe_distance=70, attract_decay=0.8
    )

    # 3. 初始化渲染器
    renderer = UIRenderer(canvas, width=1200, height=800)

    # 4. 初始化交互处理器
    interact_handler = InteractionHandler(
        canvas, renderer, dynamic_movement_algo, robot_navigation_algo
    )

    # 5. 初始化仿真场景
    SO_count = (30, 50)  # 静态障碍物数量范围
    canvas.generate_random_static_obstacles(SO_count)
    canvas.generate_random_dynamic_obstacles(dynamic_movement_algo, count=4)
    canvas.init_robot(robot_navigation_algo)

    # 6. 主循环
    while interact_handler.is_running():
        # 处理用户输入
        interact_handler.handle_events()

        # 更新画布状态
        canvas.update_all()

        # 渲染画面
        renderer.render_all()

        # 控制帧率
        renderer.get_fps()

    # 退出
    renderer.quit()

if __name__ == "__main__":
    main()