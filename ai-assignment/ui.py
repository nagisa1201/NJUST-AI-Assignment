import pygame
import math
import time
from pyamaze import maze, agent, textLabel, COLOR

map = maze(20, 20)  # 20行20列
map.CreateMaze(loopPercent=50)  # 默认入口在右下，出口在左上
map.run()

start_time = time.time()

# 全局样式配置
COLORS = {
    "bg_primary": (240, 248, 255),
    "bg_secondary": (224, 238, 255),
    "grid": (190, 210, 255),
    "robot": (0, 200, 80),
    "robot_safe": (0, 200, 80, 50),
    "robot_path": (0, 180, 80),
    "static_obs": (120, 120, 120),
    "dynamic_obs": (255, 140, 0),
    "dynamic_obs_path": (255, 140, 0),
    "target": (255, 60, 60),
    "text_primary": (30, 30, 30),
    "text_secondary": (80, 80, 80),
    "text_success": (0, 150, 0),
    "text_error": (200, 0, 0),
    "panel_bg": (255, 255, 255, 200),
    "border": (180, 180, 180)
}

class UIRenderer:
    """UI渲染器（负责所有可视化）"""
    def __init__(self, canvas, width=1200, height=800):
        pygame.init()
        self.width = width
        self.height = height
        self.canvas = canvas

        # 初始化显示
        self.screen = pygame.display.set_mode((width, height))
        pygame.display.set_caption("机器人避障导航仿真")
        self.clock = pygame.time.Clock()

        # 字体初始化
        self.font_large = pygame.font.SysFont("Microsoft YaHei", 28)
        self.font_medium = pygame.font.SysFont("Microsoft YaHei", 22)
        self.font_small = pygame.font.SysFont("Microsoft YaHei", 18)

        # 渲染优化：创建半透明面板surface
        self.panel_surface = pygame.Surface((300, 220), pygame.SRCALPHA)
        self.panel_surface.fill(COLORS["panel_bg"])

    def draw_background(self):
        """绘制背景和网格"""
        # 渐变背景（模拟）
        self.screen.fill(COLORS["bg_primary"])
        for y in range(self.height):
            alpha = int(255 * y / self.height)
            line_surface = pygame.Surface((self.width, 1))
            line_surface.fill((COLORS["bg_secondary"][0], COLORS["bg_secondary"][1], COLORS["bg_secondary"][2], alpha))
            self.screen.blit(line_surface, (0, y))

        # 绘制网格
        grid_size = 10
        for x in range(0, self.width, grid_size):
            pygame.draw.line(self.screen, COLORS["grid"], (x, 0), (x, self.height), 1)
        for y in range(0, self.height, grid_size):
            pygame.draw.line(self.screen, COLORS["grid"], (0, y), (self.width, y), 1)

    def draw_static_obstacles(self):
        """绘制静态障碍物"""
        for obs in self.canvas.static_obstacles:
            left = obs.x - obs.width/2
            top = obs.y - obs.height/2
            # 带边框的矩形
            pygame.draw.rect(self.screen, obs.color, (left, top, obs.width, obs.height))
            pygame.draw.rect(self.screen, COLORS["border"], (left, top, obs.width, obs.height), 2)

    def draw_dynamic_obstacles(self):
        """绘制动态障碍物（含轨迹）"""
        for obs in self.canvas.dynamic_obstacles:
            # 绘制轨迹
            if len(obs.path) > 1:
                for i in range(1, len(obs.path)):
                    alpha = int(255 * i / len(obs.path))
                    color = (
                        COLORS["dynamic_obs_path"][0],
                        COLORS["dynamic_obs_path"][1],
                        COLORS["dynamic_obs_path"][2],
                        alpha
                    )
                    pygame.draw.line(
                        self.screen, color,
                        (int(obs.path[i-1][0]), int(obs.path[i-1][1])),
                        (int(obs.path[i][0]), int(obs.path[i][1])),
                        1
                    )

            # 绘制障碍物本体
            pygame.draw.circle(self.screen, obs.color, (int(obs.x), int(obs.y)), obs.radius)
            pygame.draw.circle(self.screen, COLORS["border"], (int(obs.x), int(obs.y)), obs.radius, 2)

            # 绘制移动方向
            dir_x = obs.x + math.cos(obs.movement_algorithm.current_direction) * (obs.radius + 5)
            dir_y = obs.y + math.sin(obs.movement_algorithm.current_direction) * (obs.radius + 5)
            pygame.draw.line(self.screen, COLORS["text_primary"], (obs.x, obs.y), (dir_x, dir_y), 2)

    def draw_robot(self):
        """绘制机器人（含安全范围、路径）"""
        if not self.canvas.robot:
            return

        robot = self.canvas.robot

        # 绘制路径
        if len(robot.path) > 1:
            path_points = [(int(p[0]), int(p[1])) for p in robot.path]
            # 批量绘制路径（优化性能）
            for i in range(1, len(path_points)):
                alpha = int(100 + 155 * i / len(path_points))
                color = (COLORS["robot_path"][0], COLORS["robot_path"][1], COLORS["robot_path"][2], alpha)
                pygame.draw.line(self.screen, color, path_points[i-1], path_points[i], 2)

        # 绘制安全范围（半透明圆）
        safe_radius = robot.radius + robot.navigation_algorithm.safe_radius_extra
        safe_surface = pygame.Surface((safe_radius*2, safe_radius*2), pygame.SRCALPHA)
        pygame.draw.circle(safe_surface, COLORS["robot_safe"], (safe_radius, safe_radius), safe_radius)
        self.screen.blit(safe_surface, (robot.x - safe_radius, robot.y - safe_radius))

        # 绘制机器人本体
        pygame.draw.circle(self.screen, COLORS["robot"], (int(robot.x), int(robot.y)), robot.radius)
        pygame.draw.circle(self.screen, COLORS["border"], (int(robot.x), int(robot.y)), robot.radius, 2)

        # 绘制移动方向
        if robot.target:
            dir_x = robot.x + math.cos(math.atan2(robot.target[1]-robot.y, robot.target[0]-robot.x)) * (robot.radius + 5)
            dir_y = robot.y + math.sin(math.atan2(robot.target[1]-robot.y, robot.target[0]-robot.x)) * (robot.radius + 5)
            pygame.draw.line(self.screen, COLORS["text_primary"], (robot.x, robot.y), (dir_x, dir_y), 3)

        # 绘制目标点
        if robot.target:
            pygame.draw.circle(self.screen, COLORS["target"], (int(robot.target[0]), int(robot.target[1])), 8)
            pygame.draw.circle(self.screen, COLORS["border"], (int(robot.target[0]), int(robot.target[1])), 8, 2)

    def draw_operation_feedback(self):
        """绘制最后一次操作的反馈文本"""
        if not self.canvas.last_operation:
            return
        
        # 根据操作结果选择文本颜色
        if "成功" in self.canvas.last_operation or "重置" in self.canvas.last_operation or "已设置" in self.canvas.last_operation or "已瞬移" in self.canvas.last_operation or "已暂停" in self.canvas.last_operation or "已继续" in self.canvas.last_operation:
            text_color = COLORS["text_success"]
        else:
            text_color = COLORS["text_error"]
        
        # 绘制反馈文本（半透明背景）
        feedback_surface = self.font_small.render(self.canvas.last_operation, True, text_color)
        feedback_bg = pygame.Surface((feedback_surface.get_width() + 10, feedback_surface.get_height() + 6), pygame.SRCALPHA)
        feedback_bg.fill((255, 255, 255, 180))
        self.screen.blit(feedback_bg, (self.width - feedback_bg.get_width() - 20, self.height - feedback_bg.get_height() - 20))
        self.screen.blit(feedback_surface, (self.width - feedback_surface.get_width() - 15, self.height - feedback_surface.get_height() - 17))

    def draw_ui_panel(self):
        """绘制信息面板"""
        # 绘制半透明面板
        self.screen.blit(self.panel_surface, (10, 10))

        elapsed_time = int(time.time() - start_time)
        # 状态信息
        time_text = self.font_medium.render(f"运行总时间: {elapsed_time}s", True, COLORS["text_primary"])
        collision_text = self.font_medium.render(f"碰撞次数: {self.canvas.robot.collision_count if self.canvas.robot else 0}", True, COLORS["text_primary"])
        
        # 优化状态文本颜色：暂停为红色，到达目标为绿色，运行中为黑色
        if self.canvas.robot and self.canvas.robot.success:
            status_text = self.font_medium.render("状态: 到达目标", True, COLORS["text_success"])
        elif self.canvas.paused:
            status_text = self.font_medium.render("状态: 暂停", True, COLORS["text_error"])
        else:
            status_text = self.font_medium.render("状态: 运行中", True, COLORS["text_primary"])

        # 控制说明
        control_title = self.font_medium.render("操作说明", True, COLORS["text_primary"])
        controls = [
            "空格: 暂停/继续", "R: 重置仿真", "A: 添加静态障碍物",
            "D: 添加动态障碍物", "Z: 删除最后静态障", "X: 删除最后动态障",
            "左键: 设置目标点", "右键: 瞬移机器人", "ESC: 退出"
        ]

        # 绘制文本
        self.screen.blit(time_text, (20, 20))
        self.screen.blit(collision_text, (20, 50))
        self.screen.blit(status_text, (20, 80))
        self.screen.blit(control_title, (20, 110))

        for i, text in enumerate(controls):
            ctrl_surf = self.font_small.render(text, True, COLORS["text_secondary"])
            self.screen.blit(ctrl_surf, (20, 140 + i*18))

    def render_all(self):
        """渲染所有元素"""
        self.draw_background()
        self.draw_static_obstacles()
        self.draw_dynamic_obstacles()
        self.draw_robot()
        self.draw_ui_panel()
        self.draw_operation_feedback()

        # 刷新显示
        pygame.display.flip()

    def get_fps(self):
        """控制帧率（避免卡顿）"""
        return self.clock.tick(60)  # 60FPS

    def quit(self):
        """退出Pygame"""
        pygame.quit()