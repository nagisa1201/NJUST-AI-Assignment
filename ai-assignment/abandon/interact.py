import pygame
from pygame.locals import *

class InteractionHandler:
    """交互处理器（映射用户输入到画布操作）"""
    def __init__(self, canvas, renderer, movement_algorithm, navigation_algorithm):
        self.canvas = canvas
        self.renderer = renderer
        self.movement_algorithm = movement_algorithm
        self.navigation_algorithm = navigation_algorithm
        self.running = True

    def handle_events(self):
        """处理所有用户输入事件（强制捕获按键，增加调试反馈）"""
        for event in pygame.event.get():
            # 退出事件
            if event.type == QUIT:
                self.running = False
                self.canvas.last_operation = "退出仿真"
            elif event.type == KEYDOWN:
                # 强制打印按键（调试用，可删除）
                key_name = pygame.key.name(event.key)
                print(f"按下按键: {key_name}")
                # 统一转为小写，兼容大小写输入
                self.handle_keydown_lower(key_name.lower(), event.key)
            elif event.type == MOUSEBUTTONDOWN:
                self.handle_mouse_click(event.button, event.pos)

    def handle_keydown_lower(self, key_name, key_code):
        """统一处理小写按键名称，确保RADZX触发"""
        # 窗口尺寸校验
        win_rect = self.renderer.screen.get_rect()
        mouse_pos = pygame.mouse.get_pos()
        mouse_in_window = win_rect.collidepoint(mouse_pos)

        # ---------- 核心：按小写名称判断，兼容所有输入方式 ----------
        if key_name == 'escape':
            self.running = False
            self.canvas.last_operation = "退出仿真"

        elif key_name == 'r':  # 匹配R/r键
            # 重置仿真（强制执行，无前置条件）
            self.canvas.reset()
            self.canvas.generate_random_static_obstacles()
            self.canvas.generate_random_dynamic_obstacles(self.movement_algorithm)
            self.canvas.init_robot(self.navigation_algorithm)
            self.canvas.last_operation = "仿真已重置（强制）"

        elif key_name == 'space':  # 空格键
            # 暂停/继续（强制切换）
            self.canvas.paused = not self.canvas.paused
            self.canvas.last_operation = "仿真已暂停" if self.canvas.paused else "仿真已继续"

        elif key_name == 'a':  # 匹配A/a键
            # 添加静态障碍物（处理鼠标在窗口外的情况）
            if not mouse_in_window:
                self.canvas.last_operation = "添加静态障碍物失败：鼠标在窗口外"
                return
            mouse_x, mouse_y = mouse_pos
            # 强制执行添加逻辑，反馈结果
            success = self.canvas.add_static_obstacle(mouse_x, mouse_y)
            if not success:
                self.canvas.last_operation = f"添加静态障碍物失败：位置({mouse_x},{mouse_y})有重叠"

        elif key_name == 'd':  # 匹配D/d键
            # 添加动态障碍物（处理鼠标在窗口外的情况）
            if not mouse_in_window:
                self.canvas.last_operation = "添加动态障碍物失败：鼠标在窗口外"
                return
            mouse_x, mouse_y = mouse_pos
            success = self.canvas.add_dynamic_obstacle(mouse_x, mouse_y, self.movement_algorithm)
            if not success:
                self.canvas.last_operation = f"添加动态障碍物失败：位置({mouse_x},{mouse_y})有重叠"

        elif key_name == 'z':  # 匹配Z/z键
            # 删除最后一个静态障碍物（强制执行，反馈结果）
            success = self.canvas.remove_last_obstacle("static")
            if not success:
                self.canvas.last_operation = "删除静态障碍物失败：无静态障碍物可删"

        elif key_name == 'x':  # 匹配X/x键
            # 删除最后一个动态障碍物（强制执行，反馈结果）
            success = self.canvas.remove_last_obstacle("dynamic")
            if not success:
                self.canvas.last_operation = "删除动态障碍物失败：无动态障碍物可删"

        # 新增：处理无效按键，避免无反馈
        else:
            self.canvas.last_operation = f"无效按键：{key_name}（仅支持R/A/D/Z/X/空格/ESC）"

    def handle_mouse_click(self, button, pos):
        """处理鼠标点击（强化反馈）"""
        x, y = pos
        if not self.canvas.robot:
            self.canvas.last_operation = "操作失败：机器人未初始化"
            return

        if button == 1:
            self.canvas.robot.set_target(x, y)
            self.canvas.last_operation = f"目标点已设置: ({x}, {y})"
        elif button == 3:
            self.canvas.robot.teleport(x, y)
            self.canvas.last_operation = f"机器人已瞬移: ({x}, {y})"

    def is_running(self):
        """返回仿真是否运行"""
        return self.running