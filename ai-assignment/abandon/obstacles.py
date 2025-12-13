'''
Author: Nagisa 2964793117@qq.com
Date: 2025-12-10 21:08:08
LastEditors: Nagisa 2964793117@qq.com
LastEditTime: 2025-12-10 21:23:32
FilePath: \vscode_pro\ai-assignment\obstacles.py
Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
'''
import math
import random

class BaseObstacle:
    """障碍物基类，定义通用属性和方法"""
    def __init__(self, x, y, width, height, obstacle_type):
        self.x = x  # 中心x坐标（像素）
        self.y = y  # 中心y坐标（像素）
        self.width = width  # 宽度/半径（像素）
        self.height = height  # 高度（仅矩形）
        self.type = obstacle_type  # "rectangle" / "circle"
        self.color = (random.randint(80, 200), random.randint(80, 200), random.randint(80, 200))

    def get_bounding_box(self):
        """获取碰撞检测用的包围盒"""
        if self.type == "rectangle":
            return (
                self.x - self.width/2,
                self.y - self.height/2,
                self.x + self.width/2,
                self.y + self.height/2
            )
        elif self.type == "circle":
            return (
                self.x - self.width,
                self.y - self.width,
                self.x + self.width,
                self.y + self.width
            )

    def check_collision_with(self, other_obstacle):
        """检测与另一个障碍物的碰撞"""
        # 获取自身和对方的包围盒
        self_left, self_top, self_right, self_bottom = self.get_bounding_box()
        other_left, other_top, other_right, other_bottom = other_obstacle.get_bounding_box()

        # 轴对齐包围盒初步检测
        if (self_right < other_left or self_left > other_right or
            self_bottom < other_top or self_top > other_bottom):
            return False

        # 圆形-圆形精确检测
        if self.type == "circle" and other_obstacle.type == "circle":
            dx = self.x - other_obstacle.x
            dy = self.y - other_obstacle.y
            distance = math.hypot(dx, dy)
            return distance < (self.width + other_obstacle.width)

        # 矩形-矩形精确检测
        elif self.type == "rectangle" and other_obstacle.type == "rectangle":
            return True  # 包围盒重叠即碰撞

        # 圆形-矩形精确检测
        else:
            # 确定矩形最近点
            rect = other_obstacle if other_obstacle.type == "rectangle" else self
            circle = self if other_obstacle.type == "rectangle" else other_obstacle
            
            closest_x = max(rect.x - rect.width/2, min(circle.x, rect.x + rect.width/2))
            closest_y = max(rect.y - rect.height/2, min(circle.y, rect.y + rect.height/2))
            
            dx = circle.x - closest_x
            dy = circle.y - closest_y
            return math.hypot(dx, dy) < circle.width

class StaticObstacle(BaseObstacle):
    """静态障碍物类"""
    def __init__(self, x, y, width, height, obstacle_type="rectangle"):
        super().__init__(x, y, width, height, obstacle_type)

class DynamicObstacle(BaseObstacle):
    """动态障碍物类"""
    def __init__(self, x, y, radius, movement_algorithm):
        super().__init__(x, y, radius, radius, "circle")
        self.radius = radius
        self.movement_algorithm = movement_algorithm  # 注入移动算法
        self.path = []
        self.max_path_length = 50

    def move(self, canvas_width, canvas_height, static_obstacles):
        """调用算法更新位置，返回新坐标"""
        # 计算新位置
        new_x, new_y = self.movement_algorithm.calculate_next_position(
            self.x, self.y, canvas_width, canvas_height, static_obstacles
        )
        
        # 保存路径
        self.path.append((self.x, self.y))
        if len(self.path) > self.max_path_length:
            self.path.pop(0)
        
        self.x, self.y = new_x, new_y