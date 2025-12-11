import math
import random
from obstacles import StaticObstacle, DynamicObstacle


class Robot:
    """机器人类（无Pygame依赖）"""
    def __init__(self, x, y, radius, navigation_algorithm):
        self.x = x
        self.y = y
        self.radius = radius
        self.navigation_algorithm = navigation_algorithm
        self.target = None
        self.path = []
        self.collision_count = 0
        self.success = False
        self.max_path_length = 1000
        # 新增：记录上一帧速度（用于平滑）
        self.last_speed_x = 0
        self.last_speed_y = 0
        self.last_operation = ""

    def set_target(self, x, y):
        self.target = (x, y)
        self.success = False

    def teleport(self, x, y):
        """瞬移（右键脱离卡死）"""
        self.x = x
        self.y = y
        self.collision_count = 0

    def update_position(self, static_obstacles, dynamic_obstacles, canvas_width, canvas_height):
        """调用导航算法更新位置"""
        new_x, new_y = self.navigation_algorithm.calculate_next_position(
            self, self.target, static_obstacles, dynamic_obstacles, canvas_width, canvas_height
        )

        # 碰撞检测（包含安全范围）
        safe_radius = self.radius + self.navigation_algorithm.safe_radius_extra
        collision = False

        # 检测静态障碍物
        for obs in static_obstacles:
            if obs.check_collision_with(StaticObstacle(new_x, new_y, safe_radius*2, safe_radius*2, "circle")):
                collision = True
                break

        # 检测动态障碍物
        for obs in dynamic_obstacles:
            if obs.check_collision_with(StaticObstacle(new_x, new_y, safe_radius + obs.radius, safe_radius + obs.radius, "circle")):
                collision = True
                break

        if not collision:
            self.x, self.y = new_x, new_y
            # 记录路径
            self.path.append((self.x, self.y))
            if len(self.path) > self.max_path_length:
                self.path.pop(0)
        else:
            self.collision_count += 1
            self.last_operation = f"碰撞障碍物，计数{self.collision_count}"

        # 检查是否到达目标（确保math模块可用）
        if self.target and math.hypot(new_x - self.target[0], new_y - self.target[1]) < 5:
            self.success = True
            self.last_operation = "到达目标！"

class Canvas:
    """画布管理器（维护所有元素位置,无Pygame依赖）"""
    def __init__(self, width=1200, height=800):
        self.width = width
        self.height = height
        self.static_obstacles = []
        self.dynamic_obstacles = []
        self.robot = None
        self.simulation_time = 0
        self.paused = False
        self.last_operation = ""  # 记录最后一次操作结果

    def init_robot(self, navigation_algorithm):
        """初始化机器人（左下方）"""
        start_x = 50 + 15  # 半径15
        start_y = self.height - 50 - 15
        self.robot = Robot(start_x, start_y, 15, navigation_algorithm)
        # 默认目标点（右上方）
        self.robot.set_target(self.width - 50, 50)

    # -------------------------- 核心改进：像素级无重叠检测 --------------------------
    def _is_pixel_overlap(self, new_obs, existing_obs_list):
        """检测新障碍物与已有障碍物是否像素级重叠"""
        # 遍历新障碍物的所有像素点
        if new_obs.type == "rectangle":
            # 矩形障碍物：遍历所有像素
            left = int(new_obs.x - new_obs.width/2)
            top = int(new_obs.y - new_obs.height/2)
            right = int(new_obs.x + new_obs.width/2)
            bottom = int(new_obs.y + new_obs.height/2)
            # 生成矩形所有像素坐标
            new_pixels = set()
            for x in range(left, right + 1):
                for y in range(top, bottom + 1):
                    new_pixels.add((x, y))
        elif new_obs.type == "circle":
            # 圆形障碍物：遍历圆内所有像素（中点圆算法）
            radius = int(new_obs.width)
            center_x = int(new_obs.x)
            center_y = int(new_obs.y)
            new_pixels = set()
            x = 0
            y = radius
            d = 3 - 2 * radius
            while x <= y:
                # 八对称添加像素
                for dy in range(-y, y + 1):
                    if math.hypot(x, dy) <= radius:
                        new_pixels.add((center_x + x, center_y + dy))
                        new_pixels.add((center_x - x, center_y + dy))
                for dx in range(-x + 1, x):
                    if math.hypot(dx, y) <= radius:
                        new_pixels.add((center_x + dx, center_y + y))
                        new_pixels.add((center_x + dx, center_y - y))
                x += 1
                if d > 0:
                    y -= 1
                    d += 4 * (x - y) + 10
                else:
                    d += 4 * x + 6

        # 遍历已有障碍物的像素点，检测重叠
        for obs in existing_obs_list:
            if obs.type == "rectangle":
                left = int(obs.x - obs.width/2)
                top = int(obs.y - obs.height/2)
                right = int(obs.x + obs.width/2)
                bottom = int(obs.y + obs.height/2)
                for x in range(left, right + 1):
                    for y in range(top, bottom + 1):
                        if (x, y) in new_pixels:
                            return True
            elif obs.type == "circle":
                radius = int(obs.width)
                center_x = int(obs.x)
                center_y = int(obs.y)
                x = 0
                y = radius
                d = 3 - 2 * radius
                while x <= y:
                    for dy in range(-y, y + 1):
                        if math.hypot(x, dy) <= radius:
                            if (center_x + x, center_y + dy) in new_pixels or (center_x - x, center_y + dy) in new_pixels:
                                return True
                    for dx in range(-x + 1, x):
                        if math.hypot(dx, y) <= radius:
                            if (center_x + dx, center_y + y) in new_pixels or (center_x + dx, center_y - y) in new_pixels:
                                return True
                    x += 1
                    if d > 0:
                        y -= 1
                        d += 4 * (x - y) + 10
                    else:
                        d += 4 * x + 6
        return False

    def generate_random_static_obstacles(self, SO_count):
        """生成随机静态障碍物（像素级无重叠）"""
        max_retries = 100  # 最大重试次数，避免死循环
        SO_count = random.randint(SO_count[0], SO_count[1])
        print("生成静态障碍物数量：", SO_count)
        for _ in range(SO_count):
            retries = 0
            while retries < max_retries:
                retries += 1
                # 随机尺寸和位置
                width = random.randint(3, 50)
                height = random.randint(30, 100)
                x = random.randint(width//2 + 10, self.width - width//2 - 10)
                y = random.randint(height//2 + 10, self.height - height//2 - 10)
                new_obs = StaticObstacle(x, y, width, height)

                # 像素级检测与已有障碍物重叠
                if not self._is_pixel_overlap(new_obs, self.static_obstacles + self.dynamic_obstacles):
                    self.static_obstacles.append(new_obs)
                    break
            if retries >= max_retries:
                self.last_operation = f"静态障碍物生成失败：重试{max_retries}次仍无法找到无重叠位置"
                break

    def generate_random_dynamic_obstacles(self, movement_algorithm, count=4):
        """生成随机动态障碍物（像素级无重叠）"""
        max_retries = 100
        for _ in range(count):
            retries = 0
            while retries < max_retries:
                retries += 1
                radius = random.randint(12, 20)
                x = random.randint(radius + 20, self.width - radius - 20)
                y = random.randint(radius + 20, self.height - radius - 20)
                new_obs = DynamicObstacle(x, y, radius, movement_algorithm)

                # 像素级检测与已有障碍物重叠
                if not self._is_pixel_overlap(new_obs, self.static_obstacles + self.dynamic_obstacles):
                    self.dynamic_obstacles.append(new_obs)
                    break
            if retries >= max_retries:
                self.last_operation = f"动态障碍物生成失败：重试{max_retries}次仍无法找到无重叠位置"
                break

    # -------------------------- 原有方法优化 --------------------------
    def add_static_obstacle(self, x, y):
        """添加静态障碍物（鼠标位置，像素级无重叠）"""
        width = random.randint(30, 60)
        height = random.randint(30, 60)
        new_obs = StaticObstacle(x, y, width, height)

        # 像素级检测与所有静态+动态障碍物重叠
        if self._is_pixel_overlap(new_obs, self.static_obstacles + self.dynamic_obstacles):
            self.last_operation = "添加静态障碍物失败：与已有障碍物像素重叠"
            return False
        else:
            self.static_obstacles.append(new_obs)
            self.last_operation = "添加静态障碍物成功"
            return True

    def add_dynamic_obstacle(self, x, y, movement_algorithm):
        """添加动态障碍物（鼠标位置，像素级无重叠）"""
        radius = random.randint(12, 18)
        new_obs = DynamicObstacle(x, y, radius, movement_algorithm)

        # 像素级检测与所有静态+动态障碍物重叠
        if self._is_pixel_overlap(new_obs, self.static_obstacles + self.dynamic_obstacles):
            self.last_operation = "添加动态障碍物失败：与已有障碍物像素重叠"
            return False
        else:
            self.dynamic_obstacles.append(new_obs)
            self.last_operation = "添加动态障碍物成功"
            return True

    def remove_last_obstacle(self, obstacle_type):
        """移除最后添加的障碍物"""
        if obstacle_type == "static":
            if self.static_obstacles:
                self.static_obstacles.pop()
                self.last_operation = "删除最后一个静态障碍物成功"
                return True
            else:
                self.last_operation = "删除静态障碍物失败：无静态障碍物可删"
        elif obstacle_type == "dynamic":
            if self.dynamic_obstacles:
                self.dynamic_obstacles.pop()
                self.last_operation = "删除最后一个动态障碍物成功"
                return True
            else:
                self.last_operation = "删除动态障碍物失败：无动态障碍物可删"
        return False

    def update_all(self):
        """更新所有元素位置"""
        if self.paused:
            return

        self.simulation_time += 1

        # 更新动态障碍物
        for dyn_obs in self.dynamic_obstacles:
            dyn_obs.move(self.width, self.height, self.static_obstacles)

        # 更新机器人
        if self.robot:
            self.robot.update_position(self.static_obstacles, self.dynamic_obstacles, self.width, self.height)
            # 同步机器人操作反馈到画布
            if self.robot.last_operation:
                self.last_operation = self.robot.last_operation

    def reset(self):
        """重置画布"""
        self.static_obstacles.clear()
        self.dynamic_obstacles.clear()
        self.robot = None
        self.simulation_time = 0
        self.paused = False
        self.last_operation = "仿真已重置"