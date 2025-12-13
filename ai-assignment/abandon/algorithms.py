import math
import random
import numpy as np
from obstacles import BaseObstacle, StaticObstacle, DynamicObstacle

class DynamicMovementAlgorithm:
    """动态障碍物移动算法基类"""
    def calculate_next_position(self, current_x, current_y, canvas_width, canvas_height, static_obstacles):
        raise NotImplementedError("子类必须实现该方法")

class GaussianRandomMovement(DynamicMovementAlgorithm):
    """高斯随机移动算法（默认）"""
    def __init__(self, base_speed=2, sigma=0.5):
        self.base_speed = base_speed  # 基础移动速度
        self.sigma = sigma  # 高斯分布标准差
        self.current_direction = random.uniform(0, 2 * math.pi)  # 初始方向

    def calculate_next_position(self, current_x, current_y, canvas_width, canvas_height, static_obstacles):
        # 高斯随机调整方向
        direction_delta = np.random.normal(0, self.sigma)
        self.current_direction += direction_delta
        self.current_direction %= 2 * math.pi  # 保持0-2π

        # 计算新位置
        new_x = current_x + math.cos(self.current_direction) * self.base_speed
        new_y = current_y + math.sin(self.current_direction) * self.base_speed

        # 边界约束
        new_x = max(self.base_speed, min(canvas_width - self.base_speed, new_x))
        new_y = max(self.base_speed, min(canvas_height - self.base_speed, new_y))

        # 静态障碍物碰撞回避
        for obs in static_obstacles:
            if obs.check_collision_with(
                BaseObstacle(new_x, new_y, self.base_speed, self.base_speed, "circle")
            ):
                self.current_direction += math.pi/2  # 转向90度
                new_x = current_x + math.cos(self.current_direction) * self.base_speed
                new_y = current_y + math.sin(self.current_direction) * self.base_speed
                break

        return new_x, new_y

class RobotNavigationAlgorithm:
    """机器人导航算法基类"""
    def __init__(self, safe_radius_extra=20):
        self.safe_radius_extra = safe_radius_extra  # 安全范围额外像素
        self.last_target_distance = float('inf')  # 记录上一次到目标的距离
        self.stuck_count = 0  # 卡死计数
        self.stuck_threshold = 30  # 超过30帧未移动判定为卡死
        self.escape_strength = 1.5  # 逃逸力度

    def calculate_next_position(self, robot, target, static_obstacles, dynamic_obstacles, canvas_width, canvas_height):
        raise NotImplementedError("子类必须实现该方法")

class ImprovedPotentialFieldNavigation(RobotNavigationAlgorithm):
    """改进版势场法导航避障算法"""
    def __init__(self, max_speed=3, repulsion_strength=6, safe_distance=70, attract_decay=0.8):
        super().__init__(safe_radius_extra=20)
        self.max_speed = max_speed
        self.repulsion_strength = repulsion_strength  # 降低斥力强度，避免覆盖引力
        self.safe_distance = safe_distance
        self.attract_decay = attract_decay  # 引力衰减系数
        self.smoothing_factor = 0.2  # 速度平滑因子

    def calculate_next_position(self, robot, target, static_obstacles, dynamic_obstacles, canvas_width, canvas_height):
        if not target:
            return robot.x, robot.y

        # 1. 基础参数计算
        dx_target = target[0] - robot.x
        dy_target = target[1] - robot.y
        distance_target = math.hypot(dx_target, dy_target)

        # 到达目标判定
        if distance_target < 5:
            self.stuck_count = 0
            self.last_target_distance = distance_target
            return robot.x, robot.y

        # 2. 检测局部最优（卡死）：连续多帧未向目标移动
        if distance_target >= self.last_target_distance - 1:  # 允许1像素误差
            self.stuck_count += 1
        else:
            self.stuck_count = 0
        self.last_target_distance = distance_target

        # 3. 目标引力（随距离衰减，避免远距引力过大）
        attract_strength = min(self.max_speed, distance_target * self.attract_decay / 100)
        attract_x = (dx_target / distance_target) * attract_strength
        attract_y = (dy_target / distance_target) * attract_strength

        # 4. 障碍物斥力（仅在安全距离内生效，优化动态障碍物预判）
        repel_x, repel_y = 0, 0

        # 4.1 静态障碍物斥力
        for obs in static_obstacles:
            obs_left, obs_top, obs_right, obs_bottom = obs.get_bounding_box()
            closest_x = max(obs_left, min(robot.x, obs_right))
            closest_y = max(obs_top, min(robot.y, obs_bottom))
            
            dx_repel = robot.x - closest_x
            dy_repel = robot.y - closest_y
            distance_repel = math.hypot(dx_repel, dy_repel) - self.safe_radius_extra

            if 0 < distance_repel < self.safe_distance:
                # 斥力随距离衰减：距离越近，斥力越大
                repulsion = self.repulsion_strength * (1/distance_repel - 1/self.safe_distance)
                repel_x += (dx_repel / distance_repel) * repulsion
                repel_y += (dy_repel / distance_repel) * repulsion

        # 4.2 动态障碍物斥力（预判移动方向）
        for obs in dynamic_obstacles:
            # 计算动态障碍物下一步位置
            pred_x, pred_y = obs.movement_algorithm.calculate_next_position(
                obs.x, obs.y, canvas_width, canvas_height, static_obstacles
            )
            # 计算机器人到预判位置的距离
            dx_repel = robot.x - pred_x
            dy_repel = robot.y - pred_y
            distance_repel = math.hypot(dx_repel, dy_repel) - (self.safe_radius_extra + obs.radius)

            if 0 < distance_repel < self.safe_distance + 10:  # 扩大预判范围
                repulsion = self.repulsion_strength * 1.2 * (1/distance_repel - 1/(self.safe_distance + 10))
                repel_x += (dx_repel / distance_repel) * repulsion
                repel_y += (dy_repel / distance_repel) * repulsion

        # 5. 局部最优逃逸机制
        if self.stuck_count > self.stuck_threshold:
            # 随机生成逃逸方向
            escape_dir = random.uniform(0, 2 * math.pi)
            repel_x += math.cos(escape_dir) * self.escape_strength
            repel_y += math.sin(escape_dir) * self.escape_strength
            self.stuck_count = 0  # 重置卡死计数
            robot.last_operation = "触发局部最优逃逸"

        # 6. 合成速度向量 + 平滑处理
        total_x = attract_x + repel_x
        total_y = attract_y + repel_y

        # 速度平滑：与上一帧速度插值，减少抖动
        if hasattr(robot, 'last_speed_x'):
            total_x = (1 - self.smoothing_factor) * robot.last_speed_x + self.smoothing_factor * total_x
            total_y = (1 - self.smoothing_factor) * robot.last_speed_y + self.smoothing_factor * total_y
        # 记录当前速度用于下一帧平滑
        robot.last_speed_x = total_x
        robot.last_speed_y = total_y

        # 7. 限制最大速度
        speed_magnitude = math.hypot(total_x, total_y)
        if speed_magnitude > self.max_speed:
            scale = self.max_speed / speed_magnitude
            total_x *= scale
            total_y *= scale

        # 8. 边界约束
        new_x = robot.x + total_x
        new_y = robot.y + total_y
        new_x = max(robot.radius + self.safe_radius_extra, min(canvas_width - robot.radius - self.safe_radius_extra, new_x))
        new_y = max(robot.radius + self.safe_radius_extra, min(canvas_height - robot.radius - self.safe_radius_extra, new_y))

        return new_x, new_y