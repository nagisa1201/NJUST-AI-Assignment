'''
Author: 万鹏 wanpeng@njust.edu.cn
Date: 2025-12-27
Description: 使用VO算法进行动态避障
'''
import pygame
import numpy as np
import math

# === 进行动态避障 ===
class VO_Avoidance:
            
    def __init__(self, robot, v_max, obstacles, path_goal, inflate_ratio, vision_range, maze_data):
        self.robot = robot
        self.obstacles = obstacles
        self.vision_range = vision_range  # 视距范围
        self.inflate_ratio = inflate_ratio  # 膨胀系数
        self.path_goal = path_goal  # 规划路径点列表
        self.path_idx = 0  # 当前目标路径点索引
        self.last_velocity = np.zeros(2)  # 历史速度惯性
        self.v_max = v_max  # 最大速度
        self.maze_data = maze_data  # 迷宫数据

    def set_speed_limit(self, v_max):
        self.v_max = float(v_max)

    def is_velocity_in_VO(self, v, VO):
        """
        判断速度 v 是否在 VO (速度障碍锥)内
        """
        rel_v = (v[0] - VO['apex'][0], v[1] - VO['apex'][1])
        left = (VO['left_leg'][0] - VO['apex'][0], VO['left_leg'][1] - VO['apex'][1])
        right = (VO['right_leg'][0] - VO['apex'][0], VO['right_leg'][1] - VO['apex'][1])
        # 叉积判断 rel_v 是否在 left 和 right 之间
        cross1 = left[0]*rel_v[1] - left[1]*rel_v[0]
        cross2 = rel_v[0]*right[1] - rel_v[1]*right[0]
        return cross1 >= 0 and cross2 >= 0

    def construct_obstacle_VO(self, robot_state, obs_state, robot_radius=15, obs_radius=10):
        """
        构造动态障碍物的VO
        """
        rx, ry = robot_state['x'], robot_state['y']
        ox, oy = obs_state.x, obs_state.y
        # 动态障碍物的速度分量
        ovx = obs_state.speed * obs_state.direction[0]
        ovy = obs_state.speed * obs_state.direction[1]
        rel_pos = (ox - rx, oy - ry) # 相对位置
        dist = math.hypot(rel_pos[0], rel_pos[1]) # 距离
        R = (robot_radius + obs_radius) * self.inflate_ratio # 安全半径，这里的膨胀系数可以调整
        # theta是安全张角
        if dist < 0.00001:
            theta = math.pi
        else:
            ratio = min(R / dist, 1.0)
            theta = math.asin(ratio)

        dir_angle = math.atan2(rel_pos[1], rel_pos[0])
        # 构建锥形
        left_angle = dir_angle - theta
        right_angle = dir_angle + theta
        left_leg = (math.cos(left_angle), math.sin(left_angle))
        right_leg = (math.cos(right_angle), math.sin(right_angle))
        apex = (ovx, ovy) # VO的顶点，这里要写速度顶点!好好看看VO原理！
        
        return {
            'apex': apex,
            'left_leg': (apex[0] + left_leg[0], apex[1] + left_leg[1]),
            'right_leg': (apex[0] + right_leg[0], apex[1] + right_leg[1])
        }

    def filter_obstacles_in_vision(self, obstacles, robot_state):
        """
        返回在视距范围内且与N7car不间隔墙壁的动态障碍物列表
        """
        rx, ry = robot_state['x'], robot_state['y']
        walls = getattr(self.maze_data, 'walls', [])
        result = []
        # 解决墙壁遮蔽物体的问题
        def segments_intersect(A, B, C, D):
            """判断线段AB和CD是否相交"""
            def cross(p1, p2, p3):
                return (p2[0] - p1[0]) * (p3[1] - p1[1]) - (p2[1] - p1[1]) * (p3[0] - p1[0])
            
            cross1 = cross(A, C, D)
            cross2 = cross(B, C, D)
            cross3 = cross(C, A, B)
            cross4 = cross(D, A, B)
            
            return cross1 * cross2 < 0 and cross3 * cross4 < 0
        
        for obs in obstacles:
            ox, oy = obs.x, obs.y
            blocked = False
            # 视距范围外的不考虑
            if math.hypot(ox - rx, oy - ry) > self.vision_range:
                continue
            # 墙壁遮挡的不考虑
            for wall in walls:
                if segments_intersect((rx, ry), (ox, oy), wall[0], wall[1]):
                    blocked = True
                    break

            if not blocked:
                result.append(obs)
        return result
    
    def visualize_velocity_space(self, screen, robot_state, VOs, num_angles=72, num_radii=20):
        """
        速度空间可视化
        """
        cx, cy = int(robot_state['x']), int(robot_state['y'])
        # 从0.2到v_max采样速度
        for r in np.linspace(0.2, self.v_max, num_radii):
            # 从0到2pi采样角度
            for angle in np.linspace(0, 2 * np.pi, num_angles, endpoint=False):
                vx = math.cos(angle) * r
                vy = math.sin(angle) * r
                v = np.array([vx, vy])

                in_vo = False
                for vo in VOs:
                    if self.is_velocity_in_VO(v, vo):
                        in_vo = True
                        break
                if in_vo:
                    base_color = (255, 0, 0) 
                else:
                    base_color = (0, 255, 0)

                color = (base_color[0], base_color[1], base_color[2], 50)
                # 将速度放大显示
                px = int(cx + vx * 8)
                py = int(cy + vy * 8)
                
                surf = pygame.Surface((4, 4), pygame.SRCALPHA)
                pygame.draw.circle(surf, color, (2, 2), 2)
                screen.blit(surf, (px - 2, py - 2))
        
    def build_VOs(self, obstacles, robot_state):
        VOs = []
        considered_obs = self.filter_obstacles_in_vision(obstacles, robot_state)
        
        for obs in considered_obs:
            vo = self.construct_obstacle_VO(robot_state, obs)
            VOs.append(vo)
        
        print(f"N7car坐标: ({robot_state['x']:.1f}, {robot_state['y']:.1f})")
        collision_flag = False
        if considered_obs:
            print("考虑进VO的动态障碍物坐标:")
            for obs in considered_obs:
                min_dist = math.hypot(obs.x - robot_state['x'], obs.y - robot_state['y'])
                print(f"障碍物{getattr(obs, 'oid', '?')}: ({obs.x:.1f}, {obs.y:.1f})，此时距离{min_dist:.1f}")
                if min_dist < 18 : # 这里18是超参数，就是两个半径相加
                    print("!!! 警告: 障碍物过近，可能发生碰撞 !!!")
                    collision_flag = True
        else:
            print("视距内无动态障碍物被考虑进VO, 当前速度为{:.1f}".format(robot_state['speed']))
        
        # 有碰撞，调用回调
        if collision_flag:
            self.collision_callback()
        return VOs
    
    def select_velocity_outside_VOs(self, v_desired, VOs, danger_away_dir=None):
        """
        速度选择函数
        """
        if not VOs:
            v_norm = np.linalg.norm(v_desired)
            if v_norm <= self.v_max:
                self.last_velocity = v_desired
                return v_desired
            v = v_desired / v_norm * self.v_max
            self.last_velocity = v
            return v
        
        # 采样 速度是从0.2到v_max，角度是0到2pi
        radii = np.linspace(0.2, self.v_max, 20)
        angles = np.linspace(0, 2 * np.pi, 72, endpoint=False)
        
        candidates = []
        loss = []
        
        # 计算归一化参数
        MAX_DISTANCE = 2 * self.v_max  # 最大可能距离
        MAX_ANGLE = np.pi  # 最大角度180度
        
        for r in radii:
            for angle in angles:
                v = np.array([np.cos(angle), np.sin(angle)]) * r
                
                # 检查是否在VO内
                in_vo = False
                for vo in VOs:
                    if self.is_velocity_in_VO(v, vo):
                        in_vo = True
                        break
                
                if not in_vo:
                    candidates.append(v)
                    
                    # 1. 距离损失（归一化到[0,1]）
                    dist_loss = np.linalg.norm(v - v_desired)
                    norm_dist_loss = min(dist_loss / MAX_DISTANCE, 1.0)
                    
                    # 2. 方向损失（归一化到[0,1]）
                    norm_v = np.linalg.norm(v)
                    norm_vd = np.linalg.norm(v_desired)
                    if norm_v > 0 and norm_vd > 0:
                        cos_dir = np.dot(v, v_desired) / (norm_v * norm_vd)
                        cos_dir = np.clip(cos_dir, -1, 1)
                        dir_angle = np.arccos(cos_dir)
                        norm_dir_loss = dir_angle / MAX_ANGLE  # 归一化
                    else:
                        norm_dir_loss = 0.5  # 零向量，取中间值
                    
                    # 3. 惯性损失
                    norm_last = np.linalg.norm(self.last_velocity)
                    if norm_v > 0 and norm_last > 0:
                        # 计算理想方向：在目标方向和惯性方向间权衡
                        v_target_dir = v_desired / norm_vd
                        v_last_dir = self.last_velocity / norm_last
                        
                        # 计算转向角度
                        turn_angle = self.angle_between(v, self.last_velocity)
                        norm_inertia_loss = turn_angle / MAX_ANGLE  # 归一化
                        
                    else:
                        norm_inertia_loss = 0.0  # 没有历史速度，不惩罚
                    
                    # 综合损失
                    total_loss = (
                        0.5 * norm_dist_loss +    # 距离损失
                        0.3 * norm_dir_loss +    # 方向损失  
                        0.2 * norm_inertia_loss   # 惯性损失
                    )
                    loss.append(total_loss)
        
        if not candidates:
            # 没有安全速度，紧急处理
            if danger_away_dir is not None:
                # 尝试慢速远离
                for speed_factor in [0.8, 0.5, 0.3, 0.1]:
                    v_emergency = danger_away_dir * (self.v_max * speed_factor)
                    safe = True
                    for vo in VOs:
                        if self.is_velocity_in_VO(v_emergency, vo):
                            safe = False
                            break
                    if safe:
                        self.last_velocity = v_emergency
                        return v_emergency
            
            # 所有尝试都失败，紧急停止
            v = np.array([0.0, 0.0])
            self.last_velocity = v
            return v
        
        # 选择损失最小的候选速度
        best_idx = np.argmin(loss)
        v_best = candidates[best_idx]
        
        # 速度限制
        v_norm = np.linalg.norm(v_best)
        if v_norm > self.v_max + 0:
            v_best = v_best / v_norm * self.v_max
        
        self.last_velocity = v_best
        return v_best

    def angle_between(self, v1, v2):
        norm1 = np.linalg.norm(v1)
        norm2 = np.linalg.norm(v2)
        
        if norm1 < 0 or norm2 < 0:
            return 0.0  
        
        cos_theta = np.dot(v1, v2) / (norm1 * norm2)
        cos_theta = np.clip(cos_theta, -1, 1)
        
        return np.arccos(cos_theta)


    def compute_new_velocity(self, target_point, current_position):
        #  计算期望方向
        target_vec = np.array(target_point) - np.array(current_position)
        distance = np.linalg.norm(target_vec)
        
        if distance < 8:
            # 已到达目标
            v_desired = np.array([0.0, 0.0])
        else:

            v_desired = (target_vec / distance) * self.v_max
        
        # 获取状态
        robot_state = self.robot.get_N7car_state()
        robot_pos = np.array([robot_state['x'], robot_state['y']])
        robot_radius = self.robot.radius
        
        # 检测危险
        danger_away_dir = None
        min_safe_dist_sq = float('inf')
        
        for obs in self.obstacles:
            obs_pos = np.array([obs.x, obs.y])
            dist_vec = robot_pos - obs_pos
            dist_sq = np.dot(dist_vec, dist_vec)
            
            # 安全距离的平方
            safe_dist = robot_radius + obs.radius + 5
            safe_dist_sq = safe_dist * safe_dist
            
            if dist_sq < safe_dist_sq and dist_sq < min_safe_dist_sq:
                min_safe_dist_sq = dist_sq
                # 计算远离方向
                if dist_sq > 0:
                    danger_away_dir = dist_vec / np.sqrt(dist_sq)
                else:
                    danger_away_dir = np.array([1.0, 0.0])  # 默认方向
        
        # 4. 构造VO
        VOs = self.build_VOs(self.obstacles, robot_state)
        
        # 5. 选择速度
        return self.select_velocity_outside_VOs(v_desired, VOs, danger_away_dir)
    
    def update(self):
        robot_state = self.robot.get_N7car_state()
        current_position = (robot_state['x'], robot_state['y'])
        # 遍历当前子列表所有点，筛选VO外的安全点，选与下一个子列表中心点欧氏距离最小的点作为目标点
        # 没目标点了就直接返回
        if not self.path_goal or self.path_idx >= len(self.path_goal):
            return
        current_segment = self.path_goal[self.path_idx]
        # 计算当前VO遮蔽区
        robot_state = self.robot.get_N7car_state()
        VOs = self.build_VOs(self.obstacles, robot_state)
        # 获取下一个子列表的中心点 作为参考点
        def get_center_point(segment):
            if not segment:
                return None
            arr = np.array(segment)
            center = np.mean(arr, axis=0)
            # 找到距离均值最近的点
            return min(segment, key=lambda p: np.linalg.norm(np.array(p) - center))
        
        if self.path_idx + 1 < len(self.path_goal):
            next_segment = self.path_goal[self.path_idx + 1]
            ref_point = get_center_point(next_segment)
        else:
            ref_point = get_center_point(current_segment)

        # 筛选VO外的安全点
        safe_points = []
        for pt in current_segment:
            v = np.array(pt) - np.array([robot_state['x'], robot_state['y']])
            # 判断该点对应的速度是否在所有VO外
            if not any(self.is_velocity_in_VO(v, vo) for vo in VOs):
                safe_points.append(pt)

        # 如果没有安全点，降级为所有点都可选
        if not safe_points:
            safe_points = current_segment
        # 选择与参考点欧氏距离最小的安全点
        closest_point = safe_points[0]  # 初始化为第一个点
        closest_distance = np.linalg.norm(np.array(safe_points[0]) - np.array(ref_point))

        for point in safe_points[1:]:  # 从第二个点开始比较
            distance = np.linalg.norm(np.array(point) - np.array(ref_point))
            if distance < closest_distance:
                closest_distance = distance
                closest_point = point

        target_point = closest_point

        # 判断是否到达当前目标点
        if np.linalg.norm(np.array(current_position) - np.array(target_point)) < 10 :
            self.path_idx += 1
            if self.path_idx >= len(self.path_goal):
                self.robot.speed = 0
                return
            # 进入下一个子列表，重新筛选目标点
            current_segment = self.path_goal[self.path_idx]
            # 重新计算VO和参考点
            robot_state = self.robot.get_N7car_state()
            VOs = self.build_VOs(self.obstacles, robot_state)

            if self.path_idx + 1 < len(self.path_goal):
                next_segment = self.path_goal[self.path_idx + 1]
                if len(next_segment) >= 5:
                    ref_point = next_segment[4]
                else:
                    ref_point = next_segment[len(next_segment)//2]
            else:
                if len(current_segment) >= 5:
                    ref_point = current_segment[4]
                else:
                    ref_point = current_segment[len(current_segment)//2]

            safe_points = []

            for pt in current_segment:
                v = np.array(pt) - np.array([robot_state['x'], robot_state['y']])
                if not any(self.is_velocity_in_VO(v, vo) for vo in VOs):
                    safe_points.append(pt)

            if not safe_points:
                safe_points = current_segment

            closest_distance = float('inf')  # 初始化一个很大的数

            for point in safe_points:
                # 计算这个点到参考点的距离
                distance = np.linalg.norm(np.array(point) - np.array(ref_point))
                
                # 如果这个距离比之前找到的最小距离还小
                if distance < closest_distance:
                    closest_distance = distance
                    target_point = point

            self.target_point = target_point
            robot_state = self.robot.get_N7car_state()
            VOs = self.build_VOs(self.obstacles, robot_state)
            if hasattr(self, 'scene_renderer') and self.scene_renderer is not None:
                # 速度空间安全区可视化
                self.visualize_velocity_space(self.scene_renderer.screen, robot_state, VOs)
                self.target_point = target_point
            print("N7car目标点:", self.target_point)

        new_velocity = self.compute_new_velocity(target_point, current_position)
        speed = np.linalg.norm(new_velocity)
        if speed > 0:
            direction = new_velocity / speed
        else:
            direction = (0,0)
        self.robot.speed = speed
        self.robot.direction = direction
        self.last_velocity = new_velocity