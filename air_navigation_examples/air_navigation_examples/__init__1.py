import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
# 假设 SemanticObservation 来自某个包，这里保留原样
# from your_package.msg import SemanticObservation 
import threading
import numpy as np
import math
import time
import math
import threading
import time
import air_navigation
from sensor_msgs.msg import LaserScan
from air_simple_sim_msgs.msg import SemanticObservation
import numpy as np
from geometry_msgs.msg import PoseStamped
import csv

class EnvListener(Node):
    def __init__(self):
        super().__init__('env_listener')
        # 存储结构改为： [ {'x': x, 'y': y, 'z': z, 'time': timestamp}, ... ]
        self._detected_humans = [] 
        self.human_timeout = 0.5  # [核心参数] 0.5秒没更新就认为人走了
        self.scan_data = None
        
        self.create_subscription(
            SemanticObservation,
            '/semantic_sensor',
            self._cb_human, 10)
        
        self.create_subscription(
            LaserScan,
            '/scan',
            self._cb_scan, 10)

    def _cb_human(self, msg):
        # 1. 只要收到消息，就认为是新的观测
        if 'human' in msg.klass.lower():
            p = msg.point._point
            current_time = time.time()
            
            # 简单的策略：直接追加新观测
            # (更高级的策略是做 ID 匹配或距离匹配来更新旧点，但这里先用追加+过期清除的简单策略)
            self._detected_humans.append({
                'x': p.x, 
                'y': p.y, 
                'z': p.z, 
                'time': current_time
            })

    @property
    def humans(self):
        """
        返回当前有效的人类列表 [(x,y,z), ...]
        这里执行 '清理过期数据' 的逻辑
        """
        current_time = time.time()
        valid_humans = []
        
        # 过滤掉超时的数据 (TTL Check)
        # 也就是保留：当前时间 - 记录时间 < 超时阈值 的数据
        self._detected_humans = [
            h for h in self._detected_humans 
            if (current_time - h['time']) < self.human_timeout
        ]
        
        # 格式化输出给 Controller 使用
        for h in self._detected_humans:
            valid_humans.append((h['x'], h['y'], h['z']))
            
        return valid_humans

    def _cb_scan(self, msg):
        self.scan_data = msg

    def get_sector_distances(self, center_angle, angle_width):
        if self.scan_data is None:
            return float('inf')

        ranges = np.array(self.scan_data.ranges)
        angle_min = self.scan_data.angle_min
        angle_inc = self.scan_data.angle_increment
        
        # 保护：防止 angle_inc 为 0
        if angle_inc == 0: return float('inf')

        # 计算索引
        center_idx = int((center_angle - angle_min) / angle_inc)
        width_idx = int(angle_width / angle_inc / 2)

        n = len(ranges)
        start_idx = center_idx - width_idx
        end_idx = center_idx + width_idx + 1 # +1 确保覆盖中心

        # 处理环形索引 (Wrap around logic)
        if start_idx < 0 or end_idx > n:
            # np.r_ 是 concatenate 的简写，处理跨越 0/360 度的情况
            sector = np.r_[
                ranges[start_idx % n : n],
                ranges[0 : end_idx % n]
            ]
        else:
            sector = ranges[start_idx:end_idx]

        # 过滤无效数据 (inf, nan, 0)
        # 注意：有些雷达把 '没打到东西' 标记为 inf，有些标记为 0，视具体配置而定
        # 这里假设 >0.05 且 <inf 是有效距离
        valid = (sector > 0.05) & (sector < float('inf'))

        if np.any(valid):
            return float(np.min(sector[valid]))
        else:
            return float('inf')

    @property
    def front_dist(self):
        return float(self.get_sector_distances(0.0, math.radians(60)))

    @property
    def left_dist(self):
        return float(self.get_sector_distances(math.pi/2, math.radians(60)))

    @property
    def right_dist(self):
        return float(self.get_sector_distances(-math.pi/2, math.radians(60)))

_listener = None

def start_env_listener():
    global _listener
    if _listener is None:
        # [修复] 检查是否已经初始化，防止报错
        if not rclpy.ok():
            rclpy.init()
            
        _listener = EnvListener()
        
        # 启动线程
        t = threading.Thread(target=rclpy.spin, args=(_listener,), daemon=True)
        t.start()
        print("EnvListener started in background thread.")

# 假设 air_navigation 和相关全局变量 (如 _listener) 已经在环境中可用
# 如果是在本地测试没跑 simulation，可能需要 mock air_navigation

class TestController(air_navigation.Controller):
    def __init__(self):
        super().__init__()
        start_env_listener()  # 启用环境监听（激光+人体）
        
        # --- 导航状态与路径 ---
        self.global_plan = []
        self.current_index = 0
        self.control_interval = 0.1
        self.enable_human_aware = True

        # --- 关键参数 (可调) ---
        self.lookahead_dist = 0.8       # [改进] 前视距离 (Carrot distance)，越小贴合路径越紧，越大越平滑
        self.goal_tolerance = 0.2       # [改进] 到达终点的判定半径
        self.wall_escape_dist = 0.3     # 触发墙壁避障的距离
        self.human_escape_dist = 1.5    # 触发人体避让权重的距离
        self.human_collision_threshold = 0.55 # 碰撞统计阈值
        
        # --- FSM 状态 ---
        self.wall_state = 'CLEAR'
        self.wall_timer = 0.0
        self.escape_back_time = 0.5
        self.escape_turn_time = 1.0

        # --- 碰撞统计 ---
        self.collision_count = 0
        self.in_collision = False

        # --- [改进] 日志系统优化 ---
        # 提前打开文件，避免在循环中频繁开关文件导致 IO 阻塞
        self.log_file = 'aware_run_final.csv'
        #self.log_file = 'aware_base.csv'
        self.log_f = open(self.log_file, 'w', newline='')
        self.csv_writer = csv.writer(self.log_f)
        
        # 写入表头
        self.csv_writer.writerow([
            'timestamp', 'rx', 'ry', 'ryaw',
            'num_humans', 'closest_human_dist',
            'human_collision', 'collision_count',
            'wall_state', 
            'cmd_linear_x', 'cmd_angular_z',
            'target_x', 'target_y', 'target_dist'
        ])

    def cleanup(self):
        # [改进] 确保程序结束时关闭文件句柄
        if hasattr(self, 'log_f') and self.log_f:
            self.log_f.close()
        print("TestController cleanup")

    # --- 辅助函数 ---
    
    def _normalize_angle(self, angle):
        """ [改进] 将角度归一化到 [-pi, pi] """
        return math.atan2(math.sin(angle), math.cos(angle))

    def _get_robot_pose(self, pose):
        """ 解析机器人位置 """
        if hasattr(pose, 'pose'):
            x = pose.pose.position.x
            y = pose.pose.position.y
            q = pose.pose.orientation
        else:
            x = pose.x
            y = pose.y
            q = getattr(pose, 'orientation', None)

        if q:
            yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y**2 + q.z**2))
        else:
            yaw = 0.0
        return x, y, yaw

    def _get_local_goal(self, rx, ry):
        """ 
        [改进 - 核心逻辑] Carrot Following
        在全局路径上找到前方 lookahead_dist 处的点作为临时目标
        """
        if not self.global_plan:
            return None
        
        # 1. 找到路径上离机器人最近的点
        min_dist = float('inf')
        closest_idx = -1
        for i, pose in enumerate(self.global_plan):
            px = pose.pose.position.x
            py = pose.pose.position.y
            d = math.hypot(px - rx, py - ry)
            if d < min_dist:
                min_dist = d
                closest_idx = i
        
        # 2. 从最近点向后搜索，找到第一个距离大于 lookahead_dist 的点
        target_idx = closest_idx
        for i in range(closest_idx, len(self.global_plan)):
            px = pose.pose.position.x
            py = pose.pose.position.y
            d = math.hypot(px - rx, py - ry)
            if d > self.lookahead_dist:
                target_idx = i
                break
        
        # 如果没找到（说明快到终点了），就取路径终点
        if target_idx == -1 or target_idx >= len(self.global_plan): 
            target_idx = len(self.global_plan) - 1
            
        return self.global_plan[target_idx].pose.position

    def setPlan(self, path):
        self.global_plan = list(path.poses)
        self.current_index = 0

    # --- 主控制循环 ---

    def computeVelocityCommands(self, pose, velocity, goal):
        rx, ry, ryaw = self._get_robot_pose(pose)
        cmd = air_navigation.TwistStamped()

        # 1) 获取传感器数据
        # 假设 _listener 是全局变量
        front_dist = _listener.front_dist
        left_dist = _listener.left_dist
        right_dist = _listener.right_dist
        humans = _listener.humans

        # 2) [改进] 人体斥力场计算 (Vector Field)
        # 累加所有附近行人的斥力，而不仅仅是最近的一个
        human_repulsion_vector = [0.0, 0.0]
        closest_human_dist = float('inf')
        
        for hx, hy, _ in humans:
            dx = hx - rx
            dy = hy - ry
            dist = math.hypot(dx, dy)
            
            # 记录最近距离用于日志和权重计算
            if dist < closest_human_dist:
                closest_human_dist = dist
            
            # 计算斥力向量叠加
            if self.enable_human_aware and dist < 2.0:
                # 线性衰减：距离越近斥力越大，2米处为0
                strength = max(0.0, (2.0 - dist) / 2.0)
                
                # 归一化方向向量 (从人指向机器人，即斥力)
                # 注意：我们要远离人，所以向量方向应该是 -(hx-rx) 即 rx-hx
                # 这里为了方便计算混合，我们计算从机器人指向人的向量，然后在后面减去它
                if dist > 0.001:
                    ndx = dx / dist
                    ndy = dy / dist
                    # 累加：我们要远离，所以是负梯度
                    human_repulsion_vector[0] -= ndx * strength
                    human_repulsion_vector[1] -= ndy * strength

        # 3) 墙体避障 FSM (保持原有逻辑)
        if self.wall_state == 'CLEAR' and front_dist < self.wall_escape_dist:
            self.wall_state = 'ESCAPE_WALL'
            self.wall_timer = 0.0

        if self.wall_state == 'ESCAPE_WALL':
            cmd.twist.linear.x = -0.1
            cmd.twist.angular.z = 0.0
            self.wall_timer += self.control_interval
            if self.wall_timer > self.escape_back_time:
                self.wall_state = 'TURN_WALL'
                self.wall_timer = 0.0
            self._log(rx, ry, ryaw, humans, closest_human_dist, cmd)
            return cmd

        if self.wall_state == 'TURN_WALL':
            turn_dir = 1.0 if left_dist > right_dist else -1.0
            cmd.twist.linear.x = 0.0
            cmd.twist.angular.z = turn_dir * 0.5
            self.wall_timer += self.control_interval
            if self.wall_timer > self.escape_turn_time:
                self.wall_state = 'CLEAR'
                self.wall_timer = 0.0
            self._log(rx, ry, ryaw, humans, closest_human_dist, cmd)
            return cmd

        # 4) [改进] 导航与混合控制逻辑
        local_target = self._get_local_goal(rx, ry)
        
        # 默认指令
        cmd.twist.linear.x = 0.0
        cmd.twist.angular.z = 0.0

        if local_target:
            # 判断是否到达最终终点
            final_target = self.global_plan[-1].pose.position
            dist_to_final = math.hypot(final_target.x - rx, final_target.y - ry)
            
            if dist_to_final < self.goal_tolerance:
                # 到达终点，停车
                self._log(rx, ry, ryaw, humans, closest_human_dist, cmd)
                return cmd

            # 计算导航向量 (Nav Vector)
            nav_dx = local_target.x - rx
            nav_dy = local_target.y - ry
            nav_dist = math.hypot(nav_dx, nav_dy)
            
            # 归一化导航向量
            if nav_dist > 0:
                nav_dx /= nav_dist
                nav_dy /= nav_dist
            
            # 计算混合权重 alpha
            # 如果人很近 (<1.5m)，开始增加斥力权重；如果非常近 (<0.5m)，斥力主导
            alpha = 0.0
            if self.enable_human_aware and closest_human_dist < 1.5:
                # 简单的线性权重：1.5m时为0，0.5m时为0.8 (保留20%导航意图防止卡死)
                alpha = (1.5 - closest_human_dist) / 1.0 
                alpha = max(0.0, min(0.8, alpha))
            
            # 融合向量
            # Final = (1-alpha) * Goal + alpha * Repulsion
            blended_dx = (1.0 - alpha) * nav_dx + alpha * human_repulsion_vector[0]
            blended_dy = (1.0 - alpha) * nav_dy + alpha * human_repulsion_vector[1]
            
            # 计算目标角度
            desired_angle = math.atan2(blended_dy, blended_dx)
            angle_err = self._normalize_angle(desired_angle - ryaw)
            
            # 计算速度 (P-controller)
            # 角速度
            cmd.twist.angular.z = 1.5 * angle_err
            
            # 线速度调整
            base_speed = 0.5
            # 转弯时减速
            if abs(angle_err) > 0.5:
                base_speed = 0.1
            # 离人近时减速
            elif closest_human_dist < 1.0:
                base_speed = 0.2
            
            # 限制最大速度不超前视距离（防止过冲）
            cmd.twist.linear.x = min(base_speed, nav_dist)

        self._log(rx, ry, ryaw, humans, closest_human_dist, cmd)
        return cmd

    def _log(self, rx, ry, ryaw, humans, closest_human_dist, cmd):
        # --- 碰撞判定 ---
        moving = (abs(cmd.twist.linear.x) > 0.05) or (abs(cmd.twist.angular.z) > 0.05)
        human_collision = 0
        
        if 0 < closest_human_dist < self.human_collision_threshold:
            human_collision = 1
            if not self.in_collision and moving:
                self.collision_count += 1
                self.in_collision = True
        else:
            self.in_collision = False

        # --- 获取目标信息用于日志 ---
        target_x = target_y = target_dist = -1
        if self.global_plan:
            final_pt = self.global_plan[-1].pose.position
            target_x = final_pt.x
            target_y = final_pt.y
            target_dist = math.hypot(target_x - rx, target_y - ry)

        # --- [改进] 写入文件 ---
        try:
            self.csv_writer.writerow([
                time.time(), rx, ry, ryaw,
                len(humans), closest_human_dist,
                human_collision, self.collision_count,
                self.wall_state,
                cmd.twist.linear.x, cmd.twist.angular.z,
                target_x, target_y, target_dist
            ])
            # 定期刷新缓冲区，防止程序崩溃丢失数据 (虽然有点损耗性能，但比每次open好得多)
            self.log_f.flush()
        except Exception as e:
            print(f"Logging error: {e}")

    # --- 生命周期接口 ---
    def configure(self, name):
        print("TestController configure")

    def activate(self):
        print("TestController activate")

    def deactivate(self):
        print("TestController deactivate")

    def setSpeedLimit(self, speed_limit, percentage):
        print("TestController setSpeedLimit", speed_limit, percentage)