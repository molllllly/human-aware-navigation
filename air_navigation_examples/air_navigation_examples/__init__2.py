import math
import threading
import time
import rclpy
from rclpy.node import Node
import air_navigation
from sensor_msgs.msg import LaserScan
from air_simple_sim_msgs.msg import SemanticObservation
import numpy as np
import csv
import os

# -----------------------------
# EnvListener：带 TTL 的人体缓存（加锁 + 更短TTL）
# -----------------------------
class EnvListener(Node):
    def __init__(self):
        super().__init__('env_listener')

        # 缓存最近 ttl 秒的人体观测：(x, y, z, t)
        self._humans_buf = []
        self.ttl = 0.25  # 原来 0.5，建议 0.2~0.3 更稳/更公平

        # 并发锁：spin 线程写 humans_buf，controller 线程读/清理
        self._lock = threading.Lock()

        self.scan_data = None
        self.create_subscription(SemanticObservation, '/semantic_sensor', self._cb_human, 10)
        self.create_subscription(LaserScan, '/scan', self._cb_scan, 10)

    def _cb_human(self, msg):
        now = self.get_clock().now().nanoseconds * 1e-9
        if hasattr(msg, 'klass') and ('human' in msg.klass.lower()):
            p = msg.point._point
            with self._lock:
                self._humans_buf.append((p.x, p.y, p.z, now))

    def _cb_scan(self, msg):
        self.scan_data = msg

    @property
    def humans(self):
        """返回最近 ttl 秒内的人体列表 [(x,y,z), ...]"""
        now = self.get_clock().now().nanoseconds * 1e-9
        with self._lock:
            # 清理过期观测
            self._humans_buf = [
                (x, y, z, t) for (x, y, z, t) in self._humans_buf
                if (now - t) <= self.ttl
            ]
            return [(x, y, z) for (x, y, z, _) in self._humans_buf]

    # 激光扇区距离工具
    def get_sector_distances(self, center_angle, angle_width):
        if self.scan_data is None:
            return float('inf')

        ranges = np.array(self.scan_data.ranges)
        angle_min = self.scan_data.angle_min
        angle_inc = self.scan_data.angle_increment

        center_idx = int((center_angle - angle_min) / angle_inc)
        width_idx = int(angle_width / angle_inc / 2)

        n = len(ranges)
        start_idx = center_idx - width_idx
        end_idx = center_idx + width_idx + 1

        if start_idx < 0 or end_idx > n:
            sector = np.concatenate([ranges[start_idx % n:n], ranges[0:end_idx % n]])
        else:
            sector = ranges[start_idx:end_idx]

        valid = (sector > 0) & (sector < float('inf'))
        return float(np.min(sector[valid])) if np.any(valid) else float('inf')

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
        # 避免重复 init（ROS2 / Nav2 里经常已经 init 过）
        if not rclpy.ok():
            rclpy.init()
        _listener = EnvListener()
        threading.Thread(target=rclpy.spin, args=(_listener,), daemon=True).start()


# -----------------------------
# TestController：baseline 也记录真实最近人距；可选静止计碰撞；紧急刹停
# -----------------------------
class TestController(air_navigation.Controller):
    def __init__(self):
        super().__init__()
        start_env_listener()
        self.global_plan = []
        self.current_index = 0
        self.control_interval = 0.1

        # False=Baseline（不避让，但仍记录距离）；True=启用人感知
        self.enable_human_aware = True

        # FSM
        self.wall_state = 'CLEAR'
        self.wall_timer = 0.0
        self.human_state = 'CLEAR'
        self.human_timer = 0.0

        # 参数
        self.wall_escape_dist = 0.3
        self.human_escape_dist = 1.5
        self.escape_back_time = 0.5
        self.escape_turn_time = 1.0

        # 碰撞与运动判定参数
        self.human_collision_threshold = 0.5
        self.count_stationary_collisions = True
        self.collision_count = 0
        self.in_collision = False
        self.last_real_linear = 0.0
        self.last_real_angular = 0.0

        # 紧急刹停（仅在人感知开启时生效）
        self.emergency_stop_dist = 0.4

        # 日志
        self.log_file = 'robot_log_true.csv'
        if not os.path.exists(self.log_file):
            with open(self.log_file, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([
                    'timestamp', 'rx', 'ry', 'ryaw',
                    'num_humans', 'closest_human_dist',
                    'human_collision', 'collision_count',
                    'wall_state', 'human_state',
                    'cmd_linear_x', 'cmd_angular_z',
                    'target_x', 'target_y', 'target_dist',
                    'moving', 'enable_human_aware'
                ])

        # 建议：启动时打印一下，防止跑错实验组
        print("[TestController] enable_human_aware =", self.enable_human_aware)

    def _get_robot_pose(self, pose):
        if hasattr(pose, 'pose'):
            x = pose.pose.position.x
            y = pose.pose.position.y
            q = pose.pose.orientation
        else:
            x = pose.x
            y = pose.y
            q = getattr(pose, 'orientation', None)

        if q:
            yaw = math.atan2(2*(q.w*q.z + q.x*q.y),
                             1-2*(q.y**2 + q.z**2))
        else:
            yaw = 0.0
        return x, y, yaw

    def _get_real_velocity(self, velocity):
        try:
            tw = getattr(velocity, 'twist', velocity)
            lx = float(getattr(getattr(tw, 'linear', tw), 'x', 0.0))
            az = float(getattr(getattr(tw, 'angular', tw), 'z', 0.0))
        except Exception:
            lx, az = 0.0, 0.0
        return abs(lx), abs(az)

    def setPlan(self, path):
        self.global_plan = list(path.poses)
        self.current_index = 0

    def computeVelocityCommands(self, pose, velocity, goal):
        rx, ry, ryaw = self._get_robot_pose(pose)
        self.last_real_linear, self.last_real_angular = self._get_real_velocity(velocity)

        cmd = air_navigation.TwistStamped()

        # 1) 传感器
        front_dist = _listener.front_dist
        left_dist  = _listener.left_dist
        right_dist = _listener.right_dist
        humans     = _listener.humans

        # 2) 始终计算最近人距；仅在启用人感知时构建斥力/威胁
        human_threat = False
        closest_human_dist = float('inf')
        closest_human_angle = 0.0
        human_repulsion_vector = [0.0, 0.0]

        for hx, hy, _ in humans:
            dx = hx - rx
            dy = hy - ry
            d  = math.hypot(dx, dy)

            if d < closest_human_dist:
                closest_human_dist = d
                closest_human_angle = math.atan2(dy, dx)

            if self.enable_human_aware and d < 2.0:
                rep = min(1.0, (2.0 - d) / 2.0)
                nx, ny = dx/(d+1e-3), dy/(d+1e-3)
                human_repulsion_vector[0] -= nx * rep
                human_repulsion_vector[1] -= ny * rep
                if d < self.human_escape_dist:
                    human_threat = True

        # 2.5) 紧急刹停（仅在启用人感知时）
        if self.enable_human_aware and closest_human_dist < self.emergency_stop_dist:
            cmd.twist.linear.x = 0.0
            cmd.twist.angular.z = 0.0
            self._log(rx, ry, ryaw, humans, closest_human_dist, cmd)
            return cmd

        # 3) 墙体避障 FSM
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

        # 4) 人体避让（仅启用时生效）
        if self.enable_human_aware and human_threat:
            if closest_human_dist < 0.8:
                escape_angle = closest_human_angle + math.pi
                err = math.atan2(math.sin(escape_angle - ryaw),
                                 math.cos(escape_angle - ryaw))
                cmd.twist.linear.x = -0.1
                cmd.twist.angular.z = 1.5 * err
            else:
                if self.global_plan:
                    target = self.global_plan[-1].pose.position
                    nav_vec = [target.x - rx, target.y - ry]
                    nav_norm = math.hypot(nav_vec[0], nav_vec[1])
                    if nav_norm > 0:
                        nav_vec = [v/nav_norm for v in nav_vec]

                    rep_norm = math.hypot(*human_repulsion_vector)
                    if rep_norm > 0:
                        human_repulsion_vector = [v/rep_norm for v in human_repulsion_vector]

                    blend = min(1.0, (1.5 - closest_human_dist)/1.5)
                    bx = nav_vec[0]*(1-blend) + human_repulsion_vector[0]*blend
                    by = nav_vec[1]*(1-blend) + human_repulsion_vector[1]*blend

                    desired = math.atan2(by, bx)
                    err = math.atan2(math.sin(desired-ryaw), math.cos(desired-ryaw))
                    cmd.twist.linear.x = min(0.3, nav_norm)
                    cmd.twist.angular.z = 1.5 * err
            self._log(rx, ry, ryaw, humans, closest_human_dist, cmd)
            return cmd

        # 5) 正常导航（降速仅在人感知开启时）
        if self.global_plan:
            target = self.global_plan[-1].pose.position
            dx, dy = target.x - rx, target.y - ry
            dist = math.hypot(dx, dy)

            base_speed = 0.5
            if self.enable_human_aware and closest_human_dist < 1.5:
                base_speed = 0.3 * (closest_human_dist / 1.5)

            if dist > 0.05:
                desired = math.atan2(dy, dx)
                err = math.atan2(math.sin(desired-ryaw), math.cos(desired-ryaw))
                cmd.twist.linear.x = min(base_speed, dist)
                cmd.twist.angular.z = 1.5 * err

        self._log(rx, ry, ryaw, humans, closest_human_dist, cmd)
        return cmd

    def _log(self, rx, ry, ryaw, humans, closest_human_dist, cmd):
        moving = (self.last_real_linear > 0.02) or (self.last_real_angular > 0.02)
        should_count = moving or self.count_stationary_collisions

        human_collision = 0
        if 0 < closest_human_dist < self.human_collision_threshold:
            human_collision = 1
            if not self.in_collision and should_count:
                self.collision_count += 1
                self.in_collision = True
        else:
            self.in_collision = False

        target_x = target_y = target_dist = -1
        if self.global_plan:
            target = self.global_plan[-1].pose.position
            target_x = target.x
            target_y = target.y
            target_dist = math.hypot(target_x - rx, target_y - ry)

        with open(self.log_file, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                time.time(), rx, ry, ryaw,
                len(humans), closest_human_dist,
                human_collision, self.collision_count,
                self.wall_state, self.human_state,
                cmd.twist.linear.x, cmd.twist.angular.z,
                target_x, target_y, target_dist,
                int(moving), int(self.enable_human_aware)
            ])

    def configure(self, name):
        print("TestController configure")

    def cleanup(self):
        print("TestController cleanup")

    def activate(self):
        print("TestController activate")

    def deactivate(self):
        print("TestController deactivate")

    def setSpeedLimit(self, speed_limit, percentage):
        print("TestController setSpeedLimit", speed_limit, percentage)
