import math
import threading
import time
import rclpy
from rclpy.node import Node
import air_navigation
from sensor_msgs.msg import LaserScan
from air_simple_sim_msgs.msg import SemanticObservation
import numpy as np
from geometry_msgs.msg import PoseStamped
import csv
import os

class EnvListener(Node):
    def __init__(self):
        super().__init__('env_listener')
        self.humans = []
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
        self.humans = []  # Reset humans list
        if 'human' in msg.klass.lower():
            p = msg.point._point
            self.humans.append((p.x, p.y, p.z))

    def _cb_scan(self, msg):
        self.scan_data = msg

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
            sector = np.concatenate([
                ranges[start_idx % n:n],
                ranges[0:end_idx % n]])
        else:
            sector = ranges[start_idx:end_idx]

        valid = (sector > 0) & (sector < float('inf'))

        if np.any(valid):
            return float(np.min(sector[valid]))  # 返回单个float
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
        rclpy.init()
        _listener = EnvListener()
        threading.Thread(target=rclpy.spin, args=(_listener,), daemon=True).start()
""" 
class TestController(air_navigation.Controller):
    def __init__(self):
        super().__init__()
        start_env_listener()
        self.global_plan = []
        self.current_index = 0# 使用保护变量
        self.recovery_start_time = 0
        self.last_valid_cmd_time = time.time()
        self.failed_goals = []
        self.current_goal = None
        self._lock = threading.Lock()  # 状态修改锁

    

    def _navigate(self, pose, goal):
        cmd = air_navigation.TwistStamped()
        rx, ry, ryaw = self._get_robot_pose(pose)
        if self.global_plan:
            target = self.global_plan[-1].pose.position
            dx, dy = target.x - rx, target.y - ry
            err = math.atan2(math.sin(math.atan2(dy, dx) - ryaw),
                              math.cos(math.atan2(dy, dx) - ryaw))
            cmd.twist.linear.x = min(0.5, math.hypot(dx, dy) * 0.5)
            cmd.twist.angular.z = 1.5 * err
        return cmd

    def computeVelocityCommands(self, pose, velocity, goal):
        cmd = air_navigation.TwistStamped()
        cmd.twist.linear.x = 0.0
        cmd.twist.angular.z = 0.0

        # get robot pose
        rx, ry, ryaw = self._get_robot_pose(pose)

        # read distances
        front = float(_listener.get_sector_distances(0.0, math.radians(30)))
        left = float(_listener.get_sector_distances(math.pi/2, math.radians(30)))
        right = float(_listener.get_sector_distances(-math.pi/2, math.radians(30)))

        # init wall avoidance state machine
        if not hasattr(self, 'wall_state'):
            self.wall_state = 'CLEAR'
            self.avoid_timer = 0.0

        # transition to ESCAPE if too close
        if front < 0.25 and self.wall_state != 'ESCAPE':
            self.wall_state = 'ESCAPE'
            self.avoid_timer = 0.0

        # ESCAPE: back and turn
        if self.wall_state == 'ESCAPE':
            cmd.twist.linear.x = -0.1
            cmd.twist.angular.z = 1.0 if left > right else -1.0
            self.avoid_timer += self.control_interval
            if self.avoid_timer > 0.5:
                self.wall_state = 'AVOID'
                self.avoid_timer = 0.0
            return cmd

        # AVOID: slow forward and turn away
        if front < 0.5 or self.wall_state == 'AVOID':
            self.wall_state = 'AVOID'
            if left > right:
                cmd.twist.linear.x = 0.05
                cmd.twist.angular.z = 0.5
            else:
                cmd.twist.linear.x = 0.05
                cmd.twist.angular.z = -0.5
            self.avoid_timer += self.control_interval
            if front > 0.6 and self.avoid_timer > 1.0:
                self.wall_state = 'CLEAR'
                self.avoid_timer = 0.0
            return cmd

        # CLEAR: first do human avoidance
        # 人类避障
        if len(_listener.humans) > 0:
            human_pos = np.array([(h[0], h[1]) for h in _listener.humans])
            robot_pos = np.array([rx, ry])
            distances = np.linalg.norm(human_pos - robot_pos, axis=1)
            if np.any(distances < 1.5):
                closest_idx = np.argmin(distances)
                escape_dir = robot_pos - human_pos[closest_idx]
                escape_angle = math.atan2(escape_dir[1], escape_dir[0])
                err = math.atan2(math.sin(escape_angle - ryaw), math.cos(escape_angle - ryaw))
                cmd.twist.linear.x = 0.2
                cmd.twist.angular.z = 1.5 * err
                return cmd

        # CLEAR and no immediate wall/human threat: normal navigation
        return self._navigate(pose, goal)


    def _get_robot_pose(self, pose):
       
        if hasattr(pose, 'pose'):
            rx = pose.pose.position.x
            ry = pose.pose.position.y
            quat = pose.pose.orientation
        else:
            rx = pose.x
            ry = pose.y
            quat = getattr(pose, 'orientation', None)
        
        # 四元数转偏航角
        if quat is not None:
            ryaw = math.atan2(2*(quat.w*quat.z + quat.x*quat.y),
                            1-2*(quat.y**2 + quat.z**2))
        else:
            ryaw = 0.0
        return rx, ry, ryaw


    def configure(self, name):
        super().configure(name)

    def activate(self):
        super().activate()

    def setPlan(self, path):
        super().setPlan(path)
        self.global_plan = list(path.poses)
        self.current_index = 0
    

    def cleanup(self):
        print("TestController cleanup")

    def deactivate(self):
        print("TestController deactivate")    

    def setSpeedLimit(self, speed_limit, percentage):
        print("TestController setSpeedLimit:", speed_limit, percentage) 
""" 


import csv
import os
import time
import math

class TestController(air_navigation.Controller):
    def __init__(self):
        super().__init__()
        start_env_listener()  # 启用环境监听（激光+人体）
        self.global_plan = []
        self.current_index = 0
        self.control_interval = 0.1
        self.enable_human_aware = True

        # 墙体避障 FSM
        self.wall_state = 'CLEAR'
        self.wall_timer = 0.0

        # 人体避让 FSM
        self.human_state = 'CLEAR'
        self.human_timer = 0.0

        # 参数
        self.wall_escape_dist = 0.3
        self.human_escape_dist = 1.5    # <1.0m 启动人体脱困
        self.escape_back_time = 0.5
        self.escape_turn_time = 1.0

        # 碰撞检测参数和状态
        self.human_collision_threshold = 0.55  # 人体碰撞阈值（米）
        self.collision_count = 0
        self.in_collision = False

        # 日志文件路径
        
        self.log_file = 'aware_run2.csv'
        #self.log_file = 'baseline_run2.csv'
        if not os.path.exists(self.log_file):
            with open(self.log_file, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([
                    'timestamp', 'rx', 'ry', 'ryaw',
                    'num_humans', 'closest_human_dist',
                    'human_collision', 'collision_count',
                    'wall_state', 'human_state',
                    'cmd_linear_x', 'cmd_angular_z',
                    'target_x', 'target_y', 'target_dist'
                ])

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

    def setPlan(self, path):
        self.global_plan = list(path.poses)  # 存储全局路径
        self.current_index = 0

    def computeVelocityCommands(self, pose, velocity, goal):
        rx, ry, ryaw = self._get_robot_pose(pose)
        cmd = air_navigation.TwistStamped()

        # 1) 获取传感器数据
        front_dist = _listener.front_dist
        left_dist = _listener.left_dist
        right_dist = _listener.right_dist
        humans = _listener.humans


        # 2) 人体检测和分析 —— 始终计算最近距离；仅在启用时计算斥力/威胁
        human_threat = False
        closest_human_dist = float('inf')
        closest_human_angle = 0.0
        human_repulsion_vector = [0.0, 0.0]

        # 注意：EnvListener 里第三项是 z（高度），不是速度
        for hx, hy, _ in humans:
            dx = hx - rx
            dy = hy - ry
            distance = math.hypot(dx, dy)

            if distance < closest_human_dist:
                closest_human_dist = distance
                closest_human_angle = math.atan2(dy, dx)

            # 只有在启用 human-aware 时才构建斥力与触发威胁
            if self.enable_human_aware and distance < 2.0:
                repulsion_strength = min(1.0, (2.0 - distance) / 2.0)
                normalized_dx = dx / (distance + 1e-3)
                normalized_dy = dy / (distance + 1e-3)
                human_repulsion_vector[0] -= normalized_dx * repulsion_strength
                human_repulsion_vector[1] -= normalized_dy * repulsion_strength
                if distance < self.human_escape_dist:
                    human_threat = True


        
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
        


        # 4) 改进人体避让
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
                    nav_vector = [target.x - rx, target.y - ry]

                    nav_norm = math.hypot(nav_vector[0], nav_vector[1])
                    if nav_norm > 0:
                        nav_vector = [v/nav_norm for v in nav_vector]

                    rep_norm = math.hypot(*human_repulsion_vector)
                    if rep_norm > 0:
                        human_repulsion_vector = [v/rep_norm for v in human_repulsion_vector]

                    blend_factor = min(1.0, (1.5 - closest_human_dist)/1.5)
                    blended_x = nav_vector[0]*(1-blend_factor) + human_repulsion_vector[0]*blend_factor
                    blended_y = nav_vector[1]*(1-blend_factor) + human_repulsion_vector[1]*blend_factor

                    desired_angle = math.atan2(blended_y, blended_x)
                    err = math.atan2(math.sin(desired_angle - ryaw),
                                    math.cos(desired_angle - ryaw))

                    cmd.twist.linear.x = min(0.3, nav_norm)
                    cmd.twist.angular.z = 1.5 * err
            self._log(rx, ry, ryaw, humans, closest_human_dist, cmd)
            return cmd

        # 5) 正常导航（人体感知速度调整）
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
        # 碰撞检测逻辑
        moving = (abs(cmd.twist.linear.x) > 0.05) or (abs(cmd.twist.angular.z) > 0.05)
        
        human_collision = 0
        
        if 0 < closest_human_dist < self.human_collision_threshold:
            human_collision = 1
            if not self.in_collision and moving:
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
                target_x, target_y, target_dist
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



class TrajectoryCritic(air_navigation.TrajectoryCritic): 
    LOG_PATH = "/home/leime957/Document/trajectory_logs.csv"
    def __init__(self): 
        print("[HumanAwareCritic] Init...") 
        super().__init__() 
        start_env_listener() 
        self.horizon = 1.5 # Prediction horizon in seconds
        self.risk_radius = 0.5 # High risk radius 


        # 在内存中暂存上次打分结果，供 debrief 使用
        self._last_score = None
        self._last_predicted_min_dist = None

        # 如果日志文件不存在，就写入表头
        if not os.path.exists(self.LOG_PATH):
            with open(self.LOG_PATH, "w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow([
                    "timestamp",
                    "predicted_score",
                    "predicted_min_dist",
                    "actual_min_dist",
                    "near_collision"
                ])


    def onInit(self): 
        super().onInit() 
        print("[HumanAwareCritic] onInit") 
    
    def reset(self): 
        super().reset() 
        print("[HumanAwareCritic] reset") 
    
    def prepare(self, pose, vel, goal, global_plan): 
        return True 
    
    def fit_linear(self, times, vals):
        t = np.asarray(times).flatten() 
        y = np.asarray(vals).flatten() 
        t_mean = t.mean() 
        y_mean = y.mean() 
        num = ((t - t_mean) * (y - y_mean)).sum() 
        den = ((t - t_mean)**2).sum() 
        if den == 0: 
            return 0.0, y_mean 
        m = num / den 
        b = y_mean - m * t_mean 
        return m, b 

    def predict_humans(self): 
        preds = [] 
        for hx, hy, _ in _listener.humans:
            # Simple prediction: assume human continues current motion
            # For multiple humans, you might want to track their velocities
            preds.append((hx, hy))  # Just using current position for simplicity
        return preds

    def scoreTrajectory(self, traj) -> float: 
         # --- 跟之前代码相同，计算预测最小距离和 score ---
        print(">>> HumanAwareCritic scoring a trajectory!")  # 看看能否打印
        
        predicted = self.predict_humans()
        if not predicted:
            score = 1e3
            pred_min = float('inf')
        else:
            pred_min = float('inf')
            for p in traj.poses:
                for hx, hy in predicted:
                    d = math.hypot(hx - p.x, hy - p.y)
                    if d < pred_min:
                        pred_min = d

            if pred_min < self.risk_radius:
                score = 1e-2 + (self.risk_radius - pred_min)**2
            else:
                score = pred_min

        # 存到成员变量，debrief 时再写文件
        self._last_score = score
        self._last_predicted_min_dist = pred_min
        return score
    
    def debrief(self, vel): 
        super().debrief(vel)
        # 1) 获取实际人的位置序列
        humans = [(hx, hy) for hx, hy, _ in _listener.humans]
        # 2) 获取机器人实际轨迹点 —— 假设 listener 或父类能提供 executed_poses
        executed = getattr(self, "executed_poses", [])  

        # 3) 计算实际最小距离
        actual_min = float('inf')
        for p in executed:
            for hx, hy in humans:
                d = math.hypot(hx - p.x, hy - p.y)
                if d < actual_min:
                    actual_min = d
        if actual_min == float('inf'):
            actual_min = None

        # 4) 是否近碰
        near = (actual_min is not None) and (actual_min < self.risk_radius)

        # 5) 写入 CSV
        from datetime import datetime
        with open(self.LOG_PATH, "a", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([
                datetime.utcnow().isoformat(),
                self._last_score,
                self._last_predicted_min_dist,
                actual_min,
                int(near)
            ])
class TrajectoryGenerator(air_navigation.TrajectoryGenerator):
    def __init__(self):
        super().__init__()
        print("TrajectoryGenerator constructor")
  
    def initialize(self, name):
        print("TrajectoryGenerator initialize:", name)

    def reset(self):
        print("TrajectoryGenerator reset")

    def startNewIteration(self, current_velocity):
        print("TrajectoryGenerator startNewIteration")

    def hasMoreTwists(self):
        print("TrajectoryGenerator hasMoreTwists")
        return False

    def nextTwist(self):
        print("TrajectoryGenerator nextTwist")
        return air_navigation.Twist2D()

    def generateTrajectory(self, start_pose, start_vel, cmd_vel):
        print("TrajectoryGenerator generateTrajectory")
        return air_navigation.Trajectory2D()

    def setSpeedLimit(self, speed_limit, percentage):
        print("TrajectoryGenerator setSpeedLimit:", speed_limit, percentage)