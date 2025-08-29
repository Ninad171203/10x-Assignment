#!/usr/bin/env python3
import math
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker

def clamp(v, vmin, vmax):
    return max(vmin, min(vmax, v))

def yaw_from_quat(q):
    # q: geometry_msgs/Quaternion
    # yaw (z-axis rotation)
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

class DWAPlanner(Node):
    def __init__(self):
        super().__init__('dwa_planner')

        # Params
        p = self.declare_parameters('', [
            ('goal_x', 1.5),
            ('goal_y', 0.0),
            ('max_vel_x', 0.22),
            ('min_vel_x', 0.0),
            ('max_vel_theta', 2.84),
            ('acc_lim_x', 0.25),
            ('acc_lim_theta', 3.2),
            ('sim_time', 1.0),
            ('sim_dt', 0.1),
            ('v_samples', 10),
            ('omega_samples', 20),
            ('robot_radius', 0.12),
            ('safety_margin', 0.03),
            ('heading_weight', 2.5),
            ('clearance_weight', 1.5),
            ('velocity_weight', 0.8),
            ('smoothness_weight', 0.5),
            ('control_rate_hz', 10.0),
        ])

        self.goal_x = float(self.get_parameter('goal_x').value)
        self.goal_y = float(self.get_parameter('goal_y').value)
        self.max_vel_x = float(self.get_parameter('max_vel_x').value)
        self.min_vel_x = float(self.get_parameter('min_vel_x').value)
        self.max_vel_theta = float(self.get_parameter('max_vel_theta').value)
        self.acc_lim_x = float(self.get_parameter('acc_lim_x').value)
        self.acc_lim_theta = float(self.get_parameter('acc_lim_theta').value)
        self.sim_time = float(self.get_parameter('sim_time').value)
        self.sim_dt = float(self.get_parameter('sim_dt').value)
        self.v_samples = int(self.get_parameter('v_samples').value)
        self.omega_samples = int(self.get_parameter('omega_samples').value)
        self.robot_radius = float(self.get_parameter('robot_radius').value)
        self.safety_margin = float(self.get_parameter('safety_margin').value)
        self.heading_weight = float(self.get_parameter('heading_weight').value)
        self.clearance_weight = float(self.get_parameter('clearance_weight').value)
        self.velocity_weight = float(self.get_parameter('velocity_weight').value)
        self.smoothness_weight = float(self.get_parameter('smoothness_weight').value)
        self.control_rate_hz = float(self.get_parameter('control_rate_hz').value)

        # State
        self.have_scan = False
        self.have_odom = False
        self.scan = None
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.pose_yaw = 0.0
        self.curr_v = 0.0
        self.curr_w = 0.0
        self.prev_cmd = (0.0, 0.0)

        # IO
        self.sub_scan = self.create_subscription(LaserScan, 'scan', self.scan_cb, 10)
        self.sub_odom = self.create_subscription(Odometry, 'odom', self.odom_cb, 10)
        self.sub_goal = self.create_subscription(PoseStamped, 'goal_pose', self.goal_cb, 10)
        self.pub_cmd = self.create_publisher(Twist, 'cmd_vel', 10)
        self.pub_marker = self.create_publisher(Marker, 'dwa/best_trajectory', 10)

        self.timer = self.create_timer(1.0/self.control_rate_hz, self.plan_once)
        self.get_logger().info('DWA planner started. Set goal via params or publish to /goal_pose (PoseStamped).')

    # -------- Callbacks --------
    def scan_cb(self, msg: LaserScan):
        self.scan = msg
        self.have_scan = True

    def odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.pose_x = p.x
        self.pose_y = p.y
        self.pose_yaw = yaw_from_quat(q)

        self.curr_v = msg.twist.twist.linear.x
        self.curr_w = msg.twist.twist.angular.z
        self.have_odom = True

    def goal_cb(self, msg: PoseStamped):
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        self.get_logger().info(f'New goal set: ({self.goal_x:.2f}, {self.goal_y:.2f})')

    # -------- Planning --------
    def plan_once(self):
        if not (self.have_scan and self.have_odom):
            return

        # Dynamic window around current velocity
        dt = 1.0/self.control_rate_hz
        v_min = clamp(self.curr_v - self.acc_lim_x*dt, self.min_vel_x, self.max_vel_x)
        v_max = clamp(self.curr_v + self.acc_lim_x*dt, self.min_vel_x, self.max_vel_x)
        w_min = clamp(self.curr_w - self.acc_lim_theta*dt, -self.max_vel_theta, self.max_vel_theta)
        w_max = clamp(self.curr_w + self.acc_lim_theta*dt, -self.max_vel_theta, self.max_vel_theta)

        best = None  # (score, v, w, trajectory_pts)

        # Goal in robot frame (so we can compare with predicted local trajectories)
        dx = self.goal_x - self.pose_x
        dy = self.goal_y - self.pose_y
        cos_y = math.cos(-self.pose_yaw)
        sin_y = math.sin(-self.pose_yaw)
        goal_rx = cos_y*dx - sin_y*dy
        goal_ry = sin_y*dx + cos_y*dy

        for i in range(self.v_samples):
            v = v_min + (v_max - v_min) * (i / max(1, self.v_samples-1))
            for j in range(self.omega_samples):
                w = w_min + (w_max - w_min) * (j / max(1, self.omega_samples-1))
                traj, collision, min_clear = self.rollout(v, w)
                if collision:
                    continue

                # Scores
                end_x, end_y, end_th = traj[-1]
                dist_goal = math.hypot(goal_rx - end_x, goal_ry - end_y)
                heading_score = 1.0 / (1.0 + dist_goal) # smaller distance -> higher score
                max_considered = max(self.scan.range_max, 3.0)
                clearance_score = clamp((min_clear - (self.robot_radius + self.safety_margin)) / (max_considered), 0.0, 1.0)
                velocity_score = v / max(1e-3, self.max_vel_x)
                smoothness_score = 1.0 / (1.0 + abs(w))

                score = (self.heading_weight * heading_score
                         + self.clearance_weight * clearance_score
                         + self.velocity_weight * velocity_score
                         + self.smoothness_weight * smoothness_score)

                if (best is None) or (score > best[0]):
                    best = (score, v, w, traj)

        cmd = Twist()
        if best is None:
            # No valid trajectory, stop
            self.get_logger().warn('No valid trajectory! Stopping.')
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.pub_cmd.publish(cmd)
            self.publish_marker([])
            return

        score, v, w, traj = best
        cmd.linear.x = v
        cmd.angular.z = w
        self.pub_cmd.publish(cmd)
        self.prev_cmd = (v, w)

        # Visualize best trajectory (in base_link frame)
        self.publish_marker(traj)

        # Arrived check (stop if very close)
        if math.hypot(goal_rx, goal_ry) < 0.15:
            stop = Twist()
            self.pub_cmd.publish(stop)

    def rollout(self, v: float, w: float) -> Tuple[List[Tuple[float,float,float]], bool, float]:
        """
        Simulate a short trajectory starting from robot frame (0,0,0).
        Return: (list of (x,y,theta)), collision?, min_clearance
        """
        x = 0.0
        y = 0.0
        th = 0.0
        t = 0.0
        pts = [(x, y, th)]
        min_clear = float('inf')

        while t < self.sim_time:
            # Unicycle model
            x += v * math.cos(th) * self.sim_dt
            y += v * math.sin(th) * self.sim_dt
            th += w * self.sim_dt
            pts.append((x, y, th))

            # Collision check vs LaserScan (robot frame)
            r = math.hypot(x, y)
            phi = math.atan2(y, x)
            if self.scan is not None:
                rng = self.ray_at(phi)
                if math.isfinite(rng):
                    clear = rng - self.robot_radius
                else:
                    clear = self.scan.range_max - self.robot_radius
                min_clear = min(min_clear, clear)
                if r + self.safety_margin >= rng - self.robot_radius:
                    return pts, True, min_clear

            t += self.sim_dt

        if min_clear == float('inf'):
            min_clear = self.scan.range_max if self.scan is not None else 1.0
        return pts, False, min_clear

    def ray_at(self, angle: float) -> float:
        # Clamp into scan range
        if self.scan is None:
            return float('inf')
        a_min = self.scan.angle_min
        a_max = self.scan.angle_max
        aincr = self.scan.angle_increment
        # Normalize angle to [a_min, a_max]
        while angle < a_min:
            angle += 2.0*math.pi
        while angle > a_max:
            angle -= 2.0*math.pi
        idx = int((angle - a_min) / aincr)
        idx = max(0, min(idx, len(self.scan.ranges)-1))
        rng = self.scan.ranges[idx]
        if rng == 0.0 or math.isnan(rng):
            rng = float('inf')
        return rng

    def publish_marker(self, traj: List[Tuple[float,float,float]]):
        m = Marker()
        m.header.frame_id = 'base_link'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'dwa'
        m.id = 1
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.scale.x = 0.02  # line width
        # Default RViz color (we do not set a specific color as per tool rules; leave defaults)
        m.pose.orientation.w = 1.0

        m.points = []
        from geometry_msgs.msg import Point
        for x,y,th in traj:
            p = Point()
            p.x = float(x); p.y = float(y); p.z = 0.0
            m.points.append(p)

        self.pub_marker.publish(m)

def main():
    rclpy.init()
    node = DWAPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
