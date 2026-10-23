#!/usr/bin/env python3
# Copyright (c) 2026 Jungmo Kang. Licensed under the Apache License, Version 2.0.
"""Per-trial recorder of campaign 2 (see PREREG.md).

Copy of results_pilot_rerun/recorder.py (campaign 1) with one addition: the escape
path that SE-MPPI hands to MPPI while entrapped (``<plugin>/se_escape_path``,
default ``/FollowPath/se_escape_path``) is written to ``escape_path.jsonl``,
together with the number of publishers of the topic (a ``graph`` record at 2 s,
10 s and at shutdown), so a run proves whether the topic existed and was
subscribed. The CSV columns and the diagnostics file are unchanged.


Runs next to the Nav2 stack of one trial and writes, at 10 Hz of WALL time:

    wall_s, sim_s, gt_x, gt_y, gt_z, gt_roll, gt_pitch, gt_yaw,
    odom_x, odom_y, odom_yaw, nav_vx, nav_wz, cmd_vx, cmd_wz, load1

* ``sim_s`` is the last ``/clock`` value; the real-time factor of an interval is
  (difference of sim_s) / (difference of wall_s).
* ``gt_*`` is the simulator's own pose of the robot model (ground truth, world
  frame = map frame of the U-trap testbed): the first entry of the Gazebo topic
  ``/world/testbed/dynamic_pose/info``, bridged to a ROS topic of the same name.
* ``nav_*`` is ``/cmd_vel_nav`` (controller output), ``cmd_*`` is ``/cmd_vel``
  (after the velocity smoother and the collision monitor).
* ``load1`` is the host's 1-minute load average.

``/diagnostics`` messages that carry the controller's keys are appended to a
JSON-lines file. The node uses wall time (``use_sim_time`` false) on purpose.
"""

import csv
import json
import math
import sys
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage


ESCAPE_PATH_TOPIC = '/FollowPath/se_escape_path'


def rpy(q):
    """Roll, pitch, yaw of a quaternion message."""
    sinr = 2.0 * (q.w * q.x + q.y * q.z)
    cosr = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    sinp = max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x)))
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(sinr, cosr), math.asin(sinp), math.atan2(siny, cosy)


class Recorder(Node):
    def __init__(self, csv_path, diag_path, path_log=None, path_topic=ESCAPE_PATH_TOPIC):
        super().__init__('se_rerun_recorder')
        self.path_topic = path_topic
        self.path_file = open(path_log, 'w') if path_log else None
        self.t_start = time.time()
        self.graph_done = set()
        self.sim = math.nan
        self.gt = [math.nan] * 6
        self.odom = [math.nan] * 3
        self.nav = [math.nan] * 2
        self.cmd = [math.nan] * 2
        self.csv_file = open(csv_path, 'w', newline='')
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow([
            'wall_s', 'sim_s', 'gt_x', 'gt_y', 'gt_z', 'gt_roll', 'gt_pitch',
            'gt_yaw', 'odom_x', 'odom_y', 'odom_yaw', 'nav_vx', 'nav_wz',
            'cmd_vx', 'cmd_wz', 'load1'])
        self.diag_file = open(diag_path, 'w')
        best_effort = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Clock, '/clock', self.on_clock, best_effort)
        self.create_subscription(
            TFMessage, '/world/testbed/dynamic_pose/info', self.on_gt, 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(Twist, '/cmd_vel_nav', self.on_nav, 10)
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd, 10)
        self.create_subscription(DiagnosticArray, '/diagnostics', self.on_diag, 10)
        if self.path_file:
            self.create_subscription(Path, path_topic, self.on_escape_path, 10)
        self.create_timer(0.1, self.tick)

    def graph_record(self, tag):
        if self.path_file:
            self.path_file.write(json.dumps({
                'kind': 'graph', 'tag': tag, 'wall_s': time.time(), 'sim_s': self.sim,
                'topic': self.path_topic,
                'publishers': self.count_publishers(self.path_topic)}) + '\n')
            self.path_file.flush()

    def on_escape_path(self, msg):
        p = msg.poses
        rec = {'kind': 'path', 'wall_s': time.time(), 'sim_s': self.sim,
               'frame': msg.header.frame_id, 'n_poses': len(p)}
        if p:
            a, b = p[0].pose, p[-1].pose
            rec.update({
                'first_xy': [round(a.position.x, 4), round(a.position.y, 4)],
                'last_xy': [round(b.position.x, 4), round(b.position.y, 4)],
                'last_yaw': round(rpy(b.orientation)[2], 4)})
        self.path_file.write(json.dumps(rec) + '\n')
        self.path_file.flush()

    def on_clock(self, msg):
        self.sim = msg.clock.sec + msg.clock.nanosec * 1e-9

    def on_gt(self, msg):
        # The bridge's Pose_V -> TFMessage conversion drops the entity names
        # (child_frame_id stays empty). The first pose of dynamic_pose/info is
        # the model itself, followed by its links; analyze.py checks that the
        # first sample of a trial lies on the scenario's start pose.
        if msg.transforms:
            t = msg.transforms[0]
            p = t.transform.translation
            self.gt = [p.x, p.y, p.z, *rpy(t.transform.rotation)]

    def on_odom(self, msg):
        p = msg.pose.pose.position
        self.odom = [p.x, p.y, rpy(msg.pose.pose.orientation)[2]]

    def on_nav(self, msg):
        self.nav = [msg.linear.x, msg.angular.z]

    def on_cmd(self, msg):
        self.cmd = [msg.linear.x, msg.angular.z]

    def on_diag(self, msg):
        for st in msg.status:
            keys = {kv.key: kv.value for kv in st.values}
            if 'mppi_rollout_check' not in keys:
                continue
            self.diag_file.write(json.dumps({
                'wall_s': time.time(), 'sim_s': self.sim, 'name': st.name,
                'level': int.from_bytes(st.level, 'little')
                if isinstance(st.level, bytes) else int(st.level),
                'message': st.message, 'values': keys}) + '\n')
            self.diag_file.flush()

    def tick(self):
        elapsed = time.time() - self.t_start
        for mark in (2.0, 10.0):
            if elapsed >= mark and mark not in self.graph_done:
                self.graph_done.add(mark)
                self.graph_record(f'{mark:g}s')
        with open('/proc/loadavg') as f:
            load1 = float(f.read().split()[0])
        row = [time.time(), self.sim, *self.gt, *self.odom, *self.nav, *self.cmd, load1]
        self.writer.writerow([f'{v:.4f}' for v in row])
        self.csv_file.flush()


def main():
    rclpy.init()
    node = Recorder(sys.argv[1], sys.argv[2], sys.argv[3] if len(sys.argv) > 3 else None)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        try:
            node.graph_record('shutdown')
        except Exception:        # the context may already be gone at shutdown
            pass
        node.csv_file.close()
        node.diag_file.close()
        if node.path_file:
            node.path_file.close()


if __name__ == '__main__':
    main()
