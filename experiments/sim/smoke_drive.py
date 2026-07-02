#!/usr/bin/env python3
# Copyright (c) 2026 Jungmo Kang
# Licensed under the Apache License, Version 2.0.
#
# SE-MPPI loopback full drive: relocate the loopback robot to a known free,
# connected cell, command a reachable goal in the same traversable component,
# and report whether the SafeEscapeController drives it there. Coordinates come
# from offline analysis of tb3_sandbox (largest free component, eroded by the
# robot radius). Prints periodic distance feedback as runtime evidence.

import math
import os
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry, Path
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


def _quat_to_yaw(o):
    return math.atan2(2.0 * (o.w * o.z + o.x * o.y),
                      1.0 - 2.0 * (o.y * o.y + o.z * o.z))

# Free, connected, robot-safe coordinates. Defaults are for tb3_sandbox; override
# via env (SE_START_X/Y, SE_GOAL_X/Y) e.g. for the warehouse map.
START_X = float(os.environ.get('SE_START_X', -2.0))
START_Y = float(os.environ.get('SE_START_Y', -0.5))
GOAL_X = float(os.environ.get('SE_GOAL_X', -1.0))
GOAL_Y = float(os.environ.get('SE_GOAL_Y', -0.5))
# Localizer to wait on: 'bt_navigator' for loopback (no amcl), 'amcl' for Gazebo.
LOCALIZER = os.environ.get('SE_LOCALIZER', 'bt_navigator')
# Max drive time (s) before we cancel and report. Default 30 s suits the 1 m
# smoke drive of the manual playbook; the benchmark harness (RosLauncher) raises
# it to the trial watchdog via SE_DRIVE_TIMEOUT so real goals are NOT truncated
# mid-navigation (a hardcoded 30 s would log timeout/UNKNOWN for every BARN/
# HuNavSim run that legitimately takes longer).
DRIVE_TIMEOUT = float(os.environ.get('SE_DRIVE_TIMEOUT', 30.0))


def quat(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def make_goal(nav, x, y, yaw=0.0):
    p = PoseStamped()
    p.header.frame_id = 'map'
    p.header.stamp = nav.get_clock().now().to_msg()
    p.pose.position.x = x
    p.pose.position.y = y
    _, _, qz, qw = quat(yaw)
    p.pose.orientation.z = qz
    p.pose.orientation.w = qw
    return p


def main():
    rclpy.init()
    nav = BasicNavigator()

    # Relocate via /initialpose ONLY for the loopback sim, which teleports the
    # robot to match. Under Gazebo+AMCL the robot stays where it physically is,
    # so re-publishing the spawn pose on a rerun corrupts AMCL's (already
    # correct) belief. Override with SE_FORCE_RELOCATE=1 if needed.
    relocate = LOCALIZER != 'amcl' or os.environ.get('SE_FORCE_RELOCATE') == '1'
    if relocate:
        pub = nav.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)
        ip = PoseWithCovarianceStamped()
        ip.header.frame_id = 'map'
        ip.pose.pose.position.x = START_X
        ip.pose.pose.position.y = START_Y
        _, _, qz, qw = quat(0.0)
        ip.pose.pose.orientation.z = qz
        ip.pose.pose.orientation.w = qw
        nav.get_logger().info(f'Relocating robot to ({START_X}, {START_Y})...')
        for _ in range(8):
            ip.header.stamp = nav.get_clock().now().to_msg()
            pub.publish(ip)
            rclpy.spin_once(nav, timeout_sec=0.1)
            time.sleep(0.4)
    else:
        # Fresh AMCL has no belief; if we set nothing, waitUntilNav2Active()
        # publishes BasicNavigator's DEFAULT initial pose (0,0) — inside a wall
        # on tb3_sandbox — and navigation fails. amcl_pose is latched
        # (TRANSIENT_LOCAL), so on a rerun the existing belief arrives within
        # the grace window and we keep it; only a fresh launch gets seeded.
        deadline = time.time() + 3.0
        while not nav.initial_pose_received and time.time() < deadline:
            rclpy.spin_once(nav, timeout_sec=0.2)
        if nav.initial_pose_received:
            nav.get_logger().info('AMCL localizer: keeping current pose belief.')
        else:
            nav.get_logger().info(
                f'Seeding AMCL initial pose ({START_X}, {START_Y}); '
                'BasicNavigator retries until AMCL accepts it.')
            nav.setInitialPose(make_goal(nav, START_X, START_Y))

    nav.waitUntilNav2Active(localizer=LOCALIZER)
    nav.get_logger().info(f'Nav2 active. Sending goal ({GOAL_X}, {GOAL_Y}).')

    # Log the NavFn plan (first + last waypoints) to verify east routing.
    _plan_logged = [False]
    def _plan_cb(msg):
        if _plan_logged[0]:
            return
        _plan_logged[0] = True
        total = len(msg.poses)
        first = [(f'({p.pose.position.x:.2f},{p.pose.position.y:.2f})')
                 for p in msg.poses[:5]]
        last = [(f'({p.pose.position.x:.2f},{p.pose.position.y:.2f})')
                for p in msg.poses[-5:]]
        nav.get_logger().info(
            f'NAVFN_PATH total={total} first5={first} last5={last}')
    nav.create_subscription(Path, '/plan', _plan_cb, 10)

    # Wheel odometry (independent of AMCL), final cmd_vel, and raw MPPI output for diagnostic.
    _odom = [None]   # [Odometry]
    _cmdv = [None]   # [Twist]  (final: after smoother + collision_monitor)
    _nav  = [None]   # [Twist]  (raw MPPI: cmd_vel_nav before smoother; Twist not TwistStamped)
    def _odom_cb(msg): _odom[0] = msg
    def _cmdv_cb(msg): _cmdv[0] = msg
    def _nav_cb(msg):  _nav[0]  = msg
    nav.create_subscription(Odometry, '/odom', _odom_cb, 10)
    nav.create_subscription(Twist, '/cmd_vel', _cmdv_cb, 10)
    nav.create_subscription(Twist, '/cmd_vel_nav', _nav_cb, 10)

    nav.goToPose(make_goal(nav, GOAL_X, GOAL_Y))

    start = time.time()
    last = 0.0
    while not nav.isTaskComplete():
        fb = nav.getFeedback()
        now = time.time() - start
        if fb and now - last >= 0.2:
            last = now
            pos = fb.current_pose.pose.position
            amcl_yaw = _quat_to_yaw(fb.current_pose.pose.orientation)
            odom = _odom[0]
            cv = _cmdv[0]
            nv = _nav[0]
            ox = odom.pose.pose.position.x if odom else float('nan')
            oy = odom.pose.pose.position.y if odom else float('nan')
            oyaw = _quat_to_yaw(odom.pose.pose.orientation) if odom else float('nan')
            vx = cv.linear.x if cv else float('nan')
            wz = cv.angular.z if cv else float('nan')
            nvx = nv.linear.x if nv else float('nan')
            nwz = nv.angular.z if nv else float('nan')
            nav.get_logger().info(
                f'[{now:5.1f}s] dist={fb.distance_remaining:.2f}m '
                f'amcl=({pos.x:.2f},{pos.y:.2f} yaw={amcl_yaw:.2f}rad) '
                f'odom=({ox:.2f},{oy:.2f} yaw={oyaw:.2f}rad) '
                f'nav=vx:{nvx:.3f} wz:{nwz:.3f} '
                f'cmd=vx:{vx:.3f} wz:{wz:.3f}')
        if now > DRIVE_TIMEOUT:
            nav.cancelTask()
            nav.get_logger().error(f'TIMEOUT after {DRIVE_TIMEOUT:.0f} s')
            break

    result = nav.getResult()
    print(f'SMOKE_RESULT={result}')
    rclpy.shutdown()
    sys.exit(0 if result == TaskResult.SUCCEEDED else 1)


if __name__ == '__main__':
    main()
