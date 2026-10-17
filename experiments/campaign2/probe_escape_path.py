#!/usr/bin/env python3
# Copyright (c) 2026 Jungmo Kang. Licensed under the Apache License, Version 2.0.
"""Shakedown probe: publish three synthetic nav_msgs/Path messages on the escape-path topic.

Used in the shakedown trial only (run_campaign2.py --block shakedown). On s0_open the
controller never enters escape, so it never publishes on ``/FollowPath/se_escape_path``.
This probe is a second publisher on the same topic name. It lets the shakedown show that
the recorder's subscription receives messages on that topic and writes them to
``escape_path.jsonl``. The messages carry ``frame_id = 'shakedown_probe'``; analyze.py
ignores such records for every statistic.
"""

import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

TOPIC = '/FollowPath/se_escape_path'
FRAME = 'shakedown_probe'


def main():
    topic = sys.argv[1] if len(sys.argv) > 1 else TOPIC
    rclpy.init()
    node = rclpy.create_node('se_shakedown_probe')
    pub = node.create_publisher(Path, topic, 1)
    # Wait until the recorder's subscription is matched (a volatile publisher drops messages
    # that it sends before the match), for 60 s at most.
    deadline = time.time() + 60.0
    while pub.get_subscription_count() < 1 and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    print(f'subscriptions matched: {pub.get_subscription_count()}', flush=True)
    time.sleep(1.0)
    for i in range(3):
        msg = Path()
        msg.header.frame_id = FRAME
        for k in range(2):
            p = PoseStamped()
            p.header.frame_id = FRAME
            p.pose.position.x = 0.1 * i + k
            p.pose.orientation.w = 1.0
            msg.poses.append(p)
        pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.1)
        time.sleep(0.5)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
