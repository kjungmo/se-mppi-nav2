"""The campaign-2 recorder writes the escape-path topic and the graph records."""

import json
import os
import time

import pytest

rclpy = pytest.importorskip('rclpy')

from geometry_msgs.msg import PoseStamped  # noqa: E402
from nav_msgs.msg import Path  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402

import recorder as rec_mod  # noqa: E402


@pytest.fixture(scope='module')
def ros():
    os.environ.setdefault('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
    os.environ['ROS_DOMAIN_ID'] = '79'
    rclpy.init()
    yield
    rclpy.shutdown()


def spin_for(executor, seconds):
    end = time.time() + seconds
    while time.time() < end:
        executor.spin_once(timeout_sec=0.05)


def test_recorder_logs_escape_path_messages_and_publisher_count(ros, tmp_path):
    topic = '/c2_test/se_escape_path'
    node = rec_mod.Recorder(str(tmp_path / 't.csv'), str(tmp_path / 'd.jsonl'),
                            str(tmp_path / 'p.jsonl'), path_topic=topic)
    pub_node = rclpy.create_node('c2_test_publisher')
    pub = pub_node.create_publisher(Path, topic, 1)
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    ex.add_node(pub_node)
    spin_for(ex, 1.5)                                  # discovery
    msg = Path()
    msg.header.frame_id = 'odom'
    for i in range(3):
        p = PoseStamped()
        p.pose.position.x = float(i)
        p.pose.position.y = 2.0
        p.pose.orientation.z = 0.0
        p.pose.orientation.w = 1.0
        msg.poses.append(p)
    pub.publish(msg)
    spin_for(ex, 1.0)
    node.graph_record('test')
    ex.shutdown()
    node.csv_file.close()
    node.diag_file.close()
    node.path_file.close()
    node.destroy_node()
    pub_node.destroy_node()
    lines = [json.loads(ln) for ln in (tmp_path / 'p.jsonl').read_text().splitlines()]
    paths = [r for r in lines if r['kind'] == 'path']
    graph = [r for r in lines if r['kind'] == 'graph']
    assert len(paths) == 1
    assert paths[0]['frame'] == 'odom' and paths[0]['n_poses'] == 3
    assert paths[0]['first_xy'] == [0.0, 2.0] and paths[0]['last_xy'] == [2.0, 2.0]
    assert paths[0]['last_yaw'] == pytest.approx(0.0)
    assert graph[-1]['tag'] == 'test' and graph[-1]['topic'] == topic and graph[-1]['publishers'] == 1
    # the CSV header is the campaign-1 header, unchanged
    header = (tmp_path / 't.csv').read_text().splitlines()[0]
    assert header == ('wall_s,sim_s,gt_x,gt_y,gt_z,gt_roll,gt_pitch,gt_yaw,odom_x,odom_y,odom_yaw,'
                      'nav_vx,nav_wz,cmd_vx,cmd_wz,load1')


def test_default_topic_is_the_controllers_relative_name_under_the_root_namespace():
    assert rec_mod.ESCAPE_PATH_TOPIC == '/FollowPath/se_escape_path'


def test_recorder_without_a_path_log_writes_no_path_file(ros, tmp_path):
    node = rec_mod.Recorder(str(tmp_path / 't.csv'), str(tmp_path / 'd.jsonl'))
    assert node.path_file is None
    node.graph_record('x')                             # no file: must not raise
    node.csv_file.close()
    node.diag_file.close()
    node.destroy_node()
