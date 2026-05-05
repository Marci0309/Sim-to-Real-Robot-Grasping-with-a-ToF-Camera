#!/usr/bin/env python3
"""
Single-number gripper command for the xArm 1S sim gripper.

Usage:
    python3 gripper_command.py 0.0          # fully CLOSED
    python3 gripper_command.py 1.0          # fully OPEN
    python3 gripper_command.py 0.5 0.4      # half open over 0.4 s

The two fingers are driven anti-mirrored (right = -left), matching the
original URDF mimic intent. The sim-tested closed pose is left=+0.10,
right=-0.10; open pose is left=-0.45, right=+0.45.
"""
import sys
import time

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

CLOSED_LEFT = 0.10
CLOSED_RIGHT = -0.10
OPEN_LEFT = -0.45
OPEN_RIGHT = 0.45


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 1
    width = float(sys.argv[1])
    width = max(0.0, min(1.0, width))
    duration = float(sys.argv[2]) if len(sys.argv) > 2 else 0.5

    left = CLOSED_LEFT + width * (OPEN_LEFT - CLOSED_LEFT)
    right = CLOSED_RIGHT + width * (OPEN_RIGHT - CLOSED_RIGHT)

    rclpy.init()
    node = rclpy.create_node('gripper_command_oneshot')
    pub = node.create_publisher(JointTrajectory, '/gripper_controller/joint_trajectory', 10)

    msg = JointTrajectory()
    msg.joint_names = ['gripper_left_finger_joint', 'gripper_right_finger_joint']
    pt = JointTrajectoryPoint()
    pt.positions = [left, right]
    sec = int(duration)
    pt.time_from_start = Duration(sec=sec, nanosec=int((duration - sec) * 1e9))
    msg.points = [pt]

    # Wait for the publisher to register, then publish a few times.
    for _ in range(20):
        if pub.get_subscription_count() > 0:
            break
        rclpy.spin_once(node, timeout_sec=0.05)
    pub.publish(msg)
    time.sleep(0.1)
    pub.publish(msg)

    print(f'gripper -> width={width:.2f}  '
          f'(left={left:+.3f}, right={right:+.3f})  over {duration:.2f}s')
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
