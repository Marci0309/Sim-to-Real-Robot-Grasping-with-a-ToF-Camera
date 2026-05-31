#!/usr/bin/env python3
"""Print current world positions of the four objects (to confirm the carry
physically moved them). One-shot read of /model_states."""
import time

import rclpy
from gazebo_msgs.msg import ModelStates

OBJ = ["coke_can", "banana", "golf_ball", "blue_box"]

rclpy.init()
node = rclpy.create_node("post_check")
got = {}


def cb(m):
    for name in OBJ:
        if name in m.name:
            p = m.pose[m.name.index(name)].position
            got[name] = (round(p.x, 3), round(p.y, 3), round(p.z, 3))


node.create_subscription(ModelStates, "/model_states", cb, 10)
t0 = time.time()
while time.time() - t0 < 5 and len(got) < 4:
    rclpy.spin_once(node, timeout_sec=0.2)
for k in OBJ:
    print(f"{k:<10} {got.get(k)}")
rclpy.shutdown()
