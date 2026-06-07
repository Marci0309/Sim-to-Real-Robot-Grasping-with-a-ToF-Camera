#!/usr/bin/env python3
"""Continuously log the 4 objects' world poses from /model_states to a CSV
(append, flushed) so a headless grasp run can be checked afterwards for sinking
(min z below the tabletop) and flinging (large excursion / z on the floor).
Run in the background; kill when the demo finishes. Output: /tmp/grasp_track.csv
"""
import time
import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates

OBJS = ("coke_can", "banana", "golf_ball", "blue_box")


def main():
    f = open("/tmp/grasp_track.csv", "w")
    f.write("t,obj,x,y,z\n")
    rclpy.init()
    n = Node("grasp_monitor")

    def cb(m):
        t = time.time()
        for name, p in zip(m.name, m.pose):
            if name in OBJS:
                f.write(f"{t:.2f},{name},{p.position.x:.4f},"
                        f"{p.position.y:.4f},{p.position.z:.4f}\n")
        f.flush()

    n.create_subscription(ModelStates, "/model_states", cb, 10)
    rclpy.spin(n)


if __name__ == "__main__":
    main()
