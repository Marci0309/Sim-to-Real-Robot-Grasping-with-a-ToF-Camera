#!/usr/bin/env python3
"""Reset the four tabletop objects to their canonical world poses (tabletop.world)
via /set_entity_state. Use between demo runs that physically moved objects."""
import math

import rclpy
from gazebo_msgs.srv import SetEntityState

POSES = {  # name: (x, y, z, roll, pitch, yaw)  -- tighter grid (r~0.18)
    "coke_can":  (0.14, 0.11, 0.776, 1.5708, 0.0, 0.0),
    "banana":    (0.14, -0.11, 0.80, 0.0, 0.0, 0.0),
    "golf_ball": (-0.14, 0.11, 0.80, 0.0, 0.0, 0.0),
    "blue_box":  (-0.14, -0.11, 0.80, 0.0, 0.0, 0.0),
}


def rpy_to_quat(r, p, y):
    cy, sy = math.cos(y * 0.5), math.sin(y * 0.5)
    cp, sp = math.cos(p * 0.5), math.sin(p * 0.5)
    cr, sr = math.cos(r * 0.5), math.sin(r * 0.5)
    return (sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


rclpy.init()
node = rclpy.create_node("reset_objects")
cli = node.create_client(SetEntityState, "/set_entity_state")
if not cli.wait_for_service(timeout_sec=5.0):
    print("set_entity_state unavailable")
    raise SystemExit(1)
for name, (x, y, z, R, P, Y) in POSES.items():
    req = SetEntityState.Request()
    req.state.name = name
    req.state.pose.position.x = float(x)
    req.state.pose.position.y = float(y)
    req.state.pose.position.z = float(z)
    qx, qy, qz, qw = rpy_to_quat(R, P, Y)
    req.state.pose.orientation.x = qx
    req.state.pose.orientation.y = qy
    req.state.pose.orientation.z = qz
    req.state.pose.orientation.w = qw
    req.state.reference_frame = "world"
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=3.0)
    ok = fut.result() is not None and fut.result().success
    print(f"{name}: {'ok' if ok else 'FAIL'}")
rclpy.shutdown()
