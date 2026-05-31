#!/usr/bin/env python3
r"""Live viewer for an Image topic (default: the pose_estimator debug view).

Why this exists: rqt_image_view defaults to RELIABLE QoS, but Gazebo camera
topics are BEST_EFFORT, so rqt shows nothing. This viewer subscribes
BEST_EFFORT, which receives BOTH best-effort (camera) and reliable (our debug)
publishers.

Usage (after `source ~/thesis_ws/install/setup.bash` + `export ROS_LOCALHOST_ONLY=1`):
    python3 ~/thesis_ws/src/tof_perception/scripts/view_debug.py
    python3 .../view_debug.py /overhead_rgb/image_raw     # raw camera
    python3 .../view_debug.py /pose_estimator/mask_banana # one HSV mask

Shows an OpenCV window. If the cv2 GUI build is unavailable in WSL, it instead
writes the latest frame to /tmp/debug_view.png every frame — open that from
Windows Explorer:  \\wsl.localhost\Ubuntu-22.04\tmp\debug_view.png
"""
import os
import sys

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy, DurabilityPolicy,
                       HistoryPolicy)
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

_args = [a for a in sys.argv[1:] if not a.startswith('-')]
FORCE_PNG = '--png' in sys.argv          # skip the live window, just write a PNG
TOPIC = _args[0] if _args else '/pose_estimator/debug_view'


class Viewer(Node):
    def __init__(self):
        super().__init__('debug_viewer')
        self.bridge = CvBridge()
        self.latest = None
        self.count = 0
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE,
                         history=HistoryPolicy.KEEP_LAST, depth=5)
        self.create_subscription(Image, TOPIC, self._cb, qos)
        self.get_logger().info(f'subscribed to {TOPIC} (BEST_EFFORT). '
                               'Ctrl-C to quit.')

    def _cb(self, msg: Image):
        try:
            if msg.encoding in ('rgb8', 'bgr8'):
                self.latest = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            else:                       # mono8 masks, etc.
                self.latest = self.bridge.imgmsg_to_cv2(msg)
            self.count += 1
        except Exception as e:
            self.get_logger().warn(f'convert error: {e}')


def main():
    rclpy.init()
    v = Viewer()
    # Only attempt the OpenCV window if there's a display (WSLg sets $DISPLAY);
    # otherwise imshow can hard-abort. No display -> write a PNG instead.
    gui = bool(os.environ.get('DISPLAY')) and not FORCE_PNG
    if not gui:
        v.get_logger().warn(
            r'no $DISPLAY -> writing /tmp/debug_view.png each frame; open it '
            r'from Windows: \\wsl.localhost\Ubuntu-22.04\tmp\debug_view.png')
    try:
        while rclpy.ok():
            rclpy.spin_once(v, timeout_sec=0.05)
            if v.latest is None:
                continue
            if v.count == 1 or v.count % 30 == 0:
                v.get_logger().info(f'received {v.count} frames from {TOPIC}')
            if gui:
                try:
                    cv2.imshow(TOPIC, v.latest)
                    if cv2.waitKey(1) == 27:   # Esc
                        break
                    continue
                except Exception:
                    gui = False
                    v.get_logger().warn(
                        'OpenCV GUI unavailable -> writing /tmp/debug_view.png')
            cv2.imwrite('/tmp/debug_view.png', v.latest)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        v.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
