#!/usr/bin/env python3
"""Offline import smoke test: confirms the node module and all its ROS message
/ action / service imports resolve (no running sim needed). Run with the
workspace install sourced."""
import xarm1s_manipulation.grasp_executor as g

print("import OK")
print("objects:", list(g.OBJECTS))
print("arm joint order:", g.ik.ARM_JOINT_ORDER)
print("HOME:", g.HOME_CFG)
