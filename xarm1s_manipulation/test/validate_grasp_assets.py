#!/usr/bin/env python3
"""Static validation of the realistic-grasp assets (XML well-formedness +
ros2_control / controller wiring). Run after editing the URDF, world, object
SDFs, or controllers.yaml. Checks the INSTALLED copies (what Gazebo actually
loads). Exits non-zero on the first problem.
"""
import sys
import xml.dom.minidom as minidom
import xml.etree.ElementTree as ET

import yaml

INSTALL = "/home/szokedencsim/thesis_ws/install"
DESC = f"{INSTALL}/xarm1s_description/share/xarm1s_description"
GAZ = f"{INSTALL}/xarm1s_gazebo/share/xarm1s_gazebo"

URDF = f"{DESC}/urdf/xarm.urdf"
CTRL = f"{DESC}/config/controllers.yaml"
WORLD = f"{GAZ}/worlds/tabletop.world"
SDFS = [f"{GAZ}/models/tabletop_objects/{o}/model.sdf"
        for o in ("coke_can", "golf_ball", "banana")]

ok = True


def fail(msg):
    global ok
    ok = False
    print(f"  FAIL: {msg}")


# 1) XML well-formed
for f in [URDF, WORLD, *SDFS]:
    try:
        minidom.parse(f)
        print(f"OK  well-formed: {f.split('/')[-1]} ({f.split('/')[-2]})")
    except Exception as e:  # noqa: BLE001
        fail(f"XML parse {f}: {e}")

# 2) URDF: gripper joints must use the EFFORT command interface; arm stays position
root = ET.parse(URDF).getroot()
r2c = root.find("ros2_control")
cmd = {}
for j in r2c.findall("joint"):
    ci = j.find("command_interface")
    cmd[j.get("name")] = ci.get("name") if ci is not None else None
for jn in ("gripper_left_finger_joint", "gripper_right_finger_joint"):
    if cmd.get(jn) != "effort":
        fail(f"{jn} command_interface={cmd.get(jn)} (expected effort)")
    else:
        print(f"OK  {jn} -> effort")
for jn in ("xarm_2_joint", "xarm_3_joint"):
    if cmd.get(jn) != "position":
        fail(f"{jn} command_interface={cmd.get(jn)} (expected position)")
print(f"OK  arm joints stay on position interface")

# 3) controllers.yaml: gripper effort + gains for both finger joints
d = yaml.safe_load(open(CTRL))
g = d["gripper_controller"]["ros__parameters"]
if g.get("command_interfaces") != ["effort"]:
    fail(f"gripper command_interfaces={g.get('command_interfaces')}")
else:
    print("OK  gripper_controller command_interfaces=[effort]")
gains = g.get("gains", {})
for jn in ("gripper_left_finger_joint", "gripper_right_finger_joint"):
    if "p" not in gains.get(jn, {}):
        fail(f"missing PID gains for {jn}")
    else:
        print(f"OK  gains[{jn}] p={gains[jn]['p']} d={gains[jn].get('d')}")
if d["arm_controller"]["ros__parameters"].get("command_interfaces") != ["position"]:
    fail("arm_controller command_interfaces changed (expected [position])")

# 4) world: contact-trigger plugin params present
wtxt = open(WORLD).read()
for tag in ("left_finger_link", "right_finger_link", "grip_time", "fallback_timeout"):
    if f"<{tag}>" not in wtxt:
        fail(f"world plugin missing <{tag}>")
print("OK  world plugin has contact-trigger params")

# 5) every object collision has a <surface> (friction/contact tuning)
for f in [WORLD, *SDFS]:
    t = open(f).read()
    if "<collision" in t and "<surface>" not in t:
        fail(f"{f.split('/')[-1]}: collision without <surface>")
print("OK  object collisions carry <surface> params")

print("\nALL VALID" if ok else "\nVALIDATION FAILED")
sys.exit(0 if ok else 1)
