"""
make_edupanda_proto.py
======================
Regenerates ``protos/EduPanda.proto`` from the official Webots ``Panda.proto``.

Why a separate PROTO?  The official file stores its links at a non-zero
starting pose, which makes it harder to see how the 3D model relates to the
kinematic table used by the controller.  EduPanda keeps the official meshes,
collision shapes and inertias, but rebuilds every joint directly from the
URDF joint frames (all joints at 0 rad in the file), so the model and the
Product-of-Exponentials math in ``controllers/dynamic_grasping/kinematics.py``
come from exactly the same numbers.

Usage:  python3 make_edupanda_proto.py path/to/official/Panda.proto
"""

import re
import sys
from pathlib import Path

import numpy as np

# The official Webots R2025a assets, by versioned URL (Webots caches them after
# the first download).  Until the official Panda.proto uses Franka's joint
# convention, only the arm's kinematic structure is redefined here.
OFFICIAL = "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/robots/franka_emika/panda/protos/"
MESH_URL = OFFICIAL + "meshes/"

# URDF joint origins (franka_description): xyz, rpy — all joints rotate about
# their own z axis.  Limits are the Franka datasheet values.
JOINTS = [
    # xyz                   rpy                     lower     upper    vmax   tmax
    ((0, 0, 0.333),         (0, 0, 0),              -2.8973,  2.8973,  2.175, 87),
    ((0, 0, 0),             (-np.pi / 2, 0, 0),     -1.7628,  1.7628,  2.175, 87),
    ((0, -0.316, 0),        (np.pi / 2, 0, 0),      -2.8973,  2.8973,  2.175, 87),
    # joint 4: datasheet max is -0.0698 rad, but the PROTO starts at 0 rad, so
    # the motor limit is relaxed to 0; the controller (config.Q_MAX) still
    # keeps the joint below -0.0698.
    ((0.0825, 0, 0),        (np.pi / 2, 0, 0),      -3.0718,  0.0,     2.175, 87),
    ((-0.0825, 0.384, 0),   (-np.pi / 2, 0, 0),     -2.8973,  2.8973,  2.610, 12),
    ((0, 0, 0),             (np.pi / 2, 0, 0),      -0.0175,  3.7525,  2.610, 12),
    ((0.088, 0, 0),         (np.pi / 2, 0, 0),      -2.8973,  2.8973,  2.610, 12),
]


def rpy_to_matrix(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def matrix_to_axis_angle(R):
    angle = np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1))
    if angle < 1e-9:
        return (0, 0, 1, 0)
    axis = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    if np.linalg.norm(axis) < 1e-9:          # angle == pi
        axis = np.sqrt(np.maximum(np.diag(R) + 1, 0) / 2)
    axis /= np.linalg.norm(axis)
    return (*axis, angle)


def fmt(v):
    return " ".join(f"{x:.6g}" if abs(x) > 1e-12 else "0" for x in v)


def block_after(text, start):
    """Return text[start:end] where end closes the first '{' or '[' after start."""
    i = start
    while text[i] not in "{[":
        i += 1
    open_c = text[i]
    close_c = "}" if open_c == "{" else "]"
    depth = 0
    for j in range(i, len(text)):
        if text[j] == open_c:
            depth += 1
        elif text[j] == close_c:
            depth -= 1
            if depth == 0:
                return text[start:j + 1]
    raise ValueError("unbalanced block")


def link_parts(solid_block):
    """Extract (cad_url, boundingObject block, physics block) of one link Solid."""
    url = re.search(r'"meshes/(link\d)\.dae"', solid_block).group(1)
    # The link's own boundingObject/physics are the *last* ones in the block
    # (nested joints come first, inside 'children').
    bo_start = solid_block.rindex("boundingObject")
    ph_start = solid_block.rindex("physics Physics")
    return url, block_after(solid_block, bo_start), block_after(solid_block, ph_start)


def indent(text, n):
    lines = [l.strip() for l in text.strip().splitlines()]
    out, depth = [], 0
    for l in lines:
        if l.startswith(("}", "]")):
            depth -= 1
        out.append(" " * (n + 2 * depth) + l)
        if l.endswith(("{", "[")):
            depth += 1
    return "\n".join(out)


def main(official):
    text = Path(official).read_text()
    robot_block = block_after(text, text.index("Robot {"))
    base_solid = block_after(robot_block, robot_block.index("Solid {"))
    links = [link_parts(base_solid)]
    for n in range(1, 8):
        s = text.index(f"endPoint DEF panda_link{n} Solid")
        links.append(link_parts(block_after(text, s)))

    def joint(n, depth):
        xyz, rpy, lo, hi, vmax, tmax = JOINTS[n - 1]
        R = rpy_to_matrix(*rpy)
        axis = R @ np.array([0, 0, 1.0])            # joint z axis seen from the parent
        url, bo, ph = links[n]
        pad = " " * depth
        inner = joint(n + 1, depth + 6) if n < 7 else (
            f"{pad}      Pose {{\n"
            f"{pad}        translation 0 0 0.107   # flange (URDF panda_link8)\n"
            f"{pad}        children [\n"
            f"{pad}          PandaHand {{ }}\n"
            f"{pad}          GPS {{                 # measures the tool centre point (TCP)\n"
            f"{pad}            translation 0 0 0.1\n"
            f"{pad}            name \"tcp_gps\"\n"
            f"{pad}          }}\n"
            f"{pad}        ]\n"
            f"{pad}      }}")
        return (
            f"{pad}DEF JOINT{n} HingeJoint {{\n"
            f"{pad}  jointParameters HingeJointParameters {{\n"
            f"{pad}    axis {fmt(axis)}\n"
            f"{pad}    anchor {fmt(xyz)}\n"
            f"{pad}    dampingConstant 0.003\n"
            f"{pad}  }}\n"
            f"{pad}  device [\n"
            f"{pad}    RotationalMotor {{\n"
            f"{pad}      name \"panda_joint{n}\"\n"
            f"{pad}      maxVelocity {vmax}\n"
            f"{pad}      minPosition {lo}\n"
            f"{pad}      maxPosition {hi}\n"
            f"{pad}      maxTorque {tmax}\n"
            f"{pad}    }}\n"
            f"{pad}    PositionSensor {{\n"
            f"{pad}      name \"panda_joint{n}_sensor\"\n"
            f"{pad}    }}\n"
            f"{pad}  ]\n"
            f"{pad}  endPoint Solid {{\n"
            f"{pad}    translation {fmt(xyz)}\n"
            f"{pad}    rotation {fmt(matrix_to_axis_angle(R))}\n"
            f"{pad}    children [\n"
            f"{pad}      CadShape {{\n"
            f"{pad}        url [ \"{MESH_URL}{url}.dae\" ]\n"
            f"{pad}      }}\n"
            f"{inner}\n"
            f"{pad}    ]\n"
            f"{pad}    name \"panda_link{n}\"\n"
            f"{indent(bo, depth + 4)}\n"
            f"{indent(ph, depth + 4)}\n"
            f"{pad}  }}\n"
            f"{pad}}}")

    url0, bo0, _ = links[0]
    proto = f"""#VRML_SIM R2025a utf8
# license: Apache License 2.0
# license url: https://www.apache.org/licenses/LICENSE-2.0
# keywords: robot/arm
# Franka Emika Panda arm for the dynamic-grasping teaching example.
# Same meshes, collision shapes and inertias as the official Panda.proto, but
# every joint is built from the URDF joint frame with all joints at 0 rad, so
# the model matches the kinematic table in the controller one-to-one.
# Generated by tools/make_edupanda_proto.py - edit that script, not this file.

EXTERNPROTO "{OFFICIAL}PandaHand.proto"

PROTO EduPanda [
  field  SFVec3f     translation        0 0 0
  field  SFRotation  rotation           0 0 1 0
  field  SFString    name               "panda"
  field  SFString    controller         "<generic>"
  field  MFString    controllerArgs     []
  field  SFString    window             "<generic>"
  field  SFString    customData         ""
  field  SFBool      supervisor         FALSE
  field  SFBool      synchronization    TRUE
  field  MFNode      extensionSlot      []      # extra devices fixed to the base (e.g. a camera)
]
{{
  Robot {{
    translation IS translation
    rotation IS rotation
    name IS name
    controller IS controller
    controllerArgs IS controllerArgs
    window IS window
    customData IS customData
    supervisor IS supervisor
    synchronization IS synchronization
    children [
      CadShape {{
        url [ "{MESH_URL}{url0}.dae" ]
      }}
      Group {{
        children IS extensionSlot
      }}
{joint(1, 6)}
    ]
{indent(bo0, 4)}
    # no 'physics' on the base: the arm is bolted to the (static) world
  }}
}}
"""
    out = Path(__file__).resolve().parents[1] / "protos" / "EduPanda.proto"
    out.write_text(proto)
    print(f"wrote {out}")


if __name__ == "__main__":
    main(sys.argv[1])
