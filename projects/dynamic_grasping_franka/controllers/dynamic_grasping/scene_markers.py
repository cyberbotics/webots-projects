"""
scene_markers.py
================
Draws teaching aids directly in the 3D view (no collisions, no physics):

  * the predicted straight-line path of the ball,
  * every intercept candidate the planner tried (red = rejected,
    green = accepted),
  * the pre-grasp / grasp target frames (x red, y green, z blue),
  * a trail behind the gripper.

All markers live inside the ``DEF MARKERS Group`` of the world file, so a
reset just empties that group.
"""

import numpy as np

RESULT_COLORS = {
    "accepted": "0.1 0.85 0.2",
    "too late": "0.9 0.1 0.15",     # not orange: the camera must not mistake it for the ball
    "no IK solution": "0.7 0.1 0.7",
    "out of reach": "0.5 0.5 0.5",
}


def _vec(p):
    return " ".join(f"{x:.4f}" for x in p)


def _rotation_from_matrix(R):
    angle = np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1))
    if angle < 1e-9:
        return "0 0 1 0"
    axis = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    n = np.linalg.norm(axis)
    if n < 1e-9:                                    # 180°
        axis = np.sqrt(np.maximum(np.diag(R) + 1, 0) / 2)
        n = np.linalg.norm(axis)
    return f"{_vec(axis / n)} {angle:.5f}"


def _material(color, transparency=0.0, emissive=True):
    emission = f"emissiveColor {color}" if emissive else ""
    return (f"appearance PBRAppearance {{ baseColor {color} roughness 1 metalness 0 "
            f"transparency {transparency} {emission} }}")


class SceneMarkers:
    def __init__(self, supervisor):
        group = supervisor.getFromDef("MARKERS")
        self.children = group.getField("children") if group else None
        self.enabled = self.children is not None
        self.trail_points = None

    def clear(self):
        if self.children is None:
            return
        while self.children.getCount() > 0:
            self.children.removeMF(-1)
        self.trail_points = None

    def _add(self, vrml):
        if self.enabled:
            self.children.importMFNodeFromString(-1, vrml)

    # ── primitives ───────────────────────────────────────────────────────────
    def sphere(self, p, radius, color, transparency=0.0):
        self._add(f"Pose {{ translation {_vec(p)} children [ Shape {{ "
                  f"{_material(color, transparency)} "
                  f"geometry Sphere {{ radius {radius} subdivision 2 }} castShadows FALSE }} ] }}")

    def line(self, points, color):
        pts = ", ".join(_vec(p) for p in points)
        idx = " ".join(str(i) for i in range(len(points)))
        self._add(f"Shape {{ appearance Appearance {{ material Material {{ "
                  f"diffuseColor {color} emissiveColor {color} }} }} "
                  f"geometry IndexedLineSet {{ coord Coordinate {{ point [ {pts} ] }} "
                  f"coordIndex [ {idx} -1 ] }} castShadows FALSE }}")

    def frame(self, T, size=0.08, label_color="1 1 1"):
        """Coordinate frame: x red, y green, z blue."""
        shafts = []
        for axis_rot, color in (("0 1 0 1.5708", "1 0 0"),
                                ("1 0 0 -1.5708", "0 1 0"),
                                ("0 0 1 0", "0 0 1")):
            shafts.append(
                f"Pose {{ rotation {axis_rot} children [ Pose {{ translation 0 0 {size / 2} "
                f"children [ Shape {{ {_material(color)} geometry Cylinder {{ "
                f"height {size} radius 0.004 }} castShadows FALSE }} ] }} ] }}")
        self._add(f"Pose {{ translation {_vec(T[:3, 3])} rotation {_rotation_from_matrix(T[:3, :3])} "
                  f"children [ {' '.join(shafts)} ] }}")

    # ── teaching aids ────────────────────────────────────────────────────────
    def predicted_path(self, p_now, v, horizon):
        end = np.asarray(p_now) + np.asarray(v) * horizon
        self.line([p_now, end], "0.1 0.85 1")      # cyan: never confused with the orange ball

    def candidates(self, candidates):
        for c in candidates:
            if c["result"] == "out of reach":
                continue                          # keep the picture readable
            radius = 0.012 if c["result"] == "accepted" else 0.006
            self.sphere(c["p"], radius, RESULT_COLORS[c["result"]])

    def start_trail(self, p):
        if not self.enabled:
            return
        self._add(f"Shape {{ appearance Appearance {{ material Material {{ "
                  f"diffuseColor 0.2 0.6 1 emissiveColor 0.2 0.6 1 }} }} "
                  f"geometry IndexedLineSet {{ coord Coordinate {{ point [ {_vec(p)}, {_vec(p)} ] }} "
                  f"coordIndex [ 0 1 -1 ] }} castShadows FALSE }}")
        shape = self.children.getMFNode(-1)
        geometry = shape.getField("geometry").getSFNode()
        self.trail_points = geometry.getField("coord").getSFNode().getField("point")
        self.trail_index = geometry.getField("coordIndex")

    def extend_trail(self, p):
        if self.trail_points is None:
            return
        n = self.trail_points.getCount()
        self.trail_points.insertMFVec3f(n, list(p))
        self.trail_index.insertMFInt32(n, n)       # keep the final -1 last
