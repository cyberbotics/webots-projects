"""
perception.py
=============
Finding the ball with a camera instead of asking the simulator.

A camera fixed 2.5 m above the floor looks straight down.  Three steps turn
an image into a ball position:

1. Segmentation: keep the pixels whose colour looks like the orange ball.
       red > 150,   red − blue > 110,   0.35·red < green < 0.75·red
   (the brown and pink floor tiles, the glare and the green disc all fail
   at least one test).

2. Blob analysis: group those pixels into connected blobs and keep only
   blobs with the size and shape of the ball (seen from 2.5 m it is a small
   round disc).  Thin lines, a half-hidden ball or other orange-ish things
   are rejected.  If several blobs remain, take the one nearest to where
   the ball is expected.  Its centroid (u, v) is the ball's centre.

3. Back-projection (pinhole camera model).  With the focal length in pixels

       f = (W / 2) / tan(fov / 2)

   pixel (u, v) corresponds to the viewing ray, in the camera frame
   (x forward, y left, z up),

       r_cam = ( f,  −(u + ½ − W/2),  −(v + ½ − H/2) )

   Rotating it into the world, r = R · r_cam, and walking along it from the
   camera centre c until the height equals the ball radius gives the ball:

       p = c + t · r      with   t = (BALL_RADIUS − c_z) / r_z

Here R and c are read from the simulator.  On a real robot they come from a
camera calibration.
"""

import base64

import numpy as np

import config as C


class OverheadCamera:

    def __init__(self, robot, period_ms=C.CAMERA_PERIOD_MS):
        self.camera = robot.getDevice("overhead_camera")
        self.camera.enable(period_ms)
        self.period = period_ms / 1000.0
        self.width = self.camera.getWidth()
        self.height = self.camera.getHeight()
        self.focal = (self.width / 2) / np.tan(self.camera.getFov() / 2)
        # Camera pose from the simulator.  (Webots' Python getFromDevice()
        # expects the device's internal tag rather than the device object.)
        node = robot.getFromDevice(self.camera._tag)
        self.center = np.array(node.getPosition())
        self.rotation = np.array(node.getOrientation()).reshape(3, 3)
        self.last_detection = None

    # ── image → pixels → world ───────────────────────────────────────────────
    def image_rgb(self):
        """Current image as an H×W×3 uint8 array (Webots gives BGRA bytes)."""
        bgra = np.frombuffer(self.camera.getImage(), dtype=np.uint8)
        return bgra.reshape(self.height, self.width, 4)[:, :, 2::-1]

    @staticmethod
    def ball_mask(rgb):
        """Step 1: True where a pixel looks like the orange ball."""
        r, g, b = (rgb[:, :, i].astype(int) for i in range(3))
        return (r > 150) & (r - b > 110) & (g > 0.35 * r) & (g < 0.75 * r)

    def pixel_to_floor(self, u, v, height=C.BALL_RADIUS):
        """Step 3: world point at the given height seen at pixel (u, v)."""
        ray_cam = np.array([self.focal,
                            -(u + 0.5 - self.width / 2),
                            -(v + 0.5 - self.height / 2)])
        ray = self.rotation @ ray_cam
        t = (height - self.center[2]) / ray[2]
        return self.center + t * ray

    def world_to_pixel(self, p):
        """The opposite of pixel_to_floor: where a world point appears."""
        rel = self.rotation.T @ (np.asarray(p) - self.center)     # camera frame
        if rel[0] <= 0:
            return None                                           # behind the camera
        return [float(self.width / 2 - self.focal * rel[1] / rel[0] - 0.5),
                float(self.height / 2 - self.focal * rel[2] / rel[0] - 0.5)]

    @staticmethod
    def blobs(mask):
        """Step 2a: connected groups of mask pixels (4-neighbourhood)."""
        remaining = set(zip(*np.nonzero(mask)))
        groups = []
        while remaining:
            stack = [remaining.pop()]
            pixels = []
            while stack:
                v, u = stack.pop()
                pixels.append((v, u))
                for n in ((v + 1, u), (v - 1, u), (v, u + 1), (v, u - 1)):
                    if n in remaining:
                        remaining.remove(n)
                        stack.append(n)
            groups.append(np.array(pixels))
        return groups

    def expected_ball_diameter_px(self):
        """How big the ball should look, seen from the camera's height."""
        return 2 * self.focal * C.BALL_RADIUS / (self.center[2] - C.BALL_RADIUS)

    def looks_like_ball(self, pixels):
        """Step 2b: size and shape test for one blob."""
        d = self.expected_ball_diameter_px()
        area = len(pixels)
        height = np.ptp(pixels[:, 0]) + 1
        width = np.ptp(pixels[:, 1]) + 1
        expected_area = np.pi * d * d / 4
        return (0.8 * expected_area <= area <= 1.3 * expected_area
                and max(width, height) <= 1.8 * d + 1
                and min(width, height) >= 0.6 * d)

    def detect(self, near=None):
        """Find the ball in the current image.

        ``near`` (a world position) picks the closest blob when several
        look like a ball.  Returns a dict (pixel centroid, pixel count,
        blob counts, world position) or None if no blob looks like a ball.
        """
        groups = self.blobs(self.ball_mask(self.image_rgb()))
        balls = []
        for pixels in groups:
            if self.looks_like_ball(pixels):
                v, u = pixels.mean(axis=0)
                balls.append({"u": float(u), "v": float(v), "pixels": len(pixels),
                              "p": self.pixel_to_floor(u, v)})
        if not balls:
            self.last_detection = None
            return None
        if near is not None:
            balls.sort(key=lambda b: np.linalg.norm(b["p"][:2] - np.asarray(near)[:2]))
        else:
            balls.sort(key=lambda b: -b["pixels"])
        best = balls[0]
        best["blobs"], best["rejected"] = len(groups), len(groups) - len(balls)
        self.last_detection = best
        return best

    # ── for the robot window ─────────────────────────────────────────────────
    def thumbnail(self, step=2):
        """Down-sampled RGB image, base64-encoded, for the robot window."""
        rgb = np.ascontiguousarray(self.image_rgb()[::step, ::step])
        return {"w": rgb.shape[1], "h": rgb.shape[0], "step": step,
                "rgb": base64.b64encode(rgb.tobytes()).decode("ascii")}
