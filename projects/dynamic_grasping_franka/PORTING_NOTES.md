# Porting notes: MuJoCo → Webots

What changed between the original MuJoCo project
([dynamic-grasping-7dof](https://github.com/haidar996/dynamic-grasping-7dof)) and this Webots
teaching example, and why. Two kinds of change are mixed here, and each item says which kind it is:

- **[teaching]**: simplified on purpose, so a learner can follow the whole system;
- **[engine]**: needed because Webots is not MuJoCo.

---

## 1. Robot model

**[engine] The official `Panda.proto`, fixed upstream during this port.**
The example uses the stock Webots `Panda` with its `PandaHand`. While porting, I rebuilt the
R2025a model's kinematic chain offline, and it did not match Franka's model. The cause, found with
Cyberbotics' help: the `position` fields of joints 4 and 5 (−1.7708 and −1.6) did not match the pose
the links are stored at (−1.6 and 0), so the joint angles Webots reported were offset from Franka's
convention. This was fixed upstream in
[cyberbotics/webots#7078](https://github.com/cyberbotics/webots/pull/7078); the model now matches
Franka's to less than 0.001 mm. [cyberbotics/webots#7079](https://github.com/cyberbotics/webots/pull/7079)
then added the `baseSlot` used for the overhead camera and Franka's datasheet joint limits. The
controller's PoE model and Franka's DH table agree to 1e-6 m (checked in `tests/test_algorithms.py`),
and at runtime the window's **FK check** compares the PoE prediction with a GPS at the TCP.

Things worth knowing:
- The PROTO starts at q = (0, 0, 0, −1.6, 0, 1.6, 0.79), inside the datasheet limits; the
  controller first moves to its own home pose (`GO_HOME`).
- The `Robot` node has no `physics`, so the arm is fixed to the world.
- A `GPS` named `tcp_gps` sits in the `endEffectorSlot`, at the TCP (flange + 0.1 m), so the model
  can be checked live. Franka documents the Hand's TCP at flange + 0.1034 m (see `config.py`).
- Webots reports maximum joint speeds of 2.5 and 3.0 rad/s; the controller uses the tighter
  datasheet values (`config.DQ_MAX`), both for planning and for its velocity commands.

## 2. Control

| MuJoCo project | Webots example | Kind |
|---|---|---|
| Torque control: computed torque / adaptive SMC / ACADOS NMPC, dynamics from Pinocchio | Webots motors in **velocity mode**, `dq_cmd = dq_des + K_p (q_des − q)` | [teaching] + [engine] |
| 5 ms MuJoCo step, `implicitfast` | `basicTimeStep 4` ms (ODE) | [engine] |

Velocity mode (`setPosition(inf)` + `setVelocity`) lets Webots' joint motors find the torque, within
`maxTorque`, so the example needs neither a dynamics library nor an optimisation solver. The
trade-off is that torque limits are no longer modelled by the planner, only speed limits.

## 3. Planning and trajectories

| MuJoCo project | Webots example | Kind |
|---|---|---|
| Three planners (forward scan, CROFT, contractive MPC) | **Forward scan** only | [teaching] |
| TOPP-RA (time-optimal, torque-limited) and quintic | **Quintic** only, duration from the speed limits | [teaching] |
| Analytic IK (`frantik`) + Pinocchio DLS | **NumPy damped-least-squares PoE IK** | [teaching] |
| Reach sphere around the base | Wrist-centre reach test (gripper pointing down) | [teaching] |
| Grasp orientation: TCP x along the ball velocity | Same, plus the 180°-flipped version (parallel gripper is symmetric) | [teaching] |
| Planning time is "paid" from the budget (real-time search) | Planning runs inside one control step; simulated time stops while it runs (0.05–0.4 s wall time) | [engine] |

The flipped orientation was added after finding that warm-starting IK from the previous candidate
could lock onto a joint-7 branch about π away, which made the planner pick very late intercepts.
Seeding from the current pose and trying both orientations made the result repeatable.

## 4. Ball, contacts and perception

| | MuJoCo | Webots |
|---|---|---|
| Ball | sphere, r = 0.03 m, m = 0.1 kg | same |
| Ball–floor | friction 0 (contact pair) | `ContactProperties floor/ball coulombFriction 0` **[engine]** |
| Ball–finger | slide/torsion/roll = 1.5 / 0.5 / 0.1 | `ContactProperties default/ball coulombFriction 1.5`, `softCFM 1e-5`, `bounce 0`. ODE has no torsional/rolling friction. **[engine]** |
| Ball state | read from `qpos/qvel` | **Overhead camera** (default) or Supervisor `getPosition()`; velocity **estimated** by least squares and refreshed by tracking **[teaching]** |
| Launch | initial `qvel` | Supervisor `setVelocity()` on the ball node **[engine]** |

**[teaching] Camera perception.** The MuJoCo project read the ball state directly. Here a 320×240
camera 2.5 m above the floor (mounted in `Panda`'s `baseSlot`) finds the ball by colour
segmentation, blob analysis (size and shape) and pinhole back-projection. Per-frame error against
the simulator's truth is 1–4 mm. The robot plans only from what it perceived: the simulator's truth
is used only to judge the result ("was it caught?") and to show the camera error in the window.

Things testing taught us, and which are worth knowing when porting a perception pipeline:

- **[engine] Webots has no per-camera render layers, so the teaching markers are visible to the
  robot's camera.** Orange "too late" candidate dots were mistaken for the ball, and where the thin
  yellow path line crossed the brown floor tiles it blended into orange pixels. Fixes: markers in
  colours the segmentation rejects (red, cyan), and **blob analysis**, which rejects anything not
  ball-shaped. MuJoCo users who are used to `geomgroup`/visibility flags for debug geometry should
  expect this.
- **[teaching] A short observation gives a noisy direction.** Fitting 0.2 s of frames gives the
  velocity's direction to about ±1°, which becomes a 1–2 cm sideways error after 1–2 s of
  extrapolation: enough to put the ball against a finger. Fix: keep **tracking** while the arm moves
  (re-fit on every frame, with gating) and **correct the meeting point** just before descending.
  Typical correction: 1–22 mm.
- **[teaching] Occlusion.** Within about 25 cm of the gripper, the hand and its shadow distort the
  ball's blob (centroid errors up to 14 mm), so tracking stops there and the last estimate is
  extrapolated.
- **[engine] `Supervisor.getFromDevice()` in Python expects the device's internal tag**
  (`camera._tag`), not the device object.

## 5. Gripper

| MuJoCo project | Webots example |
|---|---|
| Position ramp to the sphere surface, then a torque ramp until the measured contact force exceeds `contact_mult · F_min` | One command: target 1.5 cm inside the ball with `setAvailableForce(grip_force)`. Free motion first, then a force-limited squeeze. Force read back with `getForceFeedback()`. |
| Closure triggered at `distance ≤ speed · GRIPPER_CLOSE_TIME` (0.2 s) | Closure triggered when the predicted time-to-arrival ≤ finger travel time (1 cm / 0.2 m/s = 0.05 s). With a 0.2 s lead the open fingers reach the ball surface well before the ball arrives. |
| Fingers coupled in the model (one actuator drives both) | **[engine]** The Webots `PandaHand` has **one motor per finger**. With a force limit on each and the ball a few mm off-centre, the pair drifted sideways on the frictionless floor until a finger slipped past the ball. Fix: software coupling in `Gripper.update()`, `F_left/right = F ± K·(x_left − x_right)`. The real Franka Hand is mechanically coupled, so this restores the real behaviour. |

Default grip force: 20 N per finger (about 60 × N_min). It is adjustable in the window so learners
can find the slip threshold themselves.

---

## Status: tested in Webots R2025a

All results below come from Webots R2025a on Ubuntu 22.04 (`basicTimeStep 4`), run with
`tests/run_webots_tests.sh` and the scripts it uses.

**Preset scenarios, camera perception (the default):**

| Scenario | Ball | Result | Correction before descending |
|---|---|---|---|
| Still ball | (0.55, 0.10), 0 m/s | caught and lifted | – |
| Gentle roll | (0.50, −0.80), 0.2 m/s, 90° | caught and lifted | 7 mm |
| Original MuJoCo scene | (0.50, −0.80), 0.4 m/s, 90° | caught and lifted | 9 mm |
| Diagonal crossing | (0.80, −0.60), 0.3 m/s, 130° | caught and lifted | 4 mm |
| Coming towards the robot | (1.20, 0.25), 0.3 m/s, 180° | caught and lifted | 1 mm |
| Fast pass | (0.45, −1.00), 0.6 m/s, 90° | caught and lifted | 7 mm |
| Out of reach | (1.40, −1.00), 0.3 m/s, 90° | "no feasible intercept", as intended | – |

The same seven with **simulator perception** give the same results. **12 random launches** with the
camera (start 0.9–1.3 m from the robot, speed 0.15–0.55 m/s, aimed through the reachable area): 12
caught. Camera position error per frame: 1–4 mm; at rest in the Camera tab: 0.9 mm.

**Grip force vs friction** (still ball): 0.2 N slips during the lift, 1 N holds, consistent with
N_min = m·g / (2μ) = 0.33 N. The friction values carried over from MuJoCo (μ = 1.5) needed no
retuning.

**Full feature tour** (`tests/webots_scripts/full_tour.json`, 46 steps): teaching mode pausing at all
five phases, reset, markers on/off, half arm speed (the plan moves later), IK lab (reachable and
unreachable goals), out-of-reach ball, 0.2 N grip (dropped), launching while busy (refused), reset
during a pause, teaching mode switched off during a pause, the exercise checker, and switching
perception between camera and simulator with a catch in each. All steps passed, and Webots printed no
errors or warnings.

**Lesson window, end to end** (`tests/webots_gui_check.py`, 44 checks): headless Chrome loads the
real page served by Webots and clicks through it: connection, telemetry, FK check (0.1 mm), all
eight tabs, Camera tab (ball found, error < 10 mm), perception switch, Exercises tab, presets, launch,
planner table, Launch button disabled while busy, CSV downloads (run data and candidates), teaching
mode through five pauses with a quiz answered, IK lab, grip-force warning, drop, no-plan, page reload.
All passed, with no JavaScript errors. The window was also used by hand in the Webots GUI on Ubuntu,
where it opens in the default browser.

Also checked offline (25 tests, run by GitHub Actions on Python 3.10–3.13): the PoE model matches
Franka's DH table (1e-6); IK, quintic limits, planner, camera
back-projection, the exercise checker (including its time limit), the handling of bad window
messages, and the state machine against a kinematic stand-in for Webots whose camera has the same
16 ms frame delay as Webots.

**Bugs the Webots runs found** (all fixed; each now has a regression test where possible):

1. Supervisor changes apply one step later (see below).
2. Camera frames are rendered every 16 ms, so right after the ball is moved the newest image can still
   show it at its old place. A single stale frame turned a 0.30 m/s ball into 0.77 m/s and the
   planner reported "no feasible intercept". OBSERVE now waits for two fresh frames.
3. Teaching markers seen by the camera; the path-line colour blending into orange (section 4).
4. Fingers drifting sideways with one motor per finger (section 5).
5. Velocity direction from 0.2 s of frames too coarse for long extrapolation → tracking and the
   correction before descending (section 4).
6. Teaching-mode pauses: the predictions, the planned meeting time and the stored camera frames must
   all be shifted by the pause, or the line fit sees a ball that "stood still".

**[engine] Assets are referenced by versioned URL.** The Ubuntu Webots package downloads PROTO files
and meshes on demand, so the world references every external PROTO with a versioned
`https://raw.githubusercontent.com/cyberbotics/webots/<ref>/...` URL; the `webots://` form only works
if the PROTO files are installed locally. R2025a does not include the Panda fixes, so `Panda.proto`
and `PandaHand.proto` are pinned to the commit that merged
[#7079](https://github.com/cyberbotics/webots/pull/7079) (Webots treats any ref of the official
repository as an official asset, so the meshes inside `Panda.proto` resolve at the same commit).
Once Webots R2026a is released, that commit becomes `R2026a`.
Webots caches the downloaded files, so after the first run the example also works offline. On a
slow connection the first download can fail ("Connection closed"); reloading the world retries it.

**[engine] Supervisor changes apply one step later.** When the ball is moved to its launch
position, `getPosition()` still returns the *old* position during that same step. The first
velocity sample was therefore stale, and after a Reset (ball at a different spot) the velocity
estimate was badly wrong and the catch failed. The fix is to skip the first two steps of OBSERVE.
MuJoCo applies `qpos` edits immediately, so this is a real difference to remember when porting.

**[teaching] Teaching-mode pauses freeze the ball, so the predictions must pause too.** The ball's
predicted position depends on the clock; after a pause the prediction had run ahead of the frozen
ball. Each pause now shifts the estimate's time origin, the planned meeting time and the stored
camera frames.
