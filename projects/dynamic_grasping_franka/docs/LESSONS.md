# Lessons

Seven short lessons, one per phase of the pipeline. Each one has **what to watch**, **things to
try** in the lesson window, **code to read**, and where it applies, an **exercise** from
[`exercises/`](../exercises/). Every lesson card in the window also ends with a short
**check-yourself quiz**. Do the lessons in order: each builds on the one before.

Before you start: open `worlds/dynamic_grasping.wbt`, run the simulation, and double-click the
robot to open the lesson window.

---

## Lesson 0 · Kinematics: where is the gripper?

**Idea.** Forward kinematics maps joint angles to the gripper pose. With screw theory every joint is
a *screw axis* `Sᵢ = (ωᵢ, vᵢ)`, and the pose is a product of matrix exponentials:

```
T(q) = e^[S₁]q₁ · e^[S₂]q₂ · … · e^[S₇]q₇ · M
```

`M` is the gripper pose when all joints are at zero. Inverse kinematics goes the other way. This
project uses damped least squares: measure the twist error `V = log(T(q)⁻¹ T_desired)`, take a step
`Δq = Jᵀ(JJᵀ + λ²I)⁻¹V`, and repeat.

**Watch.** The **FK check** in the status bar. It compares `T(q)` computed in Python with the
gripper position measured in the simulation. It should stay below a millimetre.

**Try.**
- *IK lab* tab: x = 0.45, y = 0.2, z = 0.3 → **Solve IK & move**. Look at the convergence plot:
  the error falls by about a factor of 10 per iteration near the solution.
- Push x to 0.8 and z to 0.03. Does IK still succeed? Why not? (Compare with the green disc on the
  floor.)
- Change *yaw* by 180°. The arm reaches the same point with a different wrist angle.

**Read.** `kinematics.py`: `exp_twist`, `forward_kinematics`, `space_jacobian`, `ik_from_seed`.

**Exercises.** 1 (forward kinematics) and 3 (one IK step).

**Question.** The Panda has 7 joints but a pose has 6 numbers. What does the 7th joint give you?
(Hint: try the same IK goal twice after moving the arm somewhere else in between.)

---

## Lesson 1 · Observe: seeing the ball and estimating its motion

**Idea.** A camera 2.5 m above the floor looks straight down. Each image goes through three steps:

1. **Segmentation.** Keep pixels whose colour looks like the orange ball
   (`red > 150`, `red − blue > 110`, `0.35·red < green < 0.75·red`).
2. **Blob analysis.** Group those pixels into connected blobs and keep the one whose size and shape
   match a ball seen from 2.5 m (a disc about 6 px across). Thin lines and half-hidden balls are
   rejected.
3. **Back-projection.** The pinhole model turns the blob's centre pixel into a viewing ray. Where the
   ray reaches the ball's height is the ball's position on the floor.

Then a straight line `p(t) = p₀ + v·t` is fitted through 0.2 s of positions by least squares.

**Watch.** The *Camera* tab: the cyan circle is the blob accepted as the ball. The table shows the
camera's position estimate next to the simulator's truth (shown only for you; the robot never uses
it in camera mode). Typical error: 1–4 mm.

**Try.**
- Launch *Gentle roll* and *Fast pass*, then read "Estimated ball velocity … from 13 camera frames;
  position error … mm" in the *Log*.
- Switch **Perception** to *Simulator (exact)* and launch again. What changes in the log?
- Start a ball right next to the robot base (x = 0.3, y = 0.0) and launch. Can the camera still see
  it under the arm?

**Read.** `perception.py` (`ball_mask`, `blobs`, `looks_like_ball`, `pixel_to_floor`);
`dynamic_grasping.py → fit_ball_motion`.

**Exercises.** 4 (velocity from noisy positions) and 6 (pixel to floor).

**Question.** Why fit a line through 13 frames instead of subtracting two positions? What would a
2 mm error do to a velocity computed from two frames 16 ms apart?

---

## Lesson 2 · Plan: where and when do we meet?

**Idea.** The ball keeps moving in a straight line. Try meeting times `t = 0.05, 0.10, …` and keep
the first one where:

1. the ball is inside the reachable area,
2. IK finds joint angles for the pre-grasp and the grasp pose,
3. the arm can get there in time: `T_approach + T_descend + T_close + margin ≤ t`.

**Watch.** The *Planner* tab:
- **top view**: candidates along the yellow path,
- **timing chart**: the dots (time *needed*) start above the diagonal (time *available*) and the
  first dot below it is the plan,
- **table**: every candidate and its rejection reason.

**Try.**
- *Original MuJoCo scene* with *Arm speed* 1.0, then 0.5. The plan moves further along the path.
- *Out of reach (no plan)*. A good planner says "impossible" quickly instead of trying anyway.
- Set your own launch so the ball passes close to the robot base (start x = 0.2, y = −0.9,
  heading 90°). Which candidates are rejected as *out of reach*, and where does the planner
  still find a catch?

**Read.** `intercept_planner.py`: `plan_intercept`, `best_grasp_ik`, `in_workspace`.

**Question.** Why does the planner try the gripper both ways round (`flipped`)? Look at joint 7's
limits in `config.py`.

---

## Lesson 3 · Approach: moving smoothly and in time

**Idea.** A quintic time scaling `s(τ) = 10τ³ − 15τ⁴ + 6τ⁵` moves all joints together from rest to
rest with zero velocity and acceleration at both ends. The shortest duration that respects the speed
limits is `T = max_j 15|Δq_j| / (8·dq_max,j)`.

Each joint then tracks the trajectory with

```
dq_cmd = dq_desired + K_p (q_desired − q_measured)
```

The feed-forward term `dq_desired` does most of the work. The feedback term corrects the small
remaining error.

**Watch.** *Motion* tab: desired (orange) and measured (blue) almost overlap; the tracking error
stays at a few milliradians.

**Try.** In `config.py` set `KP_JOINT = 0`. The arm still roughly follows (feed-forward only) but
drifts. Then put it back and set it very high, e.g. `200`. What changes in the error plot?

**Read.** `trajectory.py`; `dynamic_grasping.py → control_arm()`.

**Exercise.** 2 (quintic trajectory).

---

## Lesson 4 · Descend: approach from above

**Idea.** Coming from the side could push the ball away. The gripper first hovers 12 cm above the
meeting point, then goes straight down. Its x-axis points along the ball's velocity, so the fingers,
which close along y, end up on either side of the path like a gate.

**Watch.** The coloured frames in the 3D view (x red, y green, z blue) at the pre-grasp and grasp
poses. Turn on **Teaching mode** and look at them before pressing *Next*.

**Try.** *Diagonal crossing*: the frames rotate with the ball's direction.

**Read.** `intercept_planner.py → grasp_orientation, grasp_poses`.

---

## Lesson 5 · Keep watching, correct, then close at the right instant

**Idea.** The plan was made from only 0.2 s of observation. A tiny error in the direction of the
velocity, say 1°, puts the meeting point centimetres to the side after a second or two of
travel. So the robot **keeps tracking**: while the arm moves, every new camera frame is added and
the line is re-fitted. Frames far from the prediction are ignored (**gating**). Just before descending,
the meeting point is **corrected** with the fresh estimate.

Once the ball is within 25 cm of the gripper, the hand and its shadow start to hide it, so the
last estimate is extrapolated. The fingers need `(4 cm − 3 cm) / 0.2 m/s = 0.05 s` to reach the
ball surface, so they start closing when the predicted time-to-arrival drops to 0.05 s.

**Watch.** *Log*: "Tracking: … refreshed from N more measurements" and "Correction before
descending: meeting point moved by N mm". In the *Camera* tab, the yellow cross (the prediction)
stays on the ball.

**Try.** *Fast pass* (0.6 m/s), then raise the ball speed to 0.8 m/s. If it misses, read the
*Log* to find out why.

**Read.** `dynamic_grasping.py`: `track_ball`, `start_descend`, state `"WAIT"`;
`gripper.py → closing_time()`.

**Question.** Why stop using camera frames when the ball gets close to the gripper, instead of using
every frame until the end?

---

## Lesson 6 · Close: how hard to squeeze?

**Idea.** The ball is held by friction only. Two fingers each pressing with force N can carry at
most `2μN`, so they need `N ≥ m·g / (2μ)`. With m = 0.1 kg and μ = 1.5 that is only 0.33 N, but
the lift also *accelerates* the ball, and simulated contacts are not perfect.

The command is a single one: target 1.5 cm *inside* the ball, with the motor force capped at the grip
force. The finger moves freely (position phase), touches, and then pushes at the cap (force phase).

**Watch.** *Gripper* tab: the opening stops at 3 cm (the ball radius) and the force jumps to the
grip force you set.

**Try.** Grip force 20 N → 2 N → 0.5 N → 0.2 N. When does the ball start slipping during the
lift? Is it exactly at 0.33 N? Why might it not be?

Something only visible in simulation: the real Franka Hand couples its two fingers mechanically,
but the Webots model gives each finger its own motor. If the ball is a few millimetres off-centre,
the pair can slide sideways on the frictionless floor. `Gripper.update()` couples them in software:
the finger pushed more open gets more force.

**Read.** `gripper.py`.

**Exercise.** 5 (grip force and friction).

---

## Lesson 7 · Put it together

**Try.**
- Design a launch the robot *just barely* catches (slack < 0.05 s in the Planner summary).
- Lower *Arm speed* until that launch becomes impossible.
- Launch a ball heading *towards* the robot (*Coming towards the robot*). Which phase is hardest
  here, and why?

**Analyse.** In the *Log* tab, **export the run data (CSV)** and plot, in Python or a spreadsheet,
the desired and measured angle of joint 2, or the finger forces during the close.

**Extend** (bigger programming projects):
1. Add a "no-go zone" to `in_workspace` (e.g. `y < 0` forbidden) and watch the planner route around it.
2. Add Gaussian noise (σ = 5 mm) to the ball position in `OBSERVE` and see how the velocity
   estimate and the catch change. Then make `OBSERVE_TIME` longer.
3. Replace the quintic with a cubic (`s = 3τ² − 2τ³`). Its peak speed is `1.5/T`, so change
   `quintic_duration` too. Compare the Motion plots.
