# Exercises

Six short programming exercises, one for each building block of the dynamic-grasping pipeline.
Each file holds a single function to write, with the formulas you need written above it.

| # | File | You write | Lesson |
|---|---|---|---|
| 1 | `ex1_forward_kinematics.py` | `T(q) = e^[S₁]q₁ ⋯ e^[S₇]q₇ M` | 0 · Kinematics |
| 2 | `ex2_quintic.py` | quintic position and shortest duration | 3 · Approach |
| 3 | `ex3_ik_step.py` | one damped-least-squares IK step | 0 · Kinematics |
| 4 | `ex4_velocity_fit.py` | least-squares line through noisy positions | 1 · Observe |
| 5 | `ex5_grip_force.py` | minimum grip force, with and without lifting | 6 · Close |
| 6 | `ex6_pixel_to_floor.py` | pinhole back-projection, pixel to floor | 1 · Observe |

## How to work

1. Open an exercise file and replace `raise NotImplementedError` with your code.
2. Check it, either:
   - in the lesson window: **Exercises** tab → **Check my exercises** (no need to restart Webots), or
   - in a terminal, from the project folder: `python3 exercises/check.py` (or `check.py 3` for one exercise).
3. A wrong answer comes with a hint. Each file also names the function in the robot's own code that
   solves it, if you want to compare afterwards.

The checker compares your function against the code the robot actually runs, on many random
inputs. A pass means your version would work in the simulation too.
