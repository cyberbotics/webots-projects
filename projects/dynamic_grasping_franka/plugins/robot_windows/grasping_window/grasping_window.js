// Robot window for the dynamic-grasping teaching example.
// Talks to controllers/dynamic_grasping/dynamic_grasping.py with JSON messages:
//   window → controller : {cmd: "launch" | "reset" | "next" | "teach" | ...}
//   controller → window : {type: "hello" | "phase" | "plan" | "telemetry" | "ik" | "log"}

import RobotWindow from 'https://cyberbotics.com/wwi/R2025a/RobotWindow.js';

// ═══════════════════════════════════════════════════════════════════════════
//  Scenarios
// ═══════════════════════════════════════════════════════════════════════════
const PRESETS = [
  {name: 'Still ball (static grasp)', x: 0.55, y: 0.10, speed: 0.0, heading: 0},
  {name: 'Gentle roll', x: 0.50, y: -0.80, speed: 0.20, heading: 90},
  {name: 'Original MuJoCo scene', x: 0.50, y: -0.80, speed: 0.40, heading: 90},
  {name: 'Diagonal crossing', x: 0.80, y: -0.60, speed: 0.30, heading: 130},
  {name: 'Coming towards the robot', x: 1.20, y: 0.25, speed: 0.30, heading: 180},
  {name: 'Fast pass', x: 0.45, y: -1.00, speed: 0.60, heading: 90},
  {name: 'Out of reach (no plan)', x: 1.40, y: -1.00, speed: 0.30, heading: 90}
];

// ═══════════════════════════════════════════════════════════════════════════
//  Lesson texts — one card per phase
// ═══════════════════════════════════════════════════════════════════════════
const LESSONS = {
  IDLE: {
    title: 'Catching a moving ball',
    body: `<p>A ball slides across the floor. The robot has to <b>predict</b> where it will be,
      <b>decide</b> where and when to meet it, <b>move</b> there in time and <b>close</b> its gripper
      at exactly the right moment.</p>
      <p>Each step of that pipeline (top bar) is a classic robotics building block. Pick a scenario,
      press <b>Launch ball</b> and watch the cards change. Tick <b>Teaching mode</b> to freeze the
      scene before every step and read at your own pace.</p>
      <div class="try"><b>Start here:</b> "Still ball" first (static grasping), then "Gentle roll",
      then the faster ones.</div>`,
    code: 'controllers/dynamic_grasping/dynamic_grasping.py  — the state machine'
  },
  GO_HOME: {
    title: 'Going home',
    body: `<p>The arm moves to its <b>ready pose</b>: elbow bent, gripper pointing down, fingers open.
      Every mission starts from here.</p>`,
    code: 'config.py → Q_HOME'
  },
  OBSERVE: {
    title: '1 · Observe: find the ball and estimate its motion',
    body: `<p>A camera 2.5 m above the floor looks straight down (see the <b>Camera</b> tab).
      Each frame goes through three steps:</p>
      <ol>
        <li><b>Segmentation</b>: keep the pixels whose colour looks like the orange ball.</li>
        <li><b>Blob analysis</b>: group them into blobs and keep the one with the size and shape of
          the ball. This rejects thin lines and half-hidden balls.</li>
        <li><b>Back-projection</b>: turn the blob's centre pixel into a floor position (pinhole model):</li>
      </ol>
      <div class="eq">f = (W/2) / tan(fov/2)
ray = R · ( f, −(u − W/2), −(v − H/2) )      p = c + t·ray   (height = ball radius)</div>
      <p>Then a straight line is fitted through 0.2 s of positions (least squares):</p>
      <div class="eq">p(t) = p₀ + v · t        →   solve for p₀ and v</div>
      <p>Fitting many frames rather than differencing two averages out the pixel noise.
      Switch <b>Perception</b> to "Simulator (exact)" to skip the camera and use perfect positions.</p>`,
    code: 'perception.py → OverheadCamera.detect()   |   dynamic_grasping.py → fit_ball_motion()'
  },
  PLAN: {
    title: '2 · Plan: where and when do we meet?',
    body: `<p>Because the floor is frictionless, the ball keeps moving in a straight line (cyan path).
      The planner tries meeting times <i>t</i> = 0.05 s, 0.10 s, … and for each one asks:</p>
      <ol>
        <li><b>Reach</b>: is the predicted ball position inside the green area?</li>
        <li><b>IK</b>: can the joints place the gripper above and around the ball? The two-finger
          gripper is symmetric, so it is tried both ways round (turned by 180°), and the faster one is kept.</li>
        <li><b>Time</b>: is the arm fast enough?</li>
      </ol>
      <div class="eq">needed(t) = T_approach + T_descend + T_close + margin   ≤   t ?</div>
      <p>The first candidate that passes all three becomes the plan. Open the <b>Planner</b> tab:
      the timing chart shows "needed" dropping under "available" exactly at the accepted point.</p>`,
    code: 'intercept_planner.py → plan_intercept()'
  },
  APPROACH: {
    title: '3 · Approach: a smooth joint-space motion',
    body: `<p>The arm goes to the <b>pre-grasp</b> pose 12 cm above the meeting point along a
      <b>quintic</b> (minimum-jerk) trajectory. All 7 joints start and stop together:</p>
      <div class="eq">s(τ) = 10τ³ − 15τ⁴ + 6τ⁵ ,   τ = t / T
q(t) = q_start + s(τ) · (q_end − q_start)</div>
      <p>Velocity and acceleration are zero at both ends. The duration T is the shortest that keeps
      every joint under its speed limit (the peak of ds/dt is 15/8T):</p>
      <div class="eq">T = max_j  15 |Δq_j| / (8 · dq_max_j)</div>
      <p>Each joint follows it with <b>feed-forward + proportional feedback</b>:</p>
      <div class="eq">dq_cmd = dq_desired + K_p · (q_desired − q_measured)</div>
      <p>See the <b>Motion</b> tab: the measured curve lies almost on top of the desired one.</p>`,
    code: 'trajectory.py → QuinticTrajectory   |   dynamic_grasping.py → control_arm()'
  },
  DESCEND: {
    title: '4 · Descend: why a pre-grasp pose?',
    body: `<p>Coming straight from the side could knock the ball away. Instead the gripper first
      hovers <b>above</b> the path, then goes straight down with its fingers open, one on each
      side of the ball's path.</p>
      <p>The gripper's x-axis is turned to point along the ball's velocity, so the fingers (which
      close along the gripper's y-axis) form a gate the ball slides into. Watch the coloured
      frames drawn in the 3D view: <span style="color:#f55">x</span>,
      <span style="color:#5f5">y</span>, <span style="color:#59f">z</span>.</p>`,
    code: 'intercept_planner.py → grasp_orientation(), grasp_poses()'
  },
  WAIT: {
    title: '5 · Wait: closing at the right instant',
    body: `<p>The arm is in place before the ball arrives (that was the safety margin). Now timing
      is everything: closing too early hits the ball, closing too late lets it pass.</p>
      <div class="eq">time_to_arrival = distance_along_path / speed
close when  time_to_arrival ≤ finger_travel_time = (4 cm − 3 cm) / 0.2 m/s = 0.05 s</div>
      <p>The fingers then reach the ball's surface at the moment the ball reaches the centre of
      the gripper.</p>
      <p><b>Keep watching:</b> while the arm moved, the camera kept tracking the ball and the line was
      re-fitted on every new frame, so the estimate is fresh. Once the ball is within 25 cm of the
      gripper, the hand and its shadow start to hide it, so the last estimate is extrapolated for the
      final moments. Frames far from the prediction are ignored (<i>gating</i>).</p>`,
    code: 'dynamic_grasping.py → state "WAIT", track_ball()   |   gripper.py → closing_time()'
  },
  CLOSE: {
    title: '6 · Close: how hard to squeeze?',
    body: `<p>The ball is held by <b>friction</b> only. Each finger presses with normal force N; the
      friction it can transmit is at most μ·N. Two fingers must carry the weight m·g:</p>
      <div class="eq">2 · μ · N ≥ m · g     →     N_min = m·g / (2μ)</div>
      <p>The finger target is set <i>inside</i> the ball, with the motor force capped at the grip
      force you chose. The finger moves freely, touches the ball, and then pushes at that capped
      force. One command, two phases: position, then force.</p>
      <p>On the real Franka Hand the two fingers are coupled mechanically. In this model each finger has
      its own motor, so the controller couples them in software: the finger pushed more open gets more
      force. Otherwise an off-centre ball could slide the pair sideways.</p>
      <p>See the <b>Gripper</b> tab: the opening stops at the ball radius and the force jumps to the
      grip force.</p>`,
    code: 'gripper.py → Gripper.close(), min_grip_force()'
  },
  LIFT: {
    title: '7 · Lift and check',
    body: `<p>The arm lifts the gripper straight up (a new IK solution for the same x, y and a new
      height). At the end, the controller checks that the ball moved up with it. Accelerating
      upwards needs more than N_min, which is one reason the default grip force has a large
      safety factor.</p>`,
    code: 'dynamic_grasping.py → start_lift(), state "LIFT"'
  },
  CAUGHT: {
    title: 'Caught!',
    body: `<p>Every block worked together: perception → prediction → intercept planning →
      inverse kinematics → trajectory → joint control → timed grasp.</p>
      <div class="try"><b>Try next:</b> lower the <b>arm speed</b> and launch the same scenario. The
      planner has to pick a later meeting point (Planner tab). Then lower the <b>grip force</b>
      below the minimum and see what happens during the lift.</div>`,
    code: 'Press Reset, change one thing, launch again.'
  },
  MISSED: {
    title: 'Missed',
    body: `<p>The ball was not between the fingers when they closed. Typical reasons: the ball was
      too fast for the closing time, or the arm had not settled exactly on the predicted path.</p>
      <div class="try"><b>Investigate:</b> the Motion tab shows tracking error. The Log shows when
      the gripper started closing.</div>`,
    code: 'dynamic_grasping.py → states "WAIT" and "CLOSE"'
  },
  DROPPED: {
    title: 'Dropped',
    body: `<p>The fingers closed on the ball but the friction force was not enough to carry it while
      lifting. Compare your grip force with N_min = m·g / (2μ), then remember the lift also
      accelerates the ball.</p>`,
    code: 'gripper.py → min_grip_force()'
  },
  NO_PLAN: {
    title: 'No feasible intercept',
    body: `<p>Every candidate failed: the ball never passes through the green area at a time the arm can
      make it. A planner that can say "impossible" quickly is just as useful as one that finds
      a plan.</p>
      <div class="try"><b>Look at</b> the Planner tab: every candidate is grey (out of reach) or
      orange (too late).</div>`,
    code: 'intercept_planner.py → plan_intercept() returns None'
  },
  IK_MOVE: {
    title: 'IK lab: moving to your pose',
    body: `<p>Inverse kinematics found joint angles for your target. The arm is moving there with the same
      quintic trajectory and joint controller used for catching.</p>`,
    code: 'kinematics.py → solve_ik()'
  }
};

// ═══════════════════════════════════════════════════════════════════════════
//  Quizzes — one "check yourself" question per phase
// ═══════════════════════════════════════════════════════════════════════════
const QUIZZES = {
  OBSERVE: {
    q: 'Each camera position has about 2 mm of noise, and frames arrive every 16 ms. Why fit a line through many frames instead of using the last two?',
    options: ['Two frames take too long to compute.',
      '2 mm of noise over 16 ms could fake a velocity of about 0.1 m/s; a fit over many frames averages the noise out.',
      'The camera only sends two frames per second.'],
    answer: 1,
    why: 'Differencing two noisy positions divides the noise by a tiny time step. A least-squares fit over 0.2 s uses a long baseline and many samples.'
  },
  PLAN: {
    q: 'You halve the arm speed and launch the same ball. What happens to the accepted meeting time?',
    options: ['It gets earlier.', 'It gets later.', 'It stays the same.'],
    answer: 1,
    why: 'The time needed doubles, so "needed ≤ available" first holds further along the path. Try it and look at the timing chart.'
  },
  APPROACH: {
    q: 'A quintic trajectory moves a joint by Δq in T seconds. What is the peak joint speed?',
    options: ['Δq / T', '15 Δq / (8 T)', '2 Δq / T'],
    answer: 1,
    why: 'ds/dτ = 30τ² − 60τ³ + 30τ⁴ peaks at τ = ½ with value 15/8, so the peak speed is 15Δq/(8T), about 1.9 times the average speed.'
  },
  DESCEND: {
    q: "Why does the gripper's x axis point along the ball's direction of travel?",
    options: ["So the two fingers end up on either side of the ball's path.",
      'To keep joint 1 from moving.', 'Because the camera needs it.'],
    answer: 0,
    why: "The fingers close along the gripper's y axis. With x along the motion, they form a gate the ball slides into."
  },
  WAIT: {
    q: 'The ball moves at 0.4 m/s and the fingers need 0.05 s to reach it. How far before the gripper centre should closing start?',
    options: ['0.5 cm', '2 cm', '8 cm'],
    answer: 1,
    why: 'distance = speed × time = 0.4 m/s × 0.05 s = 0.02 m.'
  },
  CLOSE: {
    q: 'The ball weighs 0.1 kg and μ = 1.5. What is the minimum squeeze force per finger?',
    options: ['0.33 N', '0.98 N', '1.47 N'],
    answer: 0,
    why: 'N_min = m·g / (2μ) = 0.1 × 9.81 / 3 ≈ 0.33 N. Two fingers share the weight, and each can hold μ times its normal force.'
  },
  LIFT: {
    q: 'The grip force is just above N_min, yet the ball slips during the lift. Why?',
    options: ['Lifting accelerates the ball upwards, so the fingers must carry more than its weight.',
      'Friction disappears as soon as things move.', 'The camera loses sight of the ball.'],
    answer: 0,
    why: 'During the lift the fingers must provide m·(g + a). That is why a safety factor is used. Find the slip threshold with the Grip force slider.'
  }
};

const PHASE_ORDER = ['OBSERVE', 'PLAN', 'APPROACH', 'DESCEND', 'WAIT', 'CLOSE', 'LIFT'];
const RESULT_COLOR = {'accepted': '#3ccf6a', 'too late': '#ff4d5e', 'no IK solution': '#c070e0', 'out of reach': '#666'};
const RUN_LIMIT = 50000;   // telemetry rows kept for CSV export
const HISTORY_SECONDS = 8;

// ═══════════════════════════════════════════════════════════════════════════
//  State
// ═══════════════════════════════════════════════════════════════════════════
const ui = {
  config: null,
  phase: 'IDLE',
  pausedNext: null,
  telemetry: [],
  plan: null,
  candidates: [],
  pathStart: null,
  velocity: null,
  ik: null,
  run: [],
  camera: null,
  quizKey: null,
  connected: false,
  dirty: false
};

const $ = (id) => document.getElementById(id);

// ═══════════════════════════════════════════════════════════════════════════
//  Messaging
// ═══════════════════════════════════════════════════════════════════════════
function send(cmd, data = {}) {
  try {
    window.robotWindow.send(JSON.stringify({cmd, ...data}));
  } catch (e) {
    // not connected yet: the hello timer below retries
  }
}

function receive(message) {
  let msg;
  try {
    msg = JSON.parse(message);
  } catch (e) {
    return;
  }
  switch (msg.type) {
    case 'hello': onHello(msg); break;
    case 'phase': onPhase(msg); break;
    case 'plan': onPlan(msg); break;
    case 'telemetry': onTelemetry(msg); break;
    case 'ik': onIk(msg); break;
    case 'log': addLog(msg); break;
    case 'camera': onCamera(msg); break;
    case 'exercises': onExercises(msg); break;
  }
}

function onHello(msg) {
  ui.connected = true;
  ui.config = msg;
  $('connection').textContent = 'connected';
  $('connection').className = 'good';
  setSlider('grip-force', msg.grip_force);
  setSlider('arm-speed', msg.speed_scale);
  document.querySelectorAll('input[name=perception]').forEach((r) => { r.checked = r.value === msg.perception; });
  $('teach').checked = msg.teach;
  updateFminHint();
  onPhase({phase: msg.state});
}

function onPhase(msg) {
  ui.phase = msg.phase;
  ui.pausedNext = msg.phase === 'PAUSED' ? msg.next : null;
  if (msg.phase === 'OBSERVE') {
    ui.telemetry = [];
    ui.run = [];
    ui.plan = null;
    ui.candidates = [];
    ui.pathStart = null;
  }
  $('next').disabled = msg.phase !== 'PAUSED';
  // Launching and the IK lab need the arm at rest at home.
  const idle = msg.phase === 'IDLE';
  $('launch').disabled = !idle;
  $('ik-go').disabled = !idle;
  $('launch').title = idle ? '' : 'Press Reset first: the arm must be back home';
  renderPipeline();
  renderLesson();
}

function onPlan(msg) {
  ui.plan = msg.plan;
  ui.candidates = msg.candidates || [];
  if (msg.p_now) ui.pathStart = msg.p_now;
  if (msg.v) ui.velocity = msg.v;
  renderPlanner();
}

function onTelemetry(msg) {
  ui.telemetry.push(msg);
  if (ui.run.length < RUN_LIMIT) ui.run.push(msg);
  const cutoff = msg.t - HISTORY_SECONDS;
  while (ui.telemetry.length && ui.telemetry[0].t < cutoff) ui.telemetry.shift();
  $('sim-time').textContent = `t = ${msg.t.toFixed(2)} s`;
  const fk = $('fk-check');
  fk.textContent = `FK check: ${msg.fk_error_mm.toFixed(1)} mm`;
  fk.className = msg.fk_error_mm < 5 ? 'good' : 'bad';
  if (!ui.dirty) {
    ui.dirty = true;
    requestAnimationFrame(() => {
      ui.dirty = false;
      renderLive();
    });
  }
}

function onIk(msg) {
  ui.ik = msg;
  renderIk();
}

function addLog(msg) {
  const li = document.createElement('li');
  li.className = msg.level;
  li.innerHTML = `<span class="t">${msg.t.toFixed(2)} s</span>${escapeHtml(msg.text)}`;
  $('log').prepend(li);
  while ($('log').children.length > 200) $('log').lastChild.remove();
}

function escapeHtml(s) {
  return s.replace(/[&<>]/g, (c) => ({'&': '&amp;', '<': '&lt;', '>': '&gt;'}[c]));
}

// ═══════════════════════════════════════════════════════════════════════════
//  Pipeline + lesson card
// ═══════════════════════════════════════════════════════════════════════════
function renderPipeline() {
  const current = ui.phase === 'PAUSED' ? ui.pausedNext : ui.phase;
  const idx = PHASE_ORDER.indexOf(current);
  const finished = ['CAUGHT', 'MISSED', 'DROPPED', 'NO_PLAN'].includes(ui.phase);
  document.querySelectorAll('#pipeline li').forEach((li, i) => {
    li.className = '';
    if (ui.phase === 'CAUGHT') li.classList.add('done');
    else if (finished) li.classList.add(i <= lastReached ? 'done' : 'failed');
    else if (idx >= 0) {
      if (i < idx) li.classList.add('done');
      else if (i === idx) li.classList.add(ui.phase === 'PAUSED' ? 'next' : 'active');
    }
  });
  if (idx >= 0) lastReached = idx;
  if (ui.phase === 'IDLE' || ui.phase === 'GO_HOME') lastReached = -1;
}
let lastReached = -1;

function renderLesson() {
  const key = ui.phase === 'PAUSED' ? ui.pausedNext : ui.phase;
  const lesson = LESSONS[key] || LESSONS.IDLE;
  $('lesson-phase').textContent = ui.phase === 'PAUSED' ? `PAUSED · next: ${ui.pausedNext}` : ui.phase;
  $('lesson-title').textContent = lesson.title;
  $('lesson-body').innerHTML = lesson.body +
    (ui.phase === 'PAUSED' ? '<div class="try"><b>Paused.</b> The ball is frozen. Read, look around the 3D view, then press <b>Next phase</b>.</div>' : '');
  $('lesson-code').textContent = lesson.code;
  const outcome = $('outcome');
  if (['CAUGHT', 'MISSED', 'DROPPED', 'NO_PLAN'].includes(ui.phase)) {
    outcome.className = ui.phase;
    outcome.textContent = {CAUGHT: '✔ Ball caught', MISSED: '✘ Missed', DROPPED: '✘ Dropped during lift',
      NO_PLAN: '✘ No feasible intercept'}[ui.phase];
  } else outcome.className = 'hidden';
  renderQuiz(key);
}

// ═══════════════════════════════════════════════════════════════════════════
//  Plot helpers
// ═══════════════════════════════════════════════════════════════════════════
const COLORS = {
  grid: '#333a45', text: '#9aa3ad', des: '#ffb02e', act: '#4da3ff', err: '#ff5c5c', ref: '#9aa3ad', good: '#3ccf6a'
};

function plot(canvas, series, opts = {}) {
  const ctx = canvas.getContext('2d');
  const W = canvas.width, H = canvas.height;
  const pad = {l: 46, r: 10, t: 10, b: 22};
  ctx.clearRect(0, 0, W, H);
  let xmin = opts.xmin, xmax = opts.xmax, ymin = opts.ymin, ymax = opts.ymax;
  const all = series.flatMap((s) => s.data);
  if (xmin === undefined) xmin = Math.min(...all.map((p) => p[0]));
  if (xmax === undefined) xmax = Math.max(...all.map((p) => p[0]));
  if (ymin === undefined) ymin = Math.min(...all.map((p) => p[1]));
  if (ymax === undefined) ymax = Math.max(...all.map((p) => p[1]));
  if (!isFinite(xmin) || !isFinite(ymin)) return;
  if (xmax - xmin < 1e-9) xmax = xmin + 1;
  if (ymax - ymin < 1e-9) { ymax += 0.5; ymin -= 0.5; }
  const ym = (ymax - ymin) * 0.08;
  ymin -= ym; ymax += ym;
  const X = (x) => pad.l + (x - xmin) / (xmax - xmin) * (W - pad.l - pad.r);
  const Y = (y) => H - pad.b - (y - ymin) / (ymax - ymin) * (H - pad.t - pad.b);

  ctx.strokeStyle = COLORS.grid;
  ctx.fillStyle = COLORS.text;
  ctx.font = '11px sans-serif';
  ctx.lineWidth = 1;
  for (let i = 0; i <= 4; i++) {
    const y = ymin + (ymax - ymin) * i / 4;
    ctx.beginPath(); ctx.moveTo(pad.l, Y(y)); ctx.lineTo(W - pad.r, Y(y)); ctx.stroke();
    const range = Math.abs(ymax - ymin);
    ctx.fillText(y.toFixed(range < 0.01 ? 4 : range < 0.1 ? 3 : 2), 2, Y(y) + 4);
  }
  for (let i = 0; i <= 4; i++) {
    const x = xmin + (xmax - xmin) * i / 4;
    ctx.fillText(x.toFixed(xmax - xmin < 2 ? 2 : 1) + (opts.xunit || ''), X(x) - 10, H - 6);
  }
  for (const s of series) {
    if (!s.data.length) continue;
    ctx.strokeStyle = ctx.fillStyle = s.color;
    ctx.lineWidth = s.width || 2;
    ctx.setLineDash(s.dash || []);
    if (s.points) {
      for (const p of s.data) {
        ctx.fillStyle = p[2] || s.color;
        ctx.beginPath(); ctx.arc(X(p[0]), Y(p[1]), p[3] || 3, 0, 2 * Math.PI); ctx.fill();
      }
    } else {
      ctx.beginPath();
      s.data.forEach((p, i) => (i ? ctx.lineTo(X(p[0]), Y(p[1])) : ctx.moveTo(X(p[0]), Y(p[1]))));
      ctx.stroke();
    }
    ctx.setLineDash([]);
  }
  if (opts.title) {
    ctx.fillStyle = COLORS.text;
    ctx.fillText(opts.title, pad.l + 6, pad.t + 12);
  }
}

// ═══════════════════════════════════════════════════════════════════════════
//  Planner tab
// ═══════════════════════════════════════════════════════════════════════════
function floorReachRadius() {
  const c = ui.config;
  if (!c) return 0.78;
  const dz = c.shoulder[2] - (c.ball_radius + c.wrist_offset);
  return Math.sqrt(c.wrist_reach ** 2 - dz ** 2);
}

function renderMap() {
  const canvas = $('map');
  const ctx = canvas.getContext('2d');
  const W = canvas.width, H = canvas.height, S = W / 2.8;     // ±1.4 m
  const P = (p) => [W / 2 + p[0] * S, H / 2 - p[1] * S];
  ctx.clearRect(0, 0, W, H);

  // axes
  ctx.strokeStyle = COLORS.grid; ctx.lineWidth = 1;
  ctx.beginPath(); ctx.moveTo(0, H / 2); ctx.lineTo(W, H / 2); ctx.moveTo(W / 2, 0); ctx.lineTo(W / 2, H); ctx.stroke();
  ctx.fillStyle = COLORS.text; ctx.font = '11px sans-serif';
  ctx.fillText('+x', W - 18, H / 2 - 4); ctx.fillText('+y', W / 2 + 4, 12);

  // reachable annulus
  const R = floorReachRadius(), r = ui.config ? ui.config.min_radius : 0.25;
  ctx.fillStyle = 'rgba(60, 207, 106, 0.18)';
  ctx.beginPath(); ctx.arc(W / 2, H / 2, R * S, 0, 2 * Math.PI); ctx.arc(W / 2, H / 2, r * S, 0, 2 * Math.PI, true); ctx.fill();
  ctx.fillStyle = '#444';
  ctx.beginPath(); ctx.arc(W / 2, H / 2, 0.08 * S, 0, 2 * Math.PI); ctx.fill();

  // predicted path
  if (ui.pathStart && ui.velocity) {
    const a = P(ui.pathStart), b = P([ui.pathStart[0] + ui.velocity[0] * 8, ui.pathStart[1] + ui.velocity[1] * 8]);
    ctx.strokeStyle = '#2fe0ff'; ctx.setLineDash([6, 4]); ctx.lineWidth = 1.5;
    ctx.beginPath(); ctx.moveTo(...a); ctx.lineTo(...b); ctx.stroke(); ctx.setLineDash([]);
  }
  // candidates
  for (const c of ui.candidates) {
    const [x, y] = P(c.p);
    ctx.fillStyle = RESULT_COLOR[c.result];
    ctx.beginPath(); ctx.arc(x, y, c.result === 'accepted' ? 7 : 2.5, 0, 2 * Math.PI); ctx.fill();
  }
  // live ball + TCP
  const last = ui.telemetry[ui.telemetry.length - 1];
  if (last) {
    let [x, y] = P(last.ball);
    ctx.fillStyle = '#ff7a1a'; ctx.beginPath(); ctx.arc(x, y, 0.03 * S + 1, 0, 2 * Math.PI); ctx.fill();
    [x, y] = P(last.tcp);
    ctx.strokeStyle = COLORS.act; ctx.lineWidth = 2; ctx.beginPath(); ctx.arc(x, y, 6, 0, 2 * Math.PI); ctx.stroke();
  }
}

function renderTiming() {
  const pts = ui.candidates.filter((c) => c.needed !== undefined)
    .map((c) => [c.t, c.needed, RESULT_COLOR[c.result], c.result === 'accepted' ? 6 : 3]);
  const tmax = Math.max(1, ...ui.candidates.map((c) => c.t));
  const ymax = Math.max(tmax, ...pts.map((p) => p[1]));
  plot($('timing'), [
    {data: [[0, 0], [tmax, tmax]], color: COLORS.ref, dash: [5, 4], width: 1.5},
    {data: pts, color: COLORS.des, points: true}
  ], {xmin: 0, xmax: tmax, ymin: 0, ymax, xunit: 's', title: 'time needed vs available (s)'});
}

function renderPlanner() {
  renderMap();
  renderTiming();
  const tbody = $('candidates').querySelector('tbody');
  tbody.innerHTML = ui.candidates.map((c) =>
    `<tr class="${c.result === 'accepted' ? 'accepted' : ''}"><td>${c.t.toFixed(2)}</td>` +
    `<td>${c.p[0].toFixed(3)}, ${c.p[1].toFixed(3)}</td><td>${c.needed !== undefined ? c.needed.toFixed(2) : '–'}</td>` +
    `<td style="color:${RESULT_COLOR[c.result]}">${c.result}</td></tr>`).join('');
  const p = ui.plan;
  $('plan-summary').innerHTML = !ui.candidates.length ? 'No plan yet: launch a ball.'
    : p ? `Meet the ball in <b>${p.t_intercept.toFixed(2)} s</b> at (${p.p_intercept[0].toFixed(3)}, ${p.p_intercept[1].toFixed(3)}) m · ` +
          `approach ${p.T_approach.toFixed(2)} s + descend ${p.T_descend.toFixed(2)} s · slack ${p.slack.toFixed(2)} s · ` +
          `${ui.candidates.length} candidates tried`
      : `<span style="color:var(--bad)">No feasible intercept among ${ui.candidates.length} candidates.</span>`;
}

// ═══════════════════════════════════════════════════════════════════════════
//  Motion + gripper tabs (live)
// ═══════════════════════════════════════════════════════════════════════════
function renderLive() {
  const tel = ui.telemetry;
  if (!tel.length) return;
  const active = document.querySelector('.tab.active').id;
  if (active === 'tab-planner') renderMap();
  if (active === 'tab-motion') {
    const j = Number($('joint-select').value);
    const lo = ui.config ? ui.config.q_min[j] : undefined, hi = ui.config ? ui.config.q_max[j] : undefined;
    plot($('joint-plot'), [
      {data: tel.map((m) => [m.t, m.q_des[j]]), color: COLORS.des, width: 3},
      {data: tel.map((m) => [m.t, m.q[j]]), color: COLORS.act, width: 1.5}
    ], {xunit: 's', title: `joint ${j + 1} (rad)  — limits [${lo?.toFixed(2)}, ${hi?.toFixed(2)}]`});
    plot($('error-plot'), [
      {data: tel.map((m) => [m.t, Math.max(...m.q.map((q, i) => Math.abs(m.q_des[i] - q)))]), color: COLORS.err}
    ], {xunit: 's', ymin: 0});
    renderJointBars(tel[tel.length - 1]);
  }
  if (active === 'tab-gripper') {
    const grip = Number($('grip-force').value);
    const fmin = ui.config ? ui.config.min_grip_force : 0.33;
    const t0 = tel[0].t, t1 = tel[tel.length - 1].t;
    plot($('finger-plot'), [
      {data: tel.map((m) => [m.t, m.fingers[0] * 100]), color: COLORS.des},
      {data: tel.map((m) => [m.t, m.fingers[1] * 100]), color: COLORS.act, width: 1},
      {data: [[t0, 3], [t1, 3]], color: COLORS.ref, dash: [5, 4], width: 1}
    ], {xunit: 's', ymin: 0, ymax: 4.2, title: 'finger opening (cm)'});
    plot($('force-plot'), [
      {data: tel.map((m) => [m.t, (m.finger_force[0] + m.finger_force[1]) / 2]), color: COLORS.act},
      {data: [[t0, grip], [t1, grip]], color: COLORS.ref, dash: [5, 4], width: 1},
      {data: [[t0, fmin], [t1, fmin]], color: COLORS.err, dash: [2, 3], width: 1}
    ], {xunit: 's', ymin: 0, title: 'finger force (N)'});
  }
}

function renderJointBars(m) {
  if (!ui.config) return;
  const box = $('joint-bars');
  if (!box.children.length) {
    box.innerHTML = m.q.map((_, i) =>
      `<div class="jbar">J${i + 1}<div class="track"><div class="fill"></div></div><div class="val"></div></div>`).join('');
  }
  [...box.children].forEach((el, i) => {
    const lo = ui.config.q_min[i], hi = ui.config.q_max[i];
    const frac = (m.q[i] - lo) / (hi - lo);
    el.querySelector('.fill').style.bottom = `${Math.min(Math.max(frac, 0), 1) * 87}px`;
    el.querySelector('.fill').style.background = frac < 0.05 || frac > 0.95 ? 'var(--bad)' : 'var(--accent)';
    el.querySelector('.val').textContent = m.q[i].toFixed(2);
  });
}

// ═══════════════════════════════════════════════════════════════════════════
//  IK lab
// ═══════════════════════════════════════════════════════════════════════════
function renderIk() {
  const r = ui.ik;
  if (!r) return;
  plot($('ik-plot'), [
    {data: r.history.map((e, i) => [i, Math.log10(Math.max(e, 1e-6))]), color: COLORS.act}
  ], {xmin: 0, title: 'log₁₀ position error (m) per iteration'});
  const q = r.q.map((v, i) => `<tr><td>q${i + 1}</td><td>${v.toFixed(3)}</td><td>[${ui.config.q_min[i].toFixed(2)}, ${ui.config.q_max[i].toFixed(2)}]</td></tr>`).join('');
  $('ik-result').innerHTML =
    `<p class="${r.ok ? 'ok' : 'fail'}">${r.ok ? '✔ solved' : '✘ no solution (outside the workspace?)'}</p>` +
    `<p>iterations: ${r.iterations} · seeds tried: ${r.seeds}<br>final error: ${(r.error * 1000).toFixed(2)} mm</p>` +
    `<table><tr><th>joint</th><th>rad</th><th>limits</th></tr>${q}</table>`;
}

// ═══════════════════════════════════════════════════════════════════════════
//  Camera tab
// ═══════════════════════════════════════════════════════════════════════════
const cameraBuffer = document.createElement('canvas');

function cameraTabShown() {
  return document.getElementById('tab-camera').classList.contains('active');
}

function onCamera(msg) {
  ui.camera = msg;
  if (cameraTabShown()) renderCamera();
}

function renderCamera() {
  const msg = ui.camera;
  if (!msg) return;
  const img = msg.image;
  const bytes = Uint8Array.from(atob(img.rgb), (c) => c.charCodeAt(0));
  cameraBuffer.width = img.w;
  cameraBuffer.height = img.h;
  const bctx = cameraBuffer.getContext('2d');
  const data = bctx.createImageData(img.w, img.h);
  for (let i = 0, j = 0; i < bytes.length; i += 3, j += 4) {
    data.data[j] = bytes[i]; data.data[j + 1] = bytes[i + 1]; data.data[j + 2] = bytes[i + 2]; data.data[j + 3] = 255;
  }
  bctx.putImageData(data, 0, 0);
  const canvas = $('camera-view');
  const ctx = canvas.getContext('2d');
  ctx.imageSmoothingEnabled = false;
  ctx.drawImage(cameraBuffer, 0, 0, canvas.width, canvas.height);
  const scale = canvas.width / msg.image_size[0];            // full-resolution pixels → canvas
  const d = msg.detection;
  if (d) {
    ctx.strokeStyle = '#2fe0ff'; ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.arc((d.u + 0.5) * scale, (d.v + 0.5) * scale, Math.max(6, msg.expected_diameter_px * scale), 0, 2 * Math.PI);
    ctx.stroke();
  }
  if (msg.predicted_px) {
    const [x, y] = msg.predicted_px.map((p) => (p + 0.5) * scale);
    ctx.strokeStyle = '#ffd400'; ctx.lineWidth = 2;
    ctx.beginPath(); ctx.moveTo(x - 8, y); ctx.lineTo(x + 8, y); ctx.moveTo(x, y - 8); ctx.lineTo(x, y + 8); ctx.stroke();
  }
  const f = (v, n = 3) => (v === undefined || v === null ? '–' : Number(v).toFixed(n));
  const err = d ? Math.hypot(d.p[0] - msg.ball_true[0], d.p[1] - msg.ball_true[1]) * 1000 : null;
  $('camera-info').innerHTML = `<table>
    <tr><td>perception used</td><td><b>${msg.perception}</b></td></tr>
    <tr><td>ball-coloured blobs</td><td>${d ? d.blobs : 0} (rejected ${d ? d.rejected : '–'})</td></tr>
    <tr><td>ball blob size</td><td>${d ? d.pixels : '–'} px (expected ≈ ${f(Math.PI * msg.expected_diameter_px ** 2 / 4, 0)})</td></tr>
    <tr><td>centre pixel (u, v)</td><td>${d ? `${f(d.u, 1)}, ${f(d.v, 1)}` : 'not visible'}</td></tr>
    <tr><td>camera says ball at</td><td>${d ? `${f(d.p[0])}, ${f(d.p[1])} m` : '–'}</td></tr>
    <tr><td>simulator truth</td><td>${f(msg.ball_true[0])}, ${f(msg.ball_true[1])} m</td></tr>
    <tr><td>camera error</td><td>${err === null ? '–' : f(err, 1) + ' mm'}</td></tr>
    <tr><td>robot's prediction</td><td>${msg.predicted ? `${f(msg.predicted[0])}, ${f(msg.predicted[1])} m` : '– (no estimate yet)'}</td></tr>
  </table>
  <p class="hint">The truth is shown only so you can judge the camera; the robot never uses it in camera mode.</p>`;
}

// ═══════════════════════════════════════════════════════════════════════════
//  Quiz
// ═══════════════════════════════════════════════════════════════════════════
function renderQuiz(key) {
  const quiz = QUIZZES[key];
  const box = $('quiz');
  if (!quiz) { box.className = 'hidden'; ui.quizKey = null; return; }
  box.className = '';
  if (ui.quizKey === key) return;                           // keep the learner's answer
  ui.quizKey = key;
  $('quiz-question').textContent = quiz.q;
  $('quiz-feedback').textContent = '';
  $('quiz-options').innerHTML = '';
  quiz.options.forEach((text, i) => {
    const b = document.createElement('button');
    b.textContent = text;
    b.onclick = () => {
      $('quiz-options').querySelectorAll('button').forEach((x) => { x.className = ''; });
      b.className = i === quiz.answer ? 'right' : 'wrong';
      $('quiz-feedback').innerHTML = (i === quiz.answer ? '<b style="color:var(--good)">Correct.</b> '
        : '<b style="color:var(--bad)">Not quite.</b> ') + quiz.why;
    };
    $('quiz-options').appendChild(b);
  });
}

// ═══════════════════════════════════════════════════════════════════════════
//  Exercises
// ═══════════════════════════════════════════════════════════════════════════
function onExercises(msg) {
  const icon = {pass: '✔', fail: '✘', error: '!', todo: '·'};
  const done = msg.results.filter((r) => r.status === 'pass').length;
  $('exercise-results').innerHTML =
    `<li class="${done === msg.results.length ? 'pass' : ''}"><b>${done} / ${msg.results.length} exercises correct</b></li>` +
    msg.results.map((r) => `<li class="${r.status}">${icon[r.status]} <b>Exercise ${r.n}: ${escapeHtml(r.title)}</b>` +
      ` — ${escapeHtml(r.message)}</li>`).join('');
}

// ═══════════════════════════════════════════════════════════════════════════
//  CSV export
// ═══════════════════════════════════════════════════════════════════════════
function download(filename, text) {
  const url = URL.createObjectURL(new Blob([text], {type: 'text/csv'}));
  const a = document.createElement('a');
  a.href = url;
  a.download = filename;
  document.body.appendChild(a);
  a.click();
  a.remove();
  setTimeout(() => URL.revokeObjectURL(url), 1000);
}

function runCsv() {
  const j = [1, 2, 3, 4, 5, 6, 7];
  const header = ['t', 'phase', ...j.map((i) => `q${i}`), ...j.map((i) => `q${i}_desired`),
    'tcp_x', 'tcp_y', 'tcp_z', 'ball_x', 'ball_y', 'ball_z', 'finger_left', 'finger_right',
    'force_left', 'force_right'];
  const rows = ui.run.map((m) => [m.t, m.state, ...m.q, ...m.q_des, ...m.tcp, ...m.ball,
    ...m.fingers, ...m.finger_force].join(','));
  return [header.join(','), ...rows].join('\n');
}

function planCsv() {
  const rows = ui.candidates.map((c) => [c.t, c.p[0], c.p[1], c.needed ?? '', c.result].join(','));
  return ['t,ball_x,ball_y,needed,result', ...rows].join('\n');
}

// ═══════════════════════════════════════════════════════════════════════════
//  Widgets
// ═══════════════════════════════════════════════════════════════════════════
function setSlider(id, value) {
  const el = $(id);
  el.value = value;
  el.parentElement.querySelector('output').textContent = Number(value).toFixed(el.step < 0.1 ? 2 : (el.step < 1 ? 1 : 0));
}

function bindSlider(id, onChange) {
  const el = $(id);
  const out = el.parentElement.querySelector('output');
  const fmt = () => { out.textContent = Number(el.value).toFixed(el.step < 0.1 ? 2 : (el.step < 1 ? 1 : 0)); };
  el.addEventListener('input', fmt);
  if (onChange) el.addEventListener('change', () => onChange(Number(el.value)));
  fmt();
}

function applyPreset(i) {
  const p = PRESETS[i];
  setSlider('ball-x', p.x);
  setSlider('ball-y', p.y);
  setSlider('ball-speed', p.speed);
  setSlider('ball-heading', p.heading);
}

function updateFminHint() {
  const fmin = ui.config ? ui.config.min_grip_force : 0.33;
  const f = Number($('grip-force').value);
  const hint = $('fmin-hint');
  hint.textContent = `Minimum to hold the ball: N_min = m·g/(2μ) = ${fmin.toFixed(2)} N  (you: ×${(f / fmin).toFixed(1)})`;
  hint.className = f < fmin ? 'hint warn' : 'hint';
}

function setupUi() {
  $('preset').innerHTML = PRESETS.map((p, i) => `<option value="${i}">${p.name}</option>`).join('');
  $('preset').addEventListener('change', (e) => applyPreset(Number(e.target.value)));
  ['ball-x', 'ball-y', 'ball-speed', 'ball-heading', 'ik-x', 'ik-y', 'ik-z', 'ik-yaw'].forEach((id) => bindSlider(id));
  applyPreset(2);
  $('preset').value = 2;
  bindSlider('arm-speed', (v) => send('arm_speed', {value: v}));
  bindSlider('grip-force', (v) => { send('grip_force', {value: v}); updateFminHint(); });
  $('grip-force').addEventListener('input', updateFminHint);

  $('launch').onclick = () => send('launch', {
    x: Number($('ball-x').value), y: Number($('ball-y').value),
    speed: Number($('ball-speed').value), heading: Number($('ball-heading').value)
  });
  $('reset').onclick = () => send('reset');
  document.querySelectorAll('input[name=perception]').forEach((r) => {
    r.onchange = () => send('perception', {value: r.value});
  });
  $('check-exercises').onclick = () => {
    $('exercise-results').innerHTML = '<li>checking…</li>';
    send('check_exercises');
  };
  $('export-telemetry').onclick = () => {
    if (!ui.run.length) return addLog({t: 0, level: 'warn', text: 'Nothing to export yet: launch a ball first.'});
    download('grasping_run.csv', runCsv());
  };
  $('export-plan').onclick = () => {
    if (!ui.candidates.length) return addLog({t: 0, level: 'warn', text: 'No plan yet: launch a ball first.'});
    download('planner_candidates.csv', planCsv());
  };
  $('next').onclick = () => send('next');
  $('teach').onchange = (e) => send('teach', {on: e.target.checked});
  $('markers').onchange = (e) => send('markers', {on: e.target.checked});
  $('ik-go').onclick = () => send('ik_goal', {
    x: Number($('ik-x').value), y: Number($('ik-y').value), z: Number($('ik-z').value), yaw: Number($('ik-yaw').value)
  });

  $('joint-select').innerHTML = [1, 2, 3, 4, 5, 6, 7].map((j) => `<option value="${j - 1}">joint ${j}</option>`).join('');
  $('joint-select').value = 1;

  document.querySelectorAll('#tabs button').forEach((b) => {
    b.onclick = () => {
      document.querySelectorAll('#tabs button, .tab').forEach((el) => el.classList.remove('active'));
      b.classList.add('active');
      $(`tab-${b.dataset.tab}`).classList.add('active');
      if (b.dataset.tab === 'planner') renderPlanner();
      if (b.dataset.tab === 'camera') renderCamera();
      send('camera_view', {on: cameraTabShown()});   // camera images are sent only while it is shown
      renderLive();
    };
  });
  renderLesson();
  renderPlanner();
}

// ═══════════════════════════════════════════════════════════════════════════
//  Start-up
// ═══════════════════════════════════════════════════════════════════════════
window.robotWindow = new RobotWindow();
window.robotWindow.receive = receive;
setupUi();
// Ask the controller for its configuration until it answers.
// The controller starts streaming telemetry once it gets this hello.
const sayHello = () => send('hello', {camera_view: cameraTabShown()});
const helloTimer = setInterval(() => {
  if (ui.connected) clearInterval(helloTimer);
  else sayHello();
}, 1000);
sayHello();
