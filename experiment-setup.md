# Experimental setup

How every number in [results.md](results.md) is produced. There are two tiers:

1. **Isaac Lab evaluation.** Fast and deterministic, with hundreds of robots per condition.
   It runs the policy on the training controller, on the training controller behind the
   inverse footstep model (IDM), and on an "OCS2-like" emulated executor.
2. **Gazebo + OCS2 evaluation.** The full deployment stack: a Go1 in Gazebo, OCS2's
   nonlinear MPC and whole-body controller, the GaitNet ROS service, and either the exact
   terrain ("oracle") or a simulated RealSense D435 with the perception pipeline.

Paths are relative to the training repo `gait-net-train/gaitnet` unless they start with
`gait-net/` (the deployment monorepo) or `gaitnet-runs/` (Gazebo sweep outputs). The research
log behind every decision is `cluster/watcher/decisions.md` ("DEC" below).

## 1. Isaac Lab evaluation (`gaitnet/gaitnet/eval_policy.py`)

### Protocol

| Item | Value |
| --- | --- |
| Void fractions | 0, 0.1, 0.2, 0.3, 0.4 (default); the periodic eval adds 0.5 |
| Commanded speeds | 0.1 and 0.2 m/s, straight ahead, body frame, never resampled, no heading correction |
| Robots per cell | 150 |
| Strips per cell | 30 by default; 15 in the periodic eval (see below) |
| Strip | 10 m x 1 m `hole_terrain`; voids 0.5 m deep; 1 m x 1 m solid platform around the spawn |
| Void cells | 12.5 cm (0.025 m height field upsampled x5); exactly `int(d * N)` cells drawn by random permutation |
| Episode | 20 s = 500 policy ticks (25 Hz policy, 250 Hz physics) |
| Spawn | strip origin, facing +x, no pose randomisation; robots do not collide with each other |
| Controller | GPU convex MPC, `gaitnet` variant (the one every policy trained on), no randomisation |
| Seeding | `random`, `numpy`, `torch`, the terrain generator and the controller RNG seeded with `--seed` (0); torch and env reset reseeded before each checkpoint |

Each void fraction x speed pair is a **cell**; cells own contiguous blocks of strips. The
realised void fraction outside the platform is recorded per strip in `summary.json`
(`generated_void_fraction`), e.g. 0.098, 0.196, 0.304, 0.401, 0.499 for the nominal levels.

**Why 15 strips in the periodic eval.** With 0.5 added, 6 voids x 2 speeds x 30 strips makes a
5.9 M-vertex mesh, over the 5 M that PhysX can cook. Two eval cycles died on it. 15 strips keeps
150 robots per cell (10 per strip).

### Terminations and metrics

| Term | Condition |
| --- | --- |
| `bad_orientation` (tip-over) | trunk tilt > 20 deg |
| `bad_height` | trunk z < 0.15 m |
| `foot_below_ground` (void fall) | any foot z < -0.05 m |
| `time_out` | 20 s; leaving the terrain (0.5 m buffer) also counts as a time-out, not a fall |

Metrics, per cell:

- **Survival**: no termination other than time-out.
- **Distance fraction**: forward displacement / (v x 20 s).
- **Success**: survived and distance fraction >= 0.5. This is the main metric, usually averaged
  over the 10-30 % void cells and both speeds ("10-30 % band").
- **95 % Wilson intervals** on survival and success.
- **Steps per tick**: the share of ticks with a footstep (not a no-op).
- **Falls by cause**, and **landing error** (touchdown minus the chosen candidate's world point,
  in the decision's yaw frame).

### Inference rules

The policy outputs a categorical distribution over 65 slots: 16 candidates per leg x 4 legs plus
the no-op, with padding masked.

- **argmax**: take the most likely slot. This is the deployed rule and the main metric.
- **marginal**: step if the candidates' total probability exceeds the no-op's, then take the most
  likely candidate.
- **sample**: draw from the distribution, as in training.

Sampling was worse for every trained run, and marginal only helps low-entropy policies a little,
so argmax is reported (DEC 2026-10-12 07:00).

### Executors

Each checkpoint is run on three executors, matching the three periodic eval jobs.

1. **Training controller**, plain.
2. **Training controller + IDM**: every chosen foothold u is replaced by I(s, leg, T, u), where I
   is the inverse footstep model fitted to this controller. Only the command changes; the
   policy's observation does not.
3. **OCS2-like executor** (`--ctrl`, no IDM): the training controller with fixed knobs that
   reproduce what OCS2 does to GaitNet's footsteps in Gazebo.

| Knob | Value | Effect | Basis |
| --- | --- | --- | --- |
| `travel_compensation` | 1 | target is relative to the hip when commanded (OCS2's convention), not at touchdown | OCS2 anchors footholds in the world at command time |
| `touchdown_along` | -0.04 m | constant offset along the walking direction | Gazebo landings 3-5 cm short |
| `touchdown_outboard` | +0.04 m | constant outboard offset (side-signed) | Gazebo landings 3-5 cm outboard |
| `touchdown_noise` | 0.05 m | Gaussian noise on every foothold | Gazebo landing sd 4-7 cm per axis |
| `liftoff_delay_ticks` | 1 (40 ms) | the controller starts the swing one tick late | OCS2 lifts 40-50 ms after the command |

A milder "matched" set (along -0.01, noise 0.04, travel compensation 1, delay 2) appears in the
SC-vs-OCS2 investigation report.

### Schedules and extra evals

- **Periodic eval** (`cluster/watcher/watch.sh`): every 2 h, the latest checkpoint of every watched
  run on all three executors, voids 0-0.5, 15 strips per cell. Results are in
  `training/gaitnet/evals/<date_time>{,-idm,-emu}/summary.json`; `cluster/watcher/eval_report.py`
  prints the band summaries.
- **Hard eval** (`evals/hard_idm_20261013`): SC-idm at 1040/1265/1485/1650/1760 and WB500 at 1440,
  with the IDM, at 40 and 50 % voids only.
- **Lookahead** (`eval_lookahead.py`): the same protocol, comparing argmax against depth-1/2
  re-scoring by predicted landing on a privileged edge map (rays every 1 cm against the mesh).
- **IDM data** (`--dump_events --perturb_xy 0.03`): every landed step (state, leg, T, intent,
  command sent, landing) from SC@645 and SDc@1035 on the training controller. Voids 0-0.3, speeds
  0.05-0.2 m/s, 60 robots per cell, seed 1; about 180k footsteps.

### Caveats

- Terrain is seeded per cell index: changing the cell list or strip count changes the terrain.
  **Compare checkpoints within one eval only.** The 04:41 2026-10-13 eval and later use 15 strips.
- In one eval process, runs after the first reshuffle per-robot outcomes: compare cell rates (they
  agree within ±0.06), not paired robots.
- Mesh voids are about 1.25x their nominal area (walls dilate by one pixel).
- Reruns of the same configuration differ by up to 0.04-0.06.

## 2. Gazebo + OCS2 evaluation

### Worlds (`gait-net/legged_control/legged_gazebo/scripts/gen_gridvoid_world.py`)

| Item | Value |
| --- | --- |
| Layout | grid of 0.24 m tiles, each a 0.30 m pillar; a void is a real 30 cm drop to a visible ground plane |
| Voids | each tile is a void with probability = density (Bernoulli), so the realised fraction varies per world; first tile row always solid |
| Strip | 6 m long, abs(y) <= 0.84 m; start pad 0.7 m behind and 0.5 m ahead of the spawn, voids from ~0.38 m ahead |
| Densities | 0, 0.1, 0.2, 0.3, 0.4 |
| Physics | ODE, 1 ms step, real-time factor 1 |
| Seeds | 110 per density |

The tiles are 24 cm, not Isaac's 12.5 cm, because the D435 pipeline cannot resolve 12 cm voids.
12.5 cm worlds exist but are only used with oracle terrain.

**Seed roles.** State these explicitly in the paper:

| Seeds | Use |
| --- | --- |
| 51-60 | tuning the OCS2/GaitNet deployment configuration |
| 61-110 | collecting footsteps for the OCS2 IDM (does not affect the Isaac-IDM arm) |
| 1-20 | the frozen held-out benchmark; also the gait comparison |
| 1-40, 21-40, 41-80, 81-110 | perception development (1-40) and confirmation blocks |

The perception rounds were developed on 1-40, so their pooled results over 1-110 are
"confirmed on new seeds", not strictly held out from the whole study.

### Trial (`gait-net/apptainer/gazebo_trial.sh`)

1. Start Gazebo, load the controller with the world, switch controllers, and wait for the planar
   terrain (up to 90 s).
2. Stand-up: zero command until the trunk is level (tilt < 6 deg) and >= 0.25 m above the
   surface for 1 s, 30 s timeout.
3. Command (v, 0, 0) on `/cmd_vel` at 10 Hz. Ground truth is recorded at 50 Hz for the trial
   duration in sim seconds.
4. Duration: **30 s** for the frozen benchmark, all perception rounds and the gait comparison;
   20 s in earlier sweeps. Infrastructure failures are retried up to 3 times.

Metrics (`gait-net/apptainer/analyze_gazebo.py`, `gaitnet-runs/idm/inv/loop/bench_score.py`,
`gaitnet-runs/idm/inv/ahead/gait_metrics.py`):

- **Terminations**: Isaac's thresholds applied to the world (tilt > 20 deg; trunk < 0.15 m above
  the surface; any foot > 0.05 m below the surface), plus leaving the strip and the
  controller's safety check.
- **Success**: survived and distance >= 0.5 x v x duration, with 95 % Wilson intervals.
- **Gait metrics**, measured the same way for every arm from ground-truth foot heights (a foot is
  "up" when 1.5 cm above its rolling 0.5 s minimum): steps/s, steps/m, swing time, feet in the
  air, pair types, and the no-op share.
- **No-foothold share**: GaitNet requests with >= 3 feet down and 0 valid candidates, before
  termination.

**Survival differs from Isaac.** In Gazebo leaving the strip ends the trial as not survived; in
Isaac it counts as a time-out.

### How GaitNet drives OCS2

| Stage | Setting (final configuration) |
| --- | --- |
| Gait node (`legged_robot_gait_command`, GaitNet mode) | requests a decision at 25 Hz; keeps requesting while 2 legs swing, so the history gets a tick every 40 ms |
| Candidates | C-space of 37 x 37 cells at 1 cm (±0.18 m about each hip) with a 3 cm probe margin, slightly more conservative than training's 2-cell (2 cm) dilation of invalid cells; 16 random candidates per leg + no-op = 65, as in training |
| Policy | GaitNet service: masked argmax; GaitNet-T history of 25 x 26 rebuilt from requests with training's own `tick_features`; no-op and zero history until the first non-zero command |
| IDM | "Isaac IDM": u' = I(s, chosen candidate) with the training controller's inverse model. ("OCS2 IDM", fitted on Gazebo footsteps, was tested and not used in the final arms.) |
| Announce-ahead | every step starts 0.20 s after the decision (`gaitnet_long_swing_lead 0.20`, travel threshold 0) |
| Schedule delivery | `gait_absolute_times`: schedules on the MPC clock, merged with `GaitSchedule::replaceScheduleFrom` instead of re-anchoring a one-shot template per footstep; `gait_insert_lead 0.04` s |
| Foothold anchoring | target shifted by v x (lead + T) (`anchor_at_touchdown`), since training places the foot relative to the hip at touchdown |
| Foothold region | `foothold_authority_radius 0.03` m (default 0.10), chosen for the lowest landing RMS |
| Body reference | command integrated into a pose reference capped 0.15 m ahead of the body |

OCS2 settings:
- SQP MPC, 1 iteration, dt 0.015 s, horizon 0.8 s, 100 Hz; weighted WBC.
- `task_stock`: task.info with six values restored to the stock configuration (lift-off velocity
  0.05, swing height 0.08, swing xy tracking weight 0, mu 0.1, no measured contact, slip
  threshold 0).
- `task_stock_gate`: `task_stock` with `slipVelocityThreshold 0.05`.
- `reference_stance`: hip ab/adduction ±0.05 rad, CoM height 0.3 m.
- Tested and not adopted: 3 or 5 SQP iterations, a 0.2 s horizon.

### Terrain sources

- **Oracle** (`gaitnet_oracle_terrain:=true`): the world file's exact geometry is published as the
  planar terrain to both the controller and the gait node. The camera chain still runs, so the
  CPU load is the same.
- **Camera**: section 3. A split test can give the controller and the gait node different
  sources.

## 3. Perception (camera experiments)

**Sensor.** Simulated RealSense D435 (`librealsense_gazebo_plugin`): depth 424 x 240, HFOV
85.2 deg, configured 10 Hz (about 5 Hz delivered, measured), 0.2-5 m. No depth noise (Gazebo does
not apply it to depth). Mounted at base x 0.31 m, z 0.052 m, pitched 0.870 rad. Frames arrive
stamped about one frame late, so the chain re-stamps each frame to the previous frame's stamp
+ 6 ms.

**Baseline chain**, used in rounds 0 to ~15: voxel grid (5 mm), an angular-rate cloud gate, CPU
elevation mapping (1 cm cells, later 2 cm), then convex plane decomposition (2 cm cells,
unobserved-border erosion 2). The resulting planar terrain feeds both the GaitNet C-space and
OCS2's foothold regions and SDF.

**Final terrain: depth-vote terrain** (`depth_vote_terrain.py`):
- **Grid**: a fixed odometry-frame grid of 1 cm cells. Each depth frame projects cell centres at
  the walkable level into the image.
- **Votes**: a measured point within 3 cm of the level is a hit; a point below it is a miss.
  Evidence decays 0.7 per frame, and a cell is walkable while its evidence is > 0.
- **Margins**: a walkable cell also needs clearance > base margin + drift margin x (metres
  travelled since the cell was last seen).
- **Frame handling**: frames are placed at the previous frame's stamp + 6 ms; frames after a gap
  of more than 0.30 s are dropped.
- **Outputs**: a controller copy (eroded 2 cells, outlines simplified at 2 cm) and an uneroded
  gait copy.

| Final setting | Value |
| --- | --- |
| Base margin | 0.01 m |
| Polygon epsilon | 0.02 m |
| Controller erode | 2 cells |
| Drift margin | 0.01 m/m with leg odometry + slip gate; 0 with tracking odometry |

**Odometry variants:**

| Variant | What it is |
| --- | --- |
| Leg odometry (default) | OCS2's Kalman filter estimate. Its drift came from a hardware config loaded in sim (`dead_contact_feet [RF_FOOT, RH_FOOT]`): those feet's contact was taken from the MPC schedule while they were still landing, under-reading forward travel by 2-3.4 %. |
| Leg odometry + slip gate | `task_stock_gate` (`slipVelocityThreshold 0.05`) skips feet whose kinematic velocity disagrees with the estimate: forward ratio 0.999-1.000, error over 1 m p50 0.08 cm. Valid on the real Go1 too. |
| Tracking camera (T265/VIO stand-in) | `gt_tracking_odom.py`: true base position at 50 Hz, optionally with a 1 % scale error |

## 4. Benchmarks and comparisons

**Frozen benchmark** (`gaitnet-runs/idm/inv/loop/bench.py`):
- **Policy**: SC-idm@1590 + Isaac IDM, on the configuration in section 2.
- **Trials**: voids 10/20/30 % at 0.1 m/s (60 trials) and flat ground at 0.2 m/s (20 trials) on
  seeds 1-20, 30 s each. `SEEDS=a-b` reuses the same cells on other seed blocks.
- **Adoption rule**: a change is adopted only if its Wilson interval separates from the
  incumbent's (or on pooled runs when close), one variable at a time.
- **Target**: >= 80 % on voids (raised from 70 %) and >= 7/10 on flat ground at 0.2 m/s.

**Gait comparison** (`inv_gaits_1`): oracle terrain, seeds 1-20, voids 10/20/30 % at 0.1 m/s,
30 s, 60 trials per arm.

| Arm | Definition |
| --- | --- |
| GaitNet multileg | up to 2 concurrent swings, any pair |
| GaitNet trot-only | 2 swings only as a diagonal pair |
| GaitNet single-swing | 1 swing at a time |
| OCS2 trot | OCS2's own gait (LF_RH / RF_LH, 0.3 s swings, 0.6 s period) and own footholds, no GaitNet, stock task |

**Policy variants on Gazebo** (`inv_voids_3`): oracle terrain, tuning seeds 51-60, 10/20/30 % at
0.1 m/s, 30 trials per arm. Arms: SC-idm@1590 with or without the IDM, SC@645, SC-r3@1595,
SC-agnostic@1410 (with or without the OCS2 IDM), SC-ahead@1510, WB500-r3@1455 (a collapsed
checkpoint), and OCS2's trot.

**Isaac-side ablations** (`reports/IDM and lookahead metrics.md`, 150 robots per cell):
- Controllers: training ("gaitnet") against "gaitnet_core" (other tuning).
- Footstep mapping: GaitNet alone, + literal IDM, + faithful IDM (FADA composition), + lookahead
  d1/d2.
- Executor sensitivity: the OCS2-like knobs one at a time (`knob_ablation`), and history ablations
  (none, zeroed, twist zeroed, Gazebo-like twist noise).

## 5. Statistics

- 95 % Wilson score intervals (z = 1.96) on every success and survival rate, in both tiers.
- Gazebo arms are compared on pooled seed blocks where possible: n = 330 for the final perception
  arms (seeds 1-40, 41-80, 81-110), with a two-proportion test against the control.
- Isaac cells have 150 robots, so a single cell's interval is about ±5-8 points at 70-90 %
  success.

## Not verified

- The exact depth rate delivered in the final perception arm. About 5 Hz was measured earlier.
- Which of the two IDM-data jobs produced the final event files. Both used the same protocol.
- Whether a doubled `ocs2_task` launch argument in the final arm resolved to `task_stock_gate`.
  Roslaunch takes the last value, and the measured odometry confirms the gate was active.
- Exact Gazebo and ROS versions.
