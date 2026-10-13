# Methodology

What GaitNet-T is and how it chooses footsteps, as trained in the training repo and deployed on
OCS2. All facts are from the training code at gait-net `train-port` 18d4ed1 (paths below are
relative to `gait-net-train/gaitnet/gaitnet/`) and the batched MPC in the gaitnet repo
(`packages/gaitnet-core/src/gaitnet_core/control/`, pinned at c79ab67).

> **Do not cite `gaitnet/ARCHITECTURE.md` for this method.** It documents a rewritten pipeline
> (gaitnet-sim on Isaac Lab 3: 25 x 25 grid at 1.5 cm, K = 64, reach band, edge rule, minimum
> stance time, 52-dim state). None of that is what GaitNet-T was trained with. Only the batched
> MPC (`gaitnet_core.control`) is shared between the two codebases.

## 1. Overview

GaitNet plans footsteps **without a gait**. Every policy tick (25 Hz):

1. Sample up to K = 16 valid footholds for each leg that may lift. Together with a no-op this
   makes a candidate set of 65 slots.
2. A network scores every slot given the robot's state and recent motion, and outputs a swing
   duration per slot.
3. The chosen slot becomes one footstep command: a leg, a foothold relative to that leg's hip,
   and a swing duration. A no-op leaves the schedule unchanged.
4. A convex MPC executes the footstep schedule and tracks the velocity command.

At most one new footstep starts per tick. Because a leg may only lift while 3 legs are in
stance, at most two legs swing at once. Which legs step, when, and for how long is decided step
by step; no periodic gait exists anywhere in the loop. On held-out Gazebo worlds the trained
policy pairs its swings diagonally (it trots) without being told to (results.md §6).

## 2. Robot and low-level controller

**Robot.** Unitree Go1 with torque-controlled joints (DC motor model, 23.7 N·m, 30 rad/s; no
joint stiffness or damping in the actuator) (`simulation/cfg/robot.py`).

**Convex MPC** (single-rigid-body model, Di Carlo et al. 2018; `gaitnet_core/control/srbd.py`,
`mpc.py`):

| Item | Value |
| --- | --- |
| State | roll, pitch, yaw, position, angular velocity, linear velocity, gravity term (13) |
| Decision variables | foot forces over the horizon (condensed dense QP); only the first step is applied |
| Horizon | 10 steps x 0.02 s = 0.2 s, solved at 50 Hz (every 5 control steps) |
| Constraints | friction pyramid with μ = 0.8; normal force in [0.1, 10] x m·g; zero force on swing legs |
| Model ("gaitnet" variant, what every policy trained on) | mass 10.408 kg; inertia diag (0.0168, 0.0630, 0.0717) x 5 kg·m² |
| Weights | (10, 15, 0, 0, 0, 500, 0, 0, 1, 10, 10, 1, 0) on the 13 states; force regularisation 1e-5 |
| Solver | batched ADMM (OSQP-style) on the GPU, 80 iterations per solve, warm-started; ρ update every 20 iterations |

The GPU solver reproduces the CPU controller it replaces (rl-mpc-locomotion, qpOASES): the worst
torque difference at 80 iterations is 0.20 N·m (0.6 %), typically 0.025 N·m. It deliberately
keeps that controller's quirks, for example about 22 % excess normal force at standstill.

**Leg control** at 250 Hz (every physics step):
- Swing legs: Cartesian PD with Kp (700, 700, 150) N/m and Kd 7 N·s/m.
- Stance legs: Kd 14 N·s/m plus the MPC force as feed-forward; torque = Jᵀ F.

**Footstep execution** (`sim2real/gpu_mpc.py`, `gaitnet_core/control/gait.py`):
- **Lift-off and touchdown.** A commanded swing starts at the next physics step (lift-off at the
  decision). Touchdown is scheduled at start + T, not measured.
- **Target.** Every control step the target is recomputed as hip + (dx, dy), relative to the
  *current* hip. The foot therefore lands where the hip will be at touchdown plus the offset: about
  v x T past the chosen spot.
- **Swing arc.** In xy, a cubic Bézier from the lift-off position to the target. In z, two
  half-swing Béziers to an apex above the lift-off point (8.7 cm nominal).

This execution convention is what ties a policy to its controller (section 6).

## 3. Footstep candidates

**Per-leg scan** (`simulation/cfg/footstep_scanner.py`, `env_cfg/observations.py`):
- **Rays.** One ray caster per hip, yaw-aligned (gravity-aligned, rotating with the base yaw),
  41 x 41 rays at 1 cm. Updated every policy tick.
- **Validity.** A cell is valid if the terrain under it is 0 to 0.5 m below the hip. A missed ray
  or a void (0.5 m deep pits) is invalid.
- **C-space.** A 5 x 5 max-pool (stride 1, no padding) turns the 41 x 41 validity map into the
  **37 x 37 candidate grid** covering ±0.18 m about the hip. Every invalid cell grows by 2 cells,
  so the robot never chooses a cell within 2 cm of a void.
- **No other reach, edge, slope or kinematic rules** exist in this code; the reachable set is the
  ±0.18 m square.

**Eligibility** (`components/footstep_candidate_sampler.py`):
- A leg in scheduled swing cannot be chosen.
- If fewer than 3 legs are in scheduled stance, no leg can be chosen. The tick is then a forced
  no-op, so at most two legs swing at once.
- Legs waiting out a lift-off delay count as lifted.
- There is no minimum stance time.

**Sampling.** `random_footstep_sampling = True` throughout:
- For each eligible leg, K = 16 valid cell centres are drawn uniformly without replacement, with
  no sub-cell jitter.
- A leg with fewer than 16 valid cells gets padding slots.
- The no-op is always the last slot.
- (The original ContactNet cost-map sampler is not used, and the per-candidate cost feature is
  ablated to 0.)

Slot layout: 0-15 FR, 16-31 FL, 32-47 RR, 48-63 RL, 64 no-op.

## 4. Observation

A flat vector of 1195 values: `[robot state 25 | candidates 65 x 8 | history 25 x 26]`
(`components/gaitnet_observation_manager.py`). The original GaitNet reads only the first 545
(no history). Observation noise is configured but disabled (`enable_corruption = False`).

**Robot state (25):**

| Index | Feature | Frame and units |
| --- | --- | --- |
| 0-7 | foot xy relative to the trunk | trunk frame, m; legs FL, FR, RL, RR |
| 8 | base height | world z, m (absolute, not terrain-relative) |
| 9-11 | base linear velocity | base frame, m/s |
| 12-14 | base angular velocity | base frame, rad/s |
| 15-17 | velocity command (vx, vy, yaw rate) | base frame |
| 18-21 | scheduled contact (from the controller, not sensed) | legs FR, FL, RR, RL |
| 22-24 | projected gravity | base frame |

The leg order differs inside the state: the feet are FL, FR, RL, RR, while contact (and every
slot-indexed quantity below) is FR, FL, RR, RL.

**Candidate (8 per slot):**

| Feature | Value |
| --- | --- |
| 0-4 | one-hot (no-op, FR, FL, RR, RL) |
| 5-6 | (dx, dy): the foothold relative to the leg's hip, yaw-aligned frame, m |
| 7 | cost; always 0 (ablated) |

Padding and the no-op carry the no-op one-hot with zero offsets.

**Policy history (GaitNet-T): 25 ticks (1 s) x 26 features** (`components/policy_history.py`).
It is a ring buffer, oldest first, zeroed at reset.

| Index | Feature |
| --- | --- |
| 0-2 | base linear velocity, base frame |
| 3-5 | base angular velocity, base frame |
| 6-13 | foot xy relative to its hip, body frame (from joint kinematics) |
| 14-17 | scheduled contact |
| 18-25 | last-touchdown error per leg: foot position at the scheduled touchdown minus the commanded hip-frame foothold (0 before a leg's first touchdown) |

Everything in it is available on the robot: odometry twist, encoder kinematics, and the gait
node's own schedule and commands. With the IDM in the loop, "commanded" means the IDM's output
(section 6).

## 5. Policy

**Action distribution** (`gaitnet.py`):
- A masked categorical over the 65 slots. Padding logits are set to -inf; the real no-op is always
  allowed.
- Per slot, a Gaussian over the swing duration: mean = 0.1 + 0.2 x sigmoid(head), so T is in
  (0.1, 0.3) s. Its std is fixed at 0.01.
- The joint log-probability is the categorical term plus the chosen slot's Gaussian.
- The sampled duration is not clamped.

**Deployment rule:** argmax of the masked logits, with the chosen slot's mean duration.

**GaitNet** (the original network; `gaitnet.py`), about 99k parameters. Every candidate is scored
independently given the state:
- State encoder: MLP 25 → 128 → 128 → 128.
- Candidate encoder: MLP 8 → 64 → 64, shared by all slots, plus a learned no-op embedding.
- Per-slot trunk: [state ‖ candidate] 192 → 128 → 128 → 128.
- Heads: a logit head and a duration head, each Linear 128 → 1.

**GaitNet-T** (`gaitnet_t.py`), about 464k parameters. It keeps GaitNet's interface and decision
rule, but candidates attend to each other and the context includes the motion history.
- **Context**: the state encoder output plus a zero-initialised projection of a history encoder
  (MLP 650 → 128 → 64 → Linear 64 → 128).
- **Tokens**: Linear(192 → 128) + ReLU on [context ‖ candidate embedding], with the no-op
  embedding for no-op slots.
- **Candidate mixer**: 2 pre-norm transformer encoder layers, model dim 128, 4 heads, feed-forward
  128 → 256 → 128 (GELU). Padding slots are masked as keys; the real no-op is visible. The
  attention output projection and the last feed-forward layer are zero-initialised, so the mixer
  starts as the identity.
- **Readout**: MLP 128 → 128 → 128, then GaitNet's logit and duration heads.
- **Conversion**: `GaitnetTActor.from_gaitnet_actor` copies a GaitNet's weights into this layout.
  Because of the zero initialisations, the converted network computes **exactly** the original's
  outputs; this is tested, and logged at conversion. A fresh GaitNet-T is built the same way from
  a fresh GaitNet.
- The actor runs in activation-checkpointed chunks of 16,384 samples on PPO minibatches.

**Critic** (`gaitnet.py`, GaitnetCritic). It reads the same observation but not the history.
- State encoder 25 → 64 → 64 → 64; candidate encoder 8 → 32 → 32.
- Per-slot trunk 96 → 64 → 64 → 64 → 1. The 65 slot values go through an MLP 65 → 32 → 32 → 1 to
  give V(s).
- **No-op fix** (`--critic_noop_fix`, used by SC-idm). Earlier checkpoints' critics read one-hot
  column 4 (the RL leg) as the no-op, so every RL candidate was encoded as a no-op. The fix reads
  column 0.
- An optional asymmetric "critic tail" reading the history (and privileged features) exists; it
  is not used by SC-idm.

## 6. Inverse footstep model (IDM)

**Why.** Because of the execution convention in section 2, a policy learns to aim short of where
it wants the foot to land (the foot lands about v x T past the target). That makes its footholds
specific to one controller. The IDM separates "where the policy wants the foot" from "what to
send this controller".

**Definitions** (`gaitnet/idm.py`; after FADA, Xie et al., arXiv 2606.28476):
- A command u = (dx, dy) relative to the hip in the yaw frame at decision time, with duration T.
- The landing o = touchdown position in the yaw frame of the hip when commanded.
- Forward model F: (s, leg, T, u) → o. Inverse model I: (s, leg, T, o) → u.
- **Literal mode** (used): send u' = I(s, leg, T, u_chosen), i.e. treat the chosen candidate as
  where the foot should land. (The "faithful" composition u' = I_B(s, F_A(s, u)) was tested and
  is worse.)

**Network.** Input 33 = the newest history tick (26) + leg one-hot (4) + T + xy (2). MLP with two
256-wide ELU layers, output a residual on xy. The last layer is zero-initialised, so an untrained
IDM is the identity.

**Fit**:
- Optimiser: Adam, lr 1e-3, 60 epochs, batch 4096, 10 % held out, MSE loss.
- The inverse model is fitted by hindsight relabelling: input = landing, target = command sent.
- **Data**: 180,681 footsteps logged on the training controller without randomisation, from two
  policies (SC@645 and SDc@1035). Voids 0-0.3, speeds 0.05-0.2 m/s, with 3 cm Gaussian noise added
  to every command so the data covers more than the policies' own choices.
- Held-out error: 0.89 cm RMS, against 2.61 cm for the identity.

**In training** (`--idm_inverse`):
- Every chosen candidate goes through I before the controller (which then adds its randomised
  bias and noise).
- The policy still observes the raw candidates, and its history's touchdown error is measured
  against the IDM output.
- The IDM was fitted on the un-randomised controller but is applied under the step recipe's
  controller randomisation.

**At deployment** (Gazebo/OCS2) the same model is applied by the GaitNet service ("Isaac IDM").
An IDM fitted on OCS2's own landings was also built and tested (experiment-setup.md §2).

## 7. Deployment pipeline (summary)

Details and parameters are in [experiment-setup.md](experiment-setup.md) §2-3.

1. **Perception → terrain.** Depth camera → depth-vote terrain: walkable regions at 1 cm.
2. **Gait node.** Builds each leg's 37 x 37 C-space at 1 cm, matching the training grid.
3. **GaitNet service.** Samples the 65-slot candidate set, rebuilds the 25 x 26 history with the
   training code's own feature function, runs masked argmax, and applies the IDM.
4. **Gait node → OCS2.** Turns the decision into a mode schedule announced 0.2 s ahead on the MPC
   clock, with the target shifted by v x (lead + T) so it lands where training would have put it.
5. **OCS2.** SQP MPC + weighted whole-body control execute the schedule.

## 8. Code facts that contradict docstrings

These are worth knowing before quoting a docstring in the paper.

- `robot_controllers.py` says footholds are "in the body frame"; the scan rays are yaw-aligned
  (gravity-aligned hip frame), as `gaitnet_core/control/controller.py` says.
- `GaitnetActor.act_inference` takes an argmax over *unmasked* logits; the deployed rule
  (`eval_policy.act`) masks padding.
- The sampler comment "require 3 down to leave a 3-point support polygon" does not match the code's
  effect: with 3 down, one more may lift, leaving 2.
- Some docstrings still say "0-16" options or "(N, 17, 4)"; the policy outputs (slot index 0-64,
  duration).
- The CPU controller path solves its QP with qpOASES, though some comments say OSQP.
- The `sc_idm` preset's docstring says the 1590 policy spent 890 iterations on omni commands
  before the IDM. In fact it was 645 iterations on omni commands, then 245 on forward commands
  without the IDM ([runs.md](runs.md)).
