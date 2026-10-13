# Training setup

How GaitNet-T (SC-idm) is trained. Paths are relative to `gait-net-train/gaitnet/`. The setting
is reproducible with one flag, `train.py --preset sc_idm` (section 10). Architectures and the
observation are in [methodology.md](methodology.md); run history in [runs.md](runs.md).

## 1. Software and hardware

| Item | Value |
| --- | --- |
| Simulator | Isaac Sim 5.1, Isaac Lab 2.3.2 (isaaclab 0.54.2, git v2.3.2) |
| RL | rsl-rl-lib 3.1.2 (via isaaclab_rl 0.4.7) |
| Python / torch | 3.11 / 2.7.0+cu128 (from the install script; torch not re-checked) |
| Low-level MPC | `gaitnet_core.control` from the gaitnet repo at c79ab67, installed as a pinned snapshot (`cluster/setup_train_env.sh`) |
| Cluster | WPI Turing, Slurm; one GPU per run, 8 CPUs (the MPC runs on the GPU) |
| GPUs used | NVIDIA RTX PRO 6000 Blackwell (96 GB): the SC-idm lineage, the seeds, WB500. NVIDIA L40S (46 GB): SC-idm-r2/r3, SC-idm-fast |

**Caveat on code versions.** Every SC-lineage job ran gait-net commit 388cb9c plus 31-38
uncommitted files; the pipeline was committed afterwards as 415f9f6..18d4ed1. Only the four seeds
ran a clean committed tree (18d4ed1). Some features did not exist for the earlier segments:
curriculum carry-over, `--learning_rate`, `--max_forward_speed`, and `--seed` reaching the
environment.

## 2. Simulation

| Item | Value |
| --- | --- |
| Robot | Unitree Go1 (Isaac Lab `UNITREE_GO1_CFG`); DC motor model, torque control, 23.7 N·m, 30 rad/s, no actuator stiffness/damping |
| Physics | dt 0.004 s (250 Hz) |
| Policy | decimation 10, step 0.04 s (25 Hz) |
| MPC / leg control | MPC every 5 physics steps (50 Hz); leg torques every physics step (250 Hz) |
| Episode | 20 s = 500 policy steps |
| Robots | 1024 in parallel |
| Friction | robot static 1.0 / dynamic 0.9; terrain 1.0 / 1.0 (multiply) |

## 3. Terrain (`gaitnet/simulation/cfg/terrain.py`)

- **Layout**: 12 x 12 tiles of 4 x 4 m. Rows are difficulty levels, columns are independent
  random instances.
- **Voids**:
  - A tile is a 31 x 31 grid of 12.5 cm cells (0.025 m height field, x5).
  - Exactly `int(d x 961)` cells, chosen by uniform random permutation, are 0.5 m deep voids.
  - The central 1 x 1 m (8 x 8 cells) is kept solid as the spawn platform.
  - Height steps are vertical walls. Because the walls dilate by a pixel, the mesh voids are about
    1.25x their nominal area.
- **Level to void fraction**: row r has d = 0.5 x (r + U[0, 1)) / 12. Level L therefore covers
  void fractions [L/24, (L+1)/24), mean about (L + 0.5) x 4.2 %: L0 0-4 %, L11 46-50 %. The
  platform removes about 7 % of a tile's voids.
- **Spawn**: tile centre ±0.1 m, yaw uniform in ±π.

## 4. Commands

`UniformVelocityCommand`, resampled every U(2.5, 10) s; at each resample 5 % of robots get a
zero command. No heading command.

| Mode | vx (m/s) | vy (m/s) | Yaw rate (rad/s) | Used by |
| --- | --- | --- | --- | --- |
| `forward` | U(0, 0.2) | 0 | U(±0.2) | SC iterations 645 on, the preset, the seeds |
| `omni` | U(±0.2) | U(±0.2) | U(±0.4) | SC iterations 0-645 |
| `forward` + `--max_forward_speed 0.25` | U(0, 0.25) | 0 | U(±0.2) | SC-idm-fast |

Forward-only because the deployed robot maps terrain with a forward-facing camera; nothing beside
or behind it is known.

## 5. Rewards (the `step` recipe)

Every term is multiplied by its weight and by step_dt (0.04 s).

| Term | Definition | Weight | Per step |
| --- | --- | --- | --- |
| alive | 1 unless terminated by a non-time-out term | 0.4 | +0.016 (8.0 over a full episode) |
| xy tracking | exp(-‖c_xy - v_xy‖² / σ²), base frame, **σ = 0.1 m/s** | 1.0 | <= +0.04 |
| yaw tracking | exp(-(c_ω - ω_z)² / σ²), **σ = 0.2 rad/s** | 1.0 | <= +0.04 |
| footstep penalty | 1 when the action is not the no-op | -0.4 | -0.016 per footstep |
| termination | 1 when terminated by a non-time-out term | -200 | -8.0 per fall |

Notes:
- **Tracking widths.** Isaac Lab's default σ = 0.5 pays a robot that stands still 0.90 of the
  tracking reward when commands are at most 0.2 m/s. σ = 0.1 pays it 0.19.
- **No shaping.** There are no gait, foot-clearance, posture or smoothness terms.
- **Teacher recipe only** (not used by SC-idm): progress along the command (1.0), a void-edge
  touchdown penalty (-25 per foot landing within 5 cm of an invalid cell), and mild body
  stability (lin_vel_z -0.5, ang_vel_xy -0.05, flat_orientation -2.0).

## 6. Terminations (`gaitnet/gaitnet/env_cfg/terminations.py`)

| Term | Condition | Counts as a failure |
| --- | --- | --- |
| time_out | 20 s | no |
| bad_orientation | trunk tilt > 20 deg | yes |
| bad_height | trunk z < 0.15 m | yes |
| foot_below_ground | any foot z < -0.05 m (a foot in a void) | yes |
| terrain_out_of_bounds | beyond the terrain minus 0.5 m | no (treated as a time-out) |

## 7. Resets and observation noise

- **At each reset**: joints at the default pose scaled by U(0.45, 0.55), zero velocity. The
  controller state is reset and its randomisation resampled (section 8).
- **No disturbances**: external pushes are configured with zero range, so there are none, and
  there are no interval events or mass randomisation.
- **Observation noise is configured but disabled** (`enable_corruption = False`), so the policy
  trains on clean observations.

## 8. Controller randomisation

Each robot's controller is redrawn at every episode reset from the recipe's ranges, using the
controller's own RNG (seeded by `--seed`).

| Knob | Meaning | `step` recipe (SC-idm) | `agnostic` (other arms) |
| --- | --- | --- | --- |
| command_tracking | scale on the velocity the stance MPC tracks | U(0.3, 1.0) | U(0.3, 1.0) |
| planar_weight_scale | scale on the MPC's planar tracking weights | U(0.5, 1.0) | U(0.5, 1.0) |
| swing_apex | swing height above lift-off (m) | U(0.07, 0.12) | U(0.07, 0.12) |
| touchdown_bias | constant per-leg xy offset on footholds (m) | U(±0.02) per axis | U(±0.02) |
| touchdown_noise | std of fresh Gaussian noise per foothold (m) | 0.01 | U(0, 0.05) |
| liftoff_delay_ticks | ticks between decision and swing start | {0, 1} | {0, 1, 2} |
| travel_compensation | fraction of body travel removed from the target | 0 | U(0, 1) |
| touchdown_along / outboard | constant offsets (m) | 0 / 0 | U(-0.05, 0.02) / U(0, 0.05) |

Variants:
- `agnostic_ahead`: lift-off 3-6 ticks after the decision.
- `*_mix`: each episode uses the step recipe's controller with probability 0.5.

### Inverse footstep model in the loop (`--idm_inverse`)

Every chosen candidate is sent through the training controller's inverse model before the
controller adds its randomised bias and noise. The policy observes the raw candidates; the
history's touchdown error is measured against the model's output. Model:
`training/gaitnet/idm_models/gaitnet_inverse.pt` (methodology.md §6).

## 9. Terrain curriculum (`walk`)

Isaac Lab's velocity curriculum promotes a robot only past half a tile, which commands of at most
0.2 m/s never reach, so it kept every robot on level 0. The `walk` curriculum
(`gaitnet/gaitnet/env_cfg/curriculum.py`, `components/walk_curriculum.py`) runs at each episode
end.

**Distances:**
- walked = Σ (v_xy · ĉ) x 0.04, the projection on the command direction;
- commanded = Σ ‖c_xy‖ x 0.04.

**Progressed** if the commanded distance is < 0.2 m, or walked >= **0.3** x commanded. (Some help
texts say "half"; the code uses 0.3.)

**Moves:**
- Up with probability p_up after a time-out episode that progressed.
- Down with probability p_down after a failure, or after a time-out that did not progress.
- The share of failed or stalled episodes settles near p_up / (p_up + p_down).

| Setting | SC-idm (to 1590) | Later runs |
| --- | --- | --- |
| p_up | 0.1 | 0.2 (from 2026-10-12 21:45) |
| p_down | 0.3 (0.5 in the first 115 iterations) | 0.3 |
| Exploration episodes | 25 % on a uniformly random level; they do not move the robot's level | same |
| Initial level (each restart) | uniform 0..11 until iteration 585, then 0..2 (`--max_init_terrain_level 2`) | 0..2 for the preset; higher on later resumes |
| Above the top level | the robot is sent to a random level | same |
| Equilibrium failed/stalled share | 1/4 | 2/5 |

The logged `Curriculum/terrain_levels` is the mean curriculum level. SC-idm ended at 3.70
(about 17 % voids) at 1590; SC-idm-r3 reached ~6.4 (about 29 %) with p_up 0.2.

## 10. PPO (`gaitnet/gaitnet/train.py`)

| Item | Value |
| --- | --- |
| Algorithm | rsl-rl PPO, fixed learning-rate schedule |
| Rollout | 1024 robots x 100 steps = 102,400 transitions per iteration; episodes start at random lengths (`--init_at_random_ep_len`) |
| Update | 8 epochs x 8 minibatches of 12,800 = 64 gradient steps per iteration |
| Learning rate | Adam 3e-4 |
| Clip | 0.3; clipped value loss, coefficient 0.5 |
| Entropy coefficient | 0.02 |
| γ / λ | 0.99 / 0.95 |
| Max gradient norm | 1.0 |
| Normalisation | no observation normalisation; advantages normalised over the batch |
| Actor | GaitNet-T (methodology.md §5), evaluated in checkpointed chunks of 16,384 |
| Critic | GaitnetCritic with the no-op fix (`--critic_noop_fix`); it reads the same observation, without the history |
| Checkpoints | every 5 iterations, `training/gaitnet/runs/gaitnet_<timestamp>_<run name>/model_<it>.pt`, plus `curriculum_<it>.pt` (per-robot levels, restored by `--resume`) |

**Resuming and converting:**
- A GaitNet checkpoint resumed with `--arch gaitnet_t` is converted exactly (methodology.md §5).
  It gets a fresh Adam with a 20-iteration linear warm-up from 1 % of the rate.
- A from-scratch GaitNet-T is a converted fresh GaitNet.
- `--learning_rate` overrides the rate stored in a resumed optimiser. (It was added later for
  WB500; it is not in the commits pushed so far.)

**Stability:**
- The SC line never collapsed at these settings.
- The original-GaitNet runs with 500-step rollouts x 4 minibatches (WB500) collapsed three times
  at 3e-4 and are stable at 1e-4.
- The teacher runs collapsed on 8 minibatches until halved to 4.

## 11. Compute

| Configuration | GPU | s / iteration | Env steps / s |
| --- | --- | --- | --- |
| GaitNet-T, 1024 x 100 (SC lineage, seeds) | RTX PRO 6000 | 33-35 | ~3,000 |
| GaitNet-T, 1024 x 100 | L40S | 51-52 | ~2,000 |
| GaitNet, 1024 x 500 (WB500) | RTX PRO 6000 | 108-110 | ~4,700 |

Collection is about 55 % of a GaitNet-T iteration; the rest is the PPO update. WB500 needs about
70 GB of GPU memory and runs out on a 46 GB L40S.

**Budget behind SC-idm@1590**:
- 1,595 PPO updates.
- About 1.63 x 10⁸ environment transitions, about 1,815 h of simulated time. About 9.2 x 10⁷ of
  them came before the IDM was added.
- About 14.9 h of training on one RTX PRO 6000, excluding job start-ups.

## 12. The preset and the seeds

`--preset sc_idm` (train.py `PRESETS`) fills in:

```
--gpu_mpc --num_envs 1024 --arch gaitnet_t --recipe step
--idm_inverse <repo>/training/gaitnet/idm_models/gaitnet_inverse.pt
--curriculum walk --walk_p_down 0.3 --walk_random_fraction 0.25 --max_init_terrain_level 2
--commands forward --tracking_std 0.1 --yaw_tracking_std 0.2
--critic_noop_fix --num_steps_per_env 100 --num_mini_batches 8 --init_at_random_ep_len
```

Flags given on the command line override it. `--seed k` seeds the environment: Isaac Lab
seeds torch, numpy, random and warp from it, before the network is built. It also seeds the
controller's RNG. Before 2026-10-13 `--seed` reached only the controller, so every earlier run
had an unseeded environment and network initialisation.

**Reproduce one seed**, from `gait-net-train/gaitnet`:

```
sbatch -p short -q short -c 8 --mem=64G -t 1-00:00:00 --gres=gpu:rtx_pro_6000_b:1 \
  cluster/train.sbatch --preset sc_idm --seed 1 --max_iterations 1600 --run_name SC-idm-scratch-s1
```

**How the seeds differ from the path to 1590:**
- The seeds use the final setting from iteration 0.
- The 1590 lineage had 645 iterations on omni commands, then 245 on forward commands without the
  IDM.
- It started on levels 0-11 with p_down 0.5 for its first 115 iterations.
- Its curriculum was reset at each restart, and it was rolled back once (from 1103 to its best
  checkpoint at 645).
- It ran unseeded.
