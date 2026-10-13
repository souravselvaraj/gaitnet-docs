# GaitNet docs

Working notes for the GaitNet-T paper: what the method is, how it is trained, how it is
evaluated, and what has been measured so far. Written from the code and the experiment logs, with
pointers to both, so a claim in the paper can be traced back to where it came from.

| File | Contents |
| --- | --- |
| [methodology.md](methodology.md) | The planner: candidates, observation, GaitNet / GaitNet-T, the low-level MPC, the inverse footstep model, the deployment pipeline |
| [training-setup.md](training-setup.md) | Simulator, environment, rewards, terminations, curriculum, controller randomisation, PPO, compute, the preset and seeds |
| [experiment-setup.md](experiment-setup.md) | Isaac evaluation protocol, Gazebo + OCS2 protocol, perception configurations, benchmarks, statistics |
| [results.md](results.md) | Every result so far, with its source; negative results; open items |
| [runs.md](runs.md) | Which run produced which checkpoint, the lineage of the best policy, naming |
| [runs-all.tsv](runs-all.tsv) | All 72 training jobs with their settings |

## The policy the paper is about

**SC-idm@1590**: GaitNet-T (attention over candidate footholds plus a 1 s motion history),
trained with PPO in Isaac Lab on void terrain, with the training controller's inverse footstep
model in the loop.
- File: `gait-net-train/gaitnet/training/gaitnet/runs/gaitnet_20261012_185605_SC-idm/model_1590.pt`.
- Its setting is `train.py --preset sc_idm`, which four seeds are reproducing from scratch.

## Code

| What | Where | Commit |
| --- | --- | --- |
| Training, evaluation, cluster scripts | gait-net monorepo, `gaitnet/` (worktree `/scratch/sselvaraj/projects/gait-net-train`), branch `train-port` | 18d4ed1 (local) |
| The same code on GitHub | [souravselvaraj/gaitnet `train-port`](https://github.com/souravselvaraj/gaitnet/tree/train-port) | 187c96c |
| Batched convex MPC (`gaitnet_core.control`) | souravselvaraj/gaitnet `training-sim2real` | c79ab67 |
| Deployment (ROS gait node, GaitNet service, OCS2, perception, Gazebo worlds) | gait-net monorepo (`/scratch/sselvaraj/projects/gait-net`) | branch `paper-eval` |
| Experiment log | `gait-net-train/gaitnet/cluster/watcher/decisions.md` | not in git |
| Isaac eval outputs | `gait-net-train/gaitnet/training/gaitnet/evals/*/summary.json` | not in git |
| Gazebo sweep outputs | `/scratch/sselvaraj/gaitnet-runs/idm/` | not in git |

## Conventions

- **Leg names**: FL, FR, RL, RR (front/rear, left/right).
  - The controller orders legs FL, FR, RL, RR.
  - The policy's candidate slots and per-leg features use FR, FL, RR, RL.
  - The robot state's foot positions use FL, FR, RL, RR.
  - methodology.md says which order applies where.
- **Units**: SI. Rates: physics 250 Hz, policy 25 Hz (one tick = 0.04 s), MPC 50 Hz.
- **Success** (evaluation): survived and walked >= half the commanded distance, in both Isaac and
  Gazebo. The training curriculum's promotion rule is different: 0.3 of the commanded distance
  (training-setup.md §9).
- **Iterations** count from the start of a lineage across resumes, so "SC-idm@1590" means
  iteration 1590 of the SC line.
- **Void fractions**: the share of terrain cells that are voids. Isaac cells are 12.5 cm and
  0.5 m deep; Gazebo tiles are 24 cm and 0.3 m deep.

These notes describe the state of 2026-10-13. Runs still training are marked as such.
