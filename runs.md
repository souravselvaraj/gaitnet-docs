# Runs and checkpoints

Which run produced which checkpoint, from what, with which settings. Every run here trained
in Isaac Lab on the Turing cluster from the training code (see [training-setup.md](training-setup.md));
the settings below are read from each job's Slurm log (`cluster/logs/gaitnet-*-<job>.out`, first
`+ python` line). Iteration numbers continue across resumes, so "SC-idm@1590" is iteration 1590
counted from the start of the lineage, not of that job.

Checkpoints live in `gait-net-train/gaitnet/training/gaitnet/runs/gaitnet_<timestamp>_<run name>/model_<it>.pt`
(not in git). From 2026-10-12 22:05 EDT on, each checkpoint has a `curriculum_<it>.pt` beside it
with every robot's terrain level, and `--resume` restores it.

## Names

`<start><arch/recipe>[c][500]-<description>[-r<k>]`

| Part | Meaning |
| --- | --- |
| `W` | fine-tuned from the 1035 baseline (`random-sampling_1035.pt`) |
| `S` | trained from scratch |
| `B` | original GaitNet network (per-candidate MLP scorer, no history, `--arch gaitnet`), `step` recipe |
| `C` | GaitNet-T (`--arch gaitnet_t`), `step` recipe |
| `D` | GaitNet-T privileged teacher (`--recipe teacher --privileged`) |
| `c` | critic tail (`--critic_tail`) |
| `500` | 500-step rollouts x 4 minibatches (the others: 100 steps x 8 minibatches) |
| `-r<k>` | the k-th relaunch of the same run, resumed from one of its checkpoints |
| `-idm` | the training controller's inverse footstep model in the loop (`--idm_inverse`) |
| `-agnostic`, `-ahead`, `-mix` | controller randomisation for deployment on other controllers (see training-setup.md) |

**1035 baseline**: `gaitnet-ros/src/gaitnet_ros/gaitnet-models/gaitnet_20260112_201614-fe1b10f4-random-sampling_1035.pt`,
the original GaitNet policy trained before this work (CPU MPC, omni commands, Isaac Lab's
velocity curriculum). On the current eval it scores 50 % success at 10-30 % voids.

## The best policy: SC-idm@1590

GaitNet-T, step recipe, from scratch; the IDM was added at iteration 890. Segments:

| Iterations | Job | Run | Started from | What changed at the start of this segment |
| --- | --- | --- | --- | --- |
| 0-115 | 2361959 | SC-scratch-fast | scratch | GaitNet-T, step recipe, walk curriculum (p_down 0.5), omni commands, 100 x 8, `--critic_noop_fix`, tracking std 0.1 / 0.2 |
| 115-585 | 2362295 | SC-scratch-fast-r1 | r0@115 | `--walk_p_down 0.3` (the curriculum had sunk to level 0 for every run) |
| 585-645 | 2362922 | SC-scratch-fast-r2 | r1@585 | `--walk_random_fraction 0.25 --max_init_terrain_level 2` (r2 itself ran to 1103; 645 was its best eval) |
| 645-890 | 2363799 | SC-scratch-fast-r3 | r2@645 | `--commands forward` (the deployed robot only sees ahead) |
| 890-1590 | 2364455 | **SC-idm** | r3@890 | `--idm_inverse gaitnet_inverse.pt` |

So the 1590 checkpoint saw omni commands for 645 iterations and forward commands without the
IDM for 245 more before the IDM came in. `--preset sc_idm` is the final setting of the last
segment, trained from scratch (the seeds below).

Continuations of the same policy:

| Iterations | Job | Run | Started from | Change |
| --- | --- | --- | --- | --- |
| 1590-1614 | 2367013 | SC-idm-r2 | SC-idm@1590 | `--walk_p_up 0.2` (was 0.1) for every run; no curriculum files existed yet, so levels restarted low (3.65 -> 1.28) |
| 1610-2500 | 2367262 | SC-idm-r3 | r2@1610 | `--max_init_terrain_level 7` to start near the old level; from here on every checkpoint saves its curriculum |
| 1845-2500 | 2368602 | SC-idm-fast | r3@1845 | `--max_forward_speed 0.25`, `--lr_warmup_iterations 20` |

Isaac evals have shown no gain past ~1040 with the IDM, so later checkpoints are not
measurably better; 1590 is the one validated in Gazebo.

## Seeds of the final setting

`train.sbatch --preset sc_idm --seed k --max_iterations 1600`, from scratch, RTX PRO 6000,
~32 s per iteration. Code: gait-net `train-port` 18d4ed1 (gaitnet fork `train-port` 187c96c).

| Job | Run | Seed |
| --- | --- | --- |
| 2368817 | SC-idm-scratch-s1 | 1 |
| 2368818 | SC-idm-scratch-s2 | 2 |
| 2368819 | SC-idm-scratch-s3 | 3 |
| 2368820 | SC-idm-scratch-s4 | 4 |

Before 2026-10-13, `--seed` only seeded the GPU controller's randomisation; every earlier run
had an unseeded environment and network initialisation.

## The original-architecture line: WB500

GaitNet (the original per-candidate MLP scorer, no history), step recipe, 500-step rollouts x 4 minibatches, fine-tuned from the 1035 baseline.

| Iterations | Job | Run | Started from | Change / outcome |
| --- | --- | --- | --- | --- |
| 1035-1130 | 2361793 | WB500-walk-track | 1035 | walk curriculum, tracking std 0.1 / 0.2 |
| 1130-1276 | 2362288 | WB500-walk-track-r1 | @1130 | `--walk_p_down 0.3`; best eval at 1160 |
| 1275-1434 | 2362921 | WB500-walk-track-r2 | r1@1275 | exploration episodes, max init level 2 |
| 1160-1457 | 2363798 | WB500-walk-track-r3 | r1@1160 | `--commands forward`; PPO collapse between 1440 and 1455 |
| 1440-1600 | 2367807 | WB500-walk-track-r6 | r3@1440 | `--walk_p_up 0.2`; collapsed again at 1585-1588 |
| 1580-1900 | 2369294 | WB500-walk-track-r7 | r6@1580 | `--learning_rate 1e-4`; stable |

r4/r5 resumed from the collapsed 1455 checkpoint and were abandoned. Every WB500 collapse
happened at the default learning rate (3e-4). Of the three runs started from 1440 to separate
the IDM from the architecture, WB500-idm lost ~40 points in its first 10 full-rate updates,
WB500-T-idm collapsed as its 20-iteration warm-up ended, and WB500-idm-warm was stopped while
still warming up, so that comparison was never completed.

## Other arms (stopped or finished)

| Run | Started from | What it tested | Outcome |
| --- | --- | --- | --- |
| WB, WC (`-walk-fast-track`) | 1035 | GaitNet / GaitNet-T fine-tunes on 100 x 8 | WB declined below 1035; WC 0.48-0.55, never above WB500; cancelled |
| WD, WDc | 1035 | privileged GaitNet-T teacher fine-tunes | collapsed on 8 minibatches; WD flat below 1035; cancelled |
| SD, SDc (`-scratch-fast`) | scratch | teacher from scratch; SDc with critic tail | SDc best teacher (79 % at 10-30 % voids) but does not transfer to other controllers; stopped by the user |
| SB, SB500 (`-scratch`) | scratch | GaitNet from scratch | dominated / did not learn (SB500 stood still); cancelled |
| SC-agnostic | SC-r3@1140 | `--controller_randomization agnostic` (OCS2-like executors) | robust on the OCS2-like executor on flat ground, lost its void skill; stopped |
| SC-ahead | SC-agnostic@1445 | agnostic + steps executed 3-6 ticks after the decision | stopped with SC-agnostic |
| SC-agnostic-mix(-r2,-r3) | SC-agnostic@1300 | half the episodes on the step recipe's controller | finished at 2040; kept neither skill fully |
| SC-ahead-mix(-r2,-r3) | SC-agnostic@1300 | as above with commit-ahead | r3 stopped at 1380 (curriculum falling) |
| SC-agnostic-mix-idm | SC-agnostic-mix-r3@1365 | mix + IDM in the loop | finished at 2040 |
| SC-ahead-mix-idm | SC-ahead-mix-r3@1380 | ahead-mix + IDM in the loop | finished at 2045; best compromise of the mix runs |
| WB500-idm, WB500-idm-warm, WB500-T-idm | WB500-r3@1440 | IDM vs architecture (GaitNet / GaitNet-T) from one checkpoint | see above; all three stopped by the user |

A full table of all 72 training jobs (job id, run, start checkpoint, last iteration, flags) is
in [runs-all.tsv](runs-all.tsv).
