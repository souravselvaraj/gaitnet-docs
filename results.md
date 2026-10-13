# Results so far

State on 2026-10-13, ~10:00 EDT. Every number is traceable to an eval directory under
`gait-net-train/gaitnet/training/gaitnet/evals/` or to the dated entry in
`cluster/watcher/decisions.md` (DEC). Protocols are in [experiment-setup.md](experiment-setup.md);
run names in [runs.md](runs.md).

Unless noted, "success" = survived and walked >= half the commanded distance, Isaac cells have
150 robots, and "10-30 % band" averages the 10/20/30 % void cells at both 0.1 and 0.2 m/s.

## 1. Headline

- **Isaac.** GaitNet-T trained with the IDM in the loop (SC-idm) succeeds on 99 % of runs over
  10-30 % voids, about 88 % at 40 % voids and about 75 % at 50 % voids (training controller + IDM).
  The 1035 baseline scores 50 % on the 10-30 % band.
- **Gazebo + OCS2, held-out worlds.**
  - With exact terrain: 82 % [70, 89] on voids (seeds 1-20), against OCS2's own trot at 77 %.
  - With the camera + leg odometry + slip gate: 80.3 % [76, 84] (n = 330).
  - With the camera + tracking-camera odometry: 85-87 %, matching exact terrain (87 % on the same
    worlds).

## 2. The executor mismatch and the inverse footstep model (Isaac)

**Overshoot.** Every policy's foot lands past the chosen foothold on the training controller:
+1.3 to 1.9 cm along the walking direction at 0.1 m/s, and +2.9 to 3.9 cm at 0.2 m/s. The
controller executes the target against the hip at touchdown (DEC 2026-10-12 17:30). Policies
learn around it, which ties them to that controller.

**Literal IDM** (`evals/idm_A_literal`, training controller), success at 10-30 % voids,
0.1 / 0.2 m/s:

| Policy | No IDM | + IDM | 40 % voids + IDM |
| --- | --- | --- | --- |
| 1035 baseline | 0.82 / 0.17 | 0.94 / 0.93 | 0.86 / 0.81 |
| WB500 (GaitNet) | | 0.97 / 0.98 | 0.92 / 0.89 |
| SC (GaitNet-T) | | 1.00 / 0.98 | 0.97 / 0.89 |
| SDc (privileged teacher) | | 0.92 / 0.95 | 0.83 / 0.55 |

On a differently tuned controller (`gaitnet_core`) with the literal IDM, SC keeps 0.97 / 0.94
while WB500 drops to 0.75 / 0.75 and the teacher to 0.66 / 0.74, with 424 tip-overs. The
"faithful" (FADA) composition is worse everywhere, because it recreates the overshoot.
**GaitNet-T transfers across controllers; the teacher does not** (DEC 17:45).

**IDM in training vs only at deployment.** SC-idm (trained with the IDM) and SC-r3 (IDM only at
eval) are within noise on success: 0.99/0.98 against 0.99/0.97 at 10-30 %, 0.95/0.87 against
0.93/0.84 at 40 %. Training with it adds about 0.06 distance (DEC 21:00).

**Lookahead.** Depth-1 re-scoring on a privileged edge map lifts SC at 30 % voids, 0.2 m/s from
0.17 to 0.87. The non-privileged IDM is better at 40 % voids, 0.2 m/s: 0.89 against 0.73
(DEC 18:30).

## 3. Policies compared (Isaac, eval `20261013_0843`)

Success, %. Columns are the 10-30 % band and the 40 % and 50 % void cells for the first two
controllers, then flat ground and the band for the OCS2-like executor.

| Run (iteration) | + IDM 10-30 | + IDM 40 | + IDM 50 | Plain 10-30 | Plain 40 | Plain 50 | OCS2-like flat | OCS2-like 10-30 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| SC-idm-r3 (2355) | **99** | 87 | **76** | 64 | 45 | 33 | 70 | 2 |
| SC-idm-fast (2355) | **99** | **90** | 72 | 64 | 45 | 31 | 72 | 1 |
| WB500-r7 (1730) | 97 | 85 | 71 | 62 | 36 | 26 | **96** | 3 |
| SC-idm-scratch-s2 (665) | 97 | 78 | 61 | 64 | 32 | 22 | 71 | 1 |
| SC-idm-scratch-s1 (690) | 93 | 72 | 58 | 52 | 26 | 27 | 51 | 1 |
| SC-idm-scratch-s4 (605) | 91 | 65 | 50 | 48 | 29 | 16 | 73 | 0 |
| SC-idm-scratch-s3 (615) | 77 | 54 | 36 | 42 | 16 | 12 | 33 | 0 |

Earlier evals of the same 15-strip terrain show the same ordering:

| Eval | SC-idm-r3 (+IDM 10-30 / 40 / 50) | SC-idm-fast | WB500-r7 |
| --- | --- | --- | --- |
| 20261013_0441 (it 2070 / 2070 / 1600) | 99 / 88 / 72 | 98 / 90 / 77 | 95 / 82 / 64 |
| 20261013_0642 (2210 / 2210 / 1665) | 99 / 89 / 74 | 99 / 92 / 79 | 94 / 79 / 64 |
| 20261013_0843 (2355 / 2355 / 1730) | 99 / 87 / 76 | 99 / 90 / 72 | 97 / 85 / 71 |

**SC-idm stopped improving early.** In the hard eval (`hard_idm_20261013`, with the IDM), 40 %
and 50 % voids at 0.1 / 0.2 m/s:

| Checkpoint | 40 % | 50 % |
| --- | --- | --- |
| SC-idm@1040 | 87 / 83 | 89 / 58 |
| SC-idm@1265 | 91 / 86 | 93 / 57 |
| SC-idm@1485 | 91 / 91 | 91 / 66 |
| SC-idm@1650 | 92 / 87 | 95 / 56 |
| SC-idm@1760 | 93 / 87 | 92 / 64 |
| WB500@1440 (never trained with the IDM) | 86 / 86 | 71 / 53 |

No checkpoint after 1040 is better beyond the Wilson intervals. The weak cell is 50 % voids at
0.2 m/s, and those failures are mostly feet in voids.

**Seeds of the final setting** (`--preset sc_idm`, from scratch). Success with the IDM over the
10-30 % band, at each eval:

| Seed | 04:41 | 06:42 | 08:43 |
| --- | --- | --- | --- |
| s1 | 95 (it 260) | 92 (475) | 93 (690) |
| s2 | 95 (250) | 90 (460) | 97 (665) |
| s3 | 77 (190) | 98 (400) | 77 (615) |
| s4 | 95 (185) | 95 (395) | 91 (605) |

The setting reaches >= 90 % within ~200-400 iterations from scratch. The original lineage needed
about 1000. Between checkpoints the seeds swing by up to ±20 points (s3); their training curves
stay smooth. They are still training, to 1600 iterations.

**Faster commands** (SC-idm-fast, forward commands up to 0.25 m/s instead of 0.2): level with
SC-idm-r3 on the 10-30 % band, and ±5 points at 40-50 % voids depending on the eval. There's no
consistent gain yet.

## 4. Robustness to other controllers (Isaac, OCS2-like executor)

- **On voids**, every policy is at 0-9 % on the OCS2-like executor (landing noise 5 cm, 4 cm
  offsets, 1-tick delay). On the knobs one at a time (`knob_ablation`, 10-30 % voids at 0.1 m/s),
  5 cm of landing noise alone takes SC-r3 to 0.07 and WB500-r3 to 0.16; a 4 cm outboard offset
  alone takes them to 0.38 and 0.48.
- **On flat ground**, the policies differ:

  | Policy | OCS2-like flat-ground success |
  | --- | --- |
  | WB500 (GaitNet, no history) | 91-97 % |
  | SC-agnostic@1270 | 93 / 91 % at 0.1 / 0.2 m/s |
  | SC-idm | 34-75 %, varying between checkpoints with no clear trend |

  WB500 raises its step rate when its feet trail (4.0 to 5.0 steps/s); SC already steps 7-9/s
  and does not (DEC 17:45).
- **Controller-randomised training** (SC-agnostic, SC-ahead and their 50 % "mix" variants) bought
  OCS2-like robustness on flat ground but lost void skill on the training controller (SC-agnostic
  0.94 to 0.55 at 10-30 % voids, 0.1 m/s). The mix runs ended at 66-84 % with the IDM, against
  SC-idm's 99 % (`20261013_0441`).
- **History is not the cause.** Zeroing SC's history on the training controller keeps it walking
  (flat 0.97 / 0.93, 10-30 % at 0.1 m/s 0.85). On the OCS2-like executor zeroing does not rescue
  it. Zeroed touchdown-error features cost nothing (DEC 17:45, 21:00).

## 5. Training stability

- **Teacher collapses.** WD and WDc (100-step rollouts x 8 minibatches) collapsed in one update
  (tipping 0.1 to 0.4-0.85). Halving the minibatches stopped it.
- **WB500 collapses.** WB500 (500 x 4) collapsed at the default learning rate 3e-4 three times:
  r3 at 1440-1455, r6 at 1585-1588, and WB500-T-idm at the end of its warm-up. It is stable at
  1e-4 (r7).
- **No collapse in the SC line** (100 x 8, 3e-4) in any run, the seeds included.
- **Curriculum.** Isaac Lab's velocity curriculum kept every robot on level 0. The walk curriculum
  with p_down 0.5 sank to levels 0.1-0.4 (~3 % voids). p_down 0.3, exploration episodes and p_up
  0.2 bring SC-idm to ~29 % mean voids.

## 6. Gazebo + OCS2

### Deployment gap: what it was and was not

1. **Not the history encoder.** It is computed correctly in Gazebo; zeroing it does not help.
2. **Landing error.** OCS2 lands GaitNet's feet 2-4 cm short and about 2 cm outboard, with 4-5 cm
   scatter per axis, against +0.6 to +2.9 cm overshoot and 1.1 cm scatter in training. It also lifts
   40-50 ms late and truncates the swing to ~0.12 s. No OCS2 weight brings the scatter near 1 cm
   (DEC 18:15, 18:35).
3. **Our task.info changes** broke OCS2 itself at 0.15-0.2 m/s. Restoring six stock values
   (`task_stock`) gives OCS2's own trot 10/10 at 0.2 m/s (DEC 20:00).
4. **Schedule delivery was the main bug.** The gait node's GaitNet mode re-inserted a one-shot
   schedule at every footstep, cutting swings already under way. Delivered this way, OCS2's own
   trot survived 4/40, against 40/40 through keyboard mode. The fix (`gait_absolute_times`, steps
   announced 0.2 s ahead) took SC-agnostic from 7/10 to 18/20 at 0.15 m/s on flat ground
   (DEC 20:45, 21:30).
5. **The camera** was the main remaining failure on voids: 7-20 % with the camera against
   70-93 % with exact terrain (DEC 22:10, 22:20).

### Policies through the fixed stack, exact terrain (`inv_voids_3`, tuning seeds 51-60, 0.1 m/s, n = 30)

| Arm | Success on 10/20/30 % voids |
| --- | --- |
| SC-idm@1590 + Isaac IDM | **93 %** (10/9/9 of 10) |
| SC-idm@1590, no IDM | 87 % |
| SC-agnostic@1410 | 77 % |
| SC@645 | 77 % |
| OCS2's own trot | 77 % |
| SC-r3@1595 | 73 % |
| SC-agnostic@1410 + OCS2 IDM | 73 % |
| SC-ahead@1510 | 70 % |
| WB500-r3@1455 (collapsed checkpoint, not representative) | 40 % |

With exact terrain on flat ground (n = 10 per speed, 0.15 / 0.2 m/s): SC-ahead 10/10, 10/10;
SC-agnostic + OCS2 IDM 10/10, 10/10; SC@645 9/10, 9/10; SC-idm + Isaac IDM 10/10, 7/10
(DEC 22:20).

### Gait comparison (`inv_gaits_1`, exact terrain, held-out seeds 1-20, 10/20/30 % at 0.1 m/s, n = 60 per arm)

| Arm | Success | Falls | Steps/s | Steps/m | Swing | Feet in the air |
| --- | --- | --- | --- | --- | --- | --- |
| GaitNet multileg (SC-idm@1590 + IDM) | **82 %** [70, 89] | 4 tip + 7 void | 4.3 | 42 | 0.11 s | 0.49 |
| GaitNet trot-only | 82 % | | | | | |
| GaitNet single-swing | 0 % (58 tip-overs within ~5 s) | | | | | |
| OCS2's own trot | 77 % [65, 86] | 0 tip + 14 void | 6.7 | 64 | 0.22 s | 1.45 |

The multileg policy already trots: of its swing pairs, 2874 were diagonal, 15 rear, 8 same-side
and 0 front. It no-ops on 83 % of ticks. The same policy got 93 % on the tuning worlds and 82 %
on held-out ones. (Gait metrics are presumably the multileg arm; check
`gaitnet-runs/idm/inv_gaits_1_gait_metrics.csv`.)

### Camera (frozen benchmark, SC-idm@1590 + IDM)

Success on 10/20/30 % voids at 0.1 m/s, and flat ground at 0.2 m/s:

| Stage | Voids | Flat 0.2 | n (void trials) |
| --- | --- | --- | --- |
| Default camera chain | 8 % [4, 18] | 0/20 | 60 |
| 2 cm map + 1.5 m submap | 17 % [9, 28] | 0/20 | 60 |
| + map handling fixes, drift-free odometry | 55 % [42, 67] | 7/20 | 60 |
| Camera-only, rate gate off | 45 % [33, 58] | 7/20 | 60 |
| Mask terrain (pooled) | 48 % | 66/80 | 240 |
| Depth-vote terrain (pooled) | 67 % | 46/60 | 180 |
| Vote terrain tuned: eps 0.02, controller erode 2, drift 0.04 (pooled) | 73 % | | 330 |
| **+ estimator slip gate, drift margin 0.01 (leg odometry)** | **80.3 % [76, 84]** (92/81/68) | 96/110 | 330 |
| Tracking-camera stand-in, margin 0 (pooled) | 85 % [80, 88] | | 330 |
| Tracking camera with 1 % scale drift | 87 % [79, 92] | 39/40 | 120 |
| Exact terrain, same worlds and load | 87 % | 18/20 | 60 |

**Split test** (n = 80; DEC round 8). The controller's terrain drives the flat-ground tip-overs,
GaitNet's terrain drives the void falls:

| Controller terrain | GaitNet terrain | Voids | Flat 0.2 |
| --- | --- | --- | --- |
| Camera | Camera | 37 % | 7/20 |
| Exact | Camera | 47 % | 16/20 |
| Camera | Exact | 60 % | 14/20 |
| Exact | Exact | 87 % | 18/20 |

The leg-odometry fix: the estimator in sim took two feet's contact from the schedule, a hardware
setting meant for the real robot's dead contact sensors. The slip gate brings odometry error over
1 m from 2.84 cm to 0.08 cm (median). The remaining gap to the tracking camera (80 against 85 %)
is not drift.

## 7. Negative and null results worth reporting

- Sampling or marginal inference instead of argmax: no better (sampling worse for every run).
- Privileged teacher + distillation: the best teacher (SDc, 79 %) did not transfer to other
  controllers; stopped.
- Fine-tuning GaitNet-T from the 1035 GaitNet (WC): never above WB500.
- Controller-randomised training: robust on flat ground, lost void skill.
- Tuning OCS2's swing and posture weights: landing scatter stays 4-5 cm.
- 3-5 SQP iterations, a shorter MPC horizon, a longer announce lead: no gain.
- A constant per-leg landing-bias correction in Gazebo: no gain.
- Forecasting GaitNet's next leg from the current policy: right only 21-35 % of the time; it
  changes its preferred leg on 51 % of ticks.
- Training with the IDM against adding it at deployment: equal success.

## 8. Open items before the paper's claims are final

- **Architecture.** A fair GaitNet (WB500) against GaitNet-T comparison in Gazebo has not been
  run; the only WB500 Gazebo numbers are from the collapsed 1455 checkpoint. In Isaac the two are
  close with the IDM, and WB500 is more robust on the OCS2-like executor. The controlled
  WB500-idm / WB500-T-idm runs were stopped.
- **Seeds.** The four seeds finish at 1600 iterations around 17:00-18:00 on 2026-10-13. Report
  their final spread and the per-checkpoint variance.
- **Gazebo with newer checkpoints** (round 31: SC-idm-r3@2420, SC-idm-fast@2425 on the final
  camera configurations) is running.
- **Speed.** 0.2 m/s with the camera and 50 % voids at 0.2 m/s in Isaac are the weakest cells.
