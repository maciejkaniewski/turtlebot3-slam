# Improvement Plan — turtlebot3-slam

Plan for improving the algorithms and the evaluation behind the master thesis
*"Development of SLAM algorithm using probabilistic localization methods"*
(M. Kaniewski, PWr 2024). Produced from a five-part review (histogram filter,
particle filter, odometry/motion model, SLAM formulation, evaluation methodology)
carried out against commit `2fc8e91` and the thesis PDF.

**Scope:** algorithmic correctness and research validity — not code style.

---

## How to resume this work

1. `git switch develop` and read this file top to bottom.
2. Check the task checkboxes in each phase; continue at the first unchecked task.
3. All `file:line` references are **at commit `2fc8e91`**. Lines shift once edits
   start — re-locate with `grep` before editing, never trust the number blindly.
4. Phases are ordered so that every algorithmic change can be *measured*:
   Phase 0 builds the evaluation harness and records the baseline first.
5. When a task is done: tick the box, add a one-line note under **Log** at the
   bottom (date, commit, measured effect).
6. ROS 2 Humble + Gazebo 11 may not be runnable in every session. Most items in
   Phases 1–3 can be validated **offline** with plain `python3` on the stored
   `.pkl` scan maps (`ros2_ws/src/utils/maps_data/`, classes in
   `ros2_ws/src/utils/utils/scan_data.py`) and on recorded rosbags.

### Key facts established by the review (don't re-derive)

- The histogram filter (HF) is per-scan nearest-neighbour place recognition: no
  prediction, no recursive belief. The thesis (§3.1, p.11) describes a Bayes filter.
- The particle filter (PF) weights particles with pseudo-ranges to **all**
  reference points (`particle_filter.py:369`). The HF's chosen point is used
  only for its stored θ to rotate the scan (`:642`). The thesis says the PF
  compares against the nearest reference point.
- The PF publishes x,y from particles but **θ from the HF** (`:575`). Particle
  θ never enters the likelihood.
- §4.1 maps were built by teleporting the robot to exact grid poses
  (`utils/map_scanner.py:180-185`) → §4.1 is *localization in a ground-truth map*.
- §4.2 mapping: the operator added points only when the estimate "was close to
  the actual one" (thesis p.33) → ground-truth gated.
- `/odom` in Gazebo = world (ground-truth) pose: `odometry_source` unset in
  `turtlebot3_burger/model.sdf:358-395`, plugin default is WORLD. *(Believed from
  upstream source; confirm once the plugin is available.)*
- LiDAR runs at 5 Hz (`model.sdf:136`), noise σ = 0.01 m (`:151-155`).
- `INITIAL_POINTS = 0` in both filters → they load pre-built pkl maps; SLAM mode
  requires editing that constant.
- HF config loads `turtlebot3_dqn_stage4_0.1.pkl`, PF config loads
  `turtlebot3_dqn_stage4u_grid_0.25_3_3.pkl` — different maps and worlds.

---

## Phase 0 — Reproducibility & evaluation harness (do first)

Goal: be able to run any variant on identical input and get numbers, so every
later change is measured against a baseline.

- [ ] **0.1 Fix syntax error** — `histogram_filter.py:89` `0.0e` → `0.0`.
  HEAD does not run. *Verify:* `python3 -m py_compile` on all package files.
- [ ] **0.2 Mode & map as launch parameters** — replace the `INITIAL_POINTS`
  constant (`histogram_filter.py:18`, `particle_filter.py:25`) with a `mode`
  parameter (`localization` | `slam`) and a single shared `map_pkl_file`.
  Remove hard-coded paths: `particle_filter.py:826` (`/home/mkaniews/Desktop/map.pkl`)
  and `utils/map_plot.py:12` (`map11.pkl`, not in repo).
- [ ] **0.3 Commit missing artefacts** — the edited `stage4u` world, the world
  with the `gazebo_ros_state` plugin used by `map_scanner`, and the §4.2 map pkls
  (`map11.pkl`, `map15.pkl`) if still available.
- [ ] **0.4 Dedicated ground-truth topic** — add `libgazebo_ros_p3d` (world frame)
  publishing e.g. `/ground_truth`. Treat `/odom` as an *input only*. Update
  `odom_errors.py`, `particle_filter.py:601-613` (`odom_true_callback`) and
  `:659-665` (true distances) to use it. Make the initial pose an explicit
  parameter instead of copying from `/odom` (`odom_pos.py:137-156`,
  `odom_vel.py:141-159`).
- [ ] **0.5 `use_sim_time: true` everywhere** — no launch/config sets it now.
  Stamp all outputs with the source message stamp (`/joint_states`, `/scan`),
  not the wall-clock timer (`odom_pos.py:97`, `odom_vel.py:104`).
- [ ] **0.6 Replace `odom_errors.py` with timestamp-based evaluation** —
  current code pairs samples by index and ignores stamps (`odom_errors.py:95-110`),
  uses a wall-clock linspace (`:113-118`) and stores x,y only (`:49-84`).
  Plan: record rosbags → export every estimator to TUM format → use
  [`evo`](https://github.com/MichaelGrupp/evo):
  - `evo_ape` (RMSE/mean/max; unaligned **and** SE(2)-aligned; position + yaw
    with `atan2(sin Δ, cos Δ)` wrapping)
  - `evo_rpe` over 1 s and 1 m windows
  - drift % of distance travelled
- [ ] **0.7 Scripted, repeatable trajectories** — waypoint follower (or recorded
  `cmd_vel` replay) for 3 routes per world: short loop ~5 m, room traverse ~20 m,
  revisit/loop ~40 m. Record each once as a rosbag (`/scan`, `/joint_states`,
  `/imu`, `/clock`, `/ground_truth`) and replay the *same* bag into every method.
- [ ] **0.8 Seeds & parameter logging** — seed all RNG
  (`particle_filter.py:309-311, 349-351, 423`), dump the full parameter set with
  each run's results.
- [ ] **0.9 Runtime logging** — per-callback latency and actual rates of HF and
  PF; CPU/RAM.
- [ ] **0.10 Record the BASELINE** — run the unchanged algorithms (after 0.1–0.9
  only) on all bags, ≥10 seeds. Save the results table in `docs/results/baseline.md`.
  Everything below is compared against this.

---

## Phase 1 — Confirmed bugs (small, high impact)

Each item: fix, then re-run the Phase-0 harness and note the change in the Log.

### Particle filter (`ros2_ws/src/particle_filter/particle_filter/particle_filter.py`)

- [ ] **1.1 One measurement update per scan** — `particle_filter_callback` runs
  at 30 Hz (`:164`, `:188`) while the LiDAR is 5 Hz, and weights are multiplied
  (`:368`) → each scan's likelihood is applied ~6×; posterior becomes overconfident
  without being more accurate. Set a `new_scan` flag in `scan_callback` and
  update only when it is set (or move the update into `scan_callback`).
- [ ] **1.2 No update before the first scan** — `euclidean_distances =
  np.zeros(...)` (`:144`) passes the length check (`:366`) → updates run with
  z = 0 and collapse the uniform init to the grid centre. Init to `None` and skip.
- [ ] **1.3 Signed translation in prediction** — `distance_travelled =
  np.linalg.norm([Δx, Δy])` (`:346`) is ≥ 0, so reversing moves particles
  forward. Minimal fix: `δ = Δx·cos θ̄ + Δy·sin θ̄` (θ̄ = previous odom yaw).
  Superseded by 2.1, but do the minimal fix first so the baseline delta is clean.
- [ ] **1.4 Per-reference scan rotation** — one shift from the nearest
  reference's θ (`:642`) is applied to all references (`:650-652`); `int()`
  truncates. Rotate per reference: `np.roll(scan, round(θ_HF − θ_ref_k))`.
- [ ] **1.5 Nearest, not first, reference lookup; no dummy fallback** —
  `get_closest_reference_point` returns the first point inside a ±0.25 m box
  (`:511-513`); on failure a dummy point with θ = 0 and zero scan is used
  (`:624-626`). Better: HF publishes the reference **index/ID** (see 1.12).
- [ ] **1.6 Log-domain weights** — replace the linear product + `1e-300`
  (`:383`) with log-weights and log-sum-exp normalisation (an all-underflow case
  currently silently becomes uniform weights).
- [ ] **1.7 Remove or fix `update_method: 'simple'`** — `1/(e + 1e-6)` (`:378`)
  is not a likelihood; weight ratios up to ~1e54. Remove, or use `exp(−e/b)`.
- [ ] **1.8 Force `cumsum[-1] = 1` in systematic resampling** (`:425`), as the
  multinomial variant already does (`:405`).

### Histogram filter (`ros2_ws/src/histogram_filter/histogram_filter/histogram_filter.py`)

- [ ] **1.9 Consistent inf/max-range handling** — stored scans are clipped to
  3.5 m before binning (`:153`, so inf → last bin) but the live histogram is
  built from **unclipped** `msg.ranges` (`:278`, inf dropped). ~8% of beams are
  inf → ~30-count L1 bias. Offline effect: wrong matches (> 2 grid steps)
  0.25 m grid 31.4% → 23.6%, 0.1 m grid 8.6% → 5.0%. Use the same treatment on
  both sides (preferably a dedicated max-range bin). Also stop overwriting the
  stored raw scans in place at `:153`.
- [ ] **1.10 Use the stored reference heading** — `calculate_orientation`
  hard-codes `np.roll(ref, -90)` (`:234`, TODO at `:235`). Points added online
  store arbitrary θ (`:124`, `particle_filter.py:218`) → heading error =
  θ_ref − 90°, written into the next keyframe. Roll by the stored θ_ref (or
  store all scans pre-rotated to the world frame).
- [ ] **1.11 Guard the probability normalisation** — `:195-199` divides by zero
  when N = 1 or all distances are equal; `np.min` fails on an empty map.
  (Replaced properly in 2.4.)
- [ ] **1.12 Publish the reference ID** together with the pose so the PF uses
  the exact scan the HF chose (fixes the lookup mismatch in 1.5).

### Odometry (`ros2_ws/src/odometry/odometry/`)

- [ ] **1.13 Midpoint integration** — `odom_pos.py:83-85`, `odom_vel.py:83-89`
  use forward Euler; use θ + Δθ/2 in cos/sin (matches the Gazebo plugin's encoder
  mode). Also fix eq. 3.11 in the thesis.
- [ ] **1.14 Default PF/HF input to `/odom_pos`** — `particle_filter_params.yaml`
  `odom_topic: '/odom_vel'` and `histogram_filter.py:39`. Velocity-sampled
  odometry drifts (offline: 2.6° after 60 s, 6.5° after 300 s; with 3 ms dt
  jitter 14° / 0.49 m). Joint positions are exact integrals.
- [ ] **1.15 Joint lookup by name** — replace `left_wheel_indx`/`right_wheel_indx`
  with `msg.name.index('wheel_left_joint')`; load wheel params per
  `TURTLEBOT3_MODEL` (waffle separation 0.287 m vs burger 0.160 m).

---

## Phase 2 — Algorithmic improvements (days each)

### 2.1 Odometry motion model for the PF
- [ ] Replace fixed per-message noise (`particle_filter.py:349-351`; stationary
  cloud spreads ~0.64 m / 60 s because odometry publishes at 30 Hz) with Thrun's
  `sample_motion_model_odometry`:
  ```
  δrot1  = atan2(Δy, Δx) − θ̄     (if |δrot1| > π/2: δrot1 −= π·sign, δtrans = −δtrans)
  δtrans = sqrt(Δx² + Δy²)
  δrot2  = Δθ − δrot1
  σ²rot1  = α1·δrot1² + α2·δtrans²
  σ²trans = α3·δtrans² + α4·(δrot1² + δrot2²)
  σ²rot2  = α1·δrot2² + α2·δtrans²
  ```
  Skip prediction below a small motion threshold; optional time-based floor
  `σ² = q·Δt`.
- [ ] **Calibrate α1..α4** from logged data: scripted manoeuvres (±1 m straight,
  ±90°/360° spins, arcs, UMBmark CW/CCW squares) → cut into 0.5 s windows →
  non-negative least squares of squared residual (odom increment − GT increment)
  on [δrot², δtrans²]. Check consistency with NEES against χ²(3) bounds.
- [ ] Wrap Δθ from joint-state odometry; handle first message.

### 2.2 PF measurement model — step 1: signed 2D offset (1–2 days)
Problem: z_i = ‖(d_x, d_y)‖ is range-only → ring likelihood; with references
0.5 m apart the cross-range std is ~0.23 m at (1, 0.5) and ~0.40 m at (2, 0)
(Fisher-information estimate, σ = 0.125). The min over each cone (`:543`) is
biased low (true 0.20 m → mean 0.168 m with 2 cm beam noise). Clipped 3.5 m
readings (`:129`, `:637`) give zero differences in open directions.
- [ ] Per reference k, in the reference frame (beam 0 forward, beam 90 left),
  take the **median of signed** differences per cone:
  `δx' = ½[(γ180−η180) − (γ0−η0)]`, `δy' = ½[(γ270−η270) − (γ90−η90)]`;
  mask max-range beams.
- [ ] Observation in world frame: `z_k = p_ref,k + R(θ_ref,k)·δ'`.
- [ ] Likelihood `N(z_k; (x, y), Σ)` over the **k nearest** references only.
- [ ] Calibrate Σ from Tables 4.1/4.2-style static tests (RMS 0.087–0.106 m,
  mean bias −0.06 m in the thesis data). Temper for correlated beams:
  `w ∝ p(z|x)^(1/κ)`.

### 2.3 PF measurement model — step 2: likelihood field over reference scans (3–5 days)
- [ ] Build a point cloud of all reference-scan endpoints:
  `m = { p_ref,k + r_kj·[cos(θ_ref,k + φ_j), sin(θ_ref,k + φ_j)] }`; store in a
  KD-tree or distance-transform grid; update incrementally as keyframes are added.
- [ ] Score each particle (x, y, θ) with 30–60 subsampled beams:
  `log w = Σ_j log( z_hit·N(dist(e_j(x,y,θ), m); 0, σ_hit) + z_rand/z_max )`.
- [ ] This estimates θ inside the PF → stop publishing HF θ (`:575`); heading
  estimate = circular mean `atan2(Σ w sin θ, Σ w cos θ)`.
- [ ] Removes the dependence on per-reference scan rotation (1.4) and most of the
  HF→PF coupling. Standard MCL on a growing map.
- Alternative: ICP / correlative scan matching against the nearest keyframe as
  a relative-pose measurement with covariance (shared with 3.2).

### 2.4 Histogram filter → real discrete Bayes filter
- [ ] **Likelihood:** replace `:195-199` (current "probabilities": p_max/p_min
  ≈ 1.001, i.e. ≈ uniform) with `p(z|k) ∝ exp(−D_k/τ)` (τ from the spread of
  distances at true matches), or a multinomial likelihood for count histograms:
  `log p(h'|k) = Σ_b h'_b · log[(h_k,b + α)/(Σh_k + Bα)]`.
- [ ] **Prediction** over reference points using odometry:
  `bel⁻(k) = Σ_j N(x_k; x_j + R(θ̂)·Δu, Σ_u)·bel(j)`, then
  `bel⁻ ← (1−ε)·bel⁻ + ε/N` (recovery), `bel(k) = η·p(z|k)·bel⁻(k)`.
  On a regular grid: shift + Gaussian blur, O(N).
  Offline (0.25 m grid, 150 random walks × 60 steps): wrong matches L1 25.0% →
  10.7%; Wasserstein 11.4% → 6.1%.
- [ ] Publish the **top-k belief** (IDs + probabilities), not just the argmax.
- [ ] Re-do the Manhattan-vs-others choice (p.29) on accuracy, not heat-map
  contrast.

### 2.5 HF similarity measure
Offline wrong-match rates (after 1.9), 0.1 m / 0.25 m grids:

| Method | 0.1 m | 0.25 m |
|---|---|---|
| L1, 20 bins (current) | 5.0% | 23.6% |
| χ² | 3.7% | 22.3% |
| EMD (L1 of CDFs) | 4.3% | 13.2% |
| 1D Wasserstein on sorted ranges (no bins) | **2.5%** | 11.7% |
| Rotation-invariant circular cross-correlation (FFT) | 4.3% | **6.7%** |

- [ ] Implement Wasserstein-on-sorted-ranges (cheap drop-in).
- [ ] Implement FFT cross-correlation:
  `SSD(s) = ‖q‖² + ‖r‖² − 2·Re FFT⁻¹(F_q·F_r*)` — gives match score **and**
  heading in O(M log M), replacing the 360× `np.roll` loop (`:238-241`).
- [ ] Optional: Scan Context / ring-key descriptor for retrieval at scale.
- [ ] Vectorise: keep an (N, B) matrix; no per-point Python loop (`:182-186`);
  no O(N²) `np.append` rebuild on every trigger.

### 2.6 Joint (Δx, Δy, Δθ) registration against the matched keyframe
Heading error is dominated by the robot's offset from the reference, not by the
1° step (offline, true nearest ref: 0.25 m grid median 4.9°, p90 26.8°; parabolic
sub-degree interpolation doesn't help). HF position is quantised to the grid
(`:203`); top-k weighted averaging doesn't help (0.224 → 0.244 m).
- [ ] Correlative scan matching (coarse, from the FFT peak) → point-to-line ICP
  refinement → `x̂ = x_k + R(θ_k)·Δ` with covariance.
- [ ] Mask/down-weight max-range beams.
- [ ] Output used by: HF pose, PF measurement (2.3 alt.), pose-graph edges (3.2).

### 2.7 PF robustness
- [ ] Augmented MCL: `w_slow += α_slow(w_avg − w_slow)`, `w_fast += α_fast(w_avg − w_fast)`;
  inject `max(0, 1 − w_fast/w_slow)` fraction of particles into free space
  (or around HF top-k keyframes — Mixture MCL).
- [ ] Reject particles in occupied PGM cells at init and after prediction
  (PGM already loaded, `:110`).
- [ ] SLAM mode: initialise a Gaussian around the start pose, not uniform.
- [ ] Pose output: circular mean for θ; largest-cluster mean when multimodal;
  publish `PoseWithCovarianceStamped` (the variance is computed then discarded
  at `:201`).
- [ ] Vectorise particles as an (N, 3) array:
  `D = hypot(P[:,None,:2] − L[None])`, `logw += −0.5·Σ((D − z)/σ)²` → 10⁴
  particles feasible.
- [ ] Time alignment: apply the measurement at the scan stamp (buffer odometry,
  interpolate), not after later predictions.

### 2.8 Better prediction inputs (optional)
- [ ] Fuse wheel odometry + IMU gyro yaw rate via `robot_localization` EKF
  (`two_d_mode`). Gyro is simulated but currently unused.
- [ ] Scan-matching odometry (`rf2o_laser_odometry` or PL-ICP/csm) as a second
  input.

---

## Phase 3 — Make it SLAM (weeks)

- [ ] **3.1 Automatic keyframe insertion, no ground truth in the loop** — replace
  `add_map_point.sh` + manual gating (p.33) with insertion every ~0.3 m / 20° of
  odometry, gated on PF covariance and/or low HF best-match similarity.
  Log the GT pose beside each stored pose (evaluation only).
- [ ] **3.2 Single map-server node** — owns keyframes (ID, pose, covariance, scan,
  descriptor), publishes updates; HF and PF subscribe. Replaces the two
  independent copies written by `/trigger_pf` and `/trigger_hf`
  (`particle_filter.py:205-220`, `histogram_filter.py:~120-125`).
- [ ] **3.3 Back-end — choose one (see Open questions):**
  - **Option A — Pose graph (recommended).** Nodes = keyframes. Edges =
    odometry between consecutive keyframes + scan-matching (2.6) to k nearest
    keyframes, each with covariance. Loop closures: HF top-k candidates (2.4)
    verified by scan-matching fitness. Optimiser: GTSAM Python / g2o, or a
    hand-written 2D Gauss-Newton (~150 lines for < 1000 nodes). The existing
    `ScanData(pose, scan)` is effectively Karto's `LocalizedRangeScan`.
  - **Option B — Rao-Blackwellised PF (keeps the "particle filter SLAM" title
    literal).** Each particle holds its own keyframe poses (scans shared →
    3 floats × keyframes × particles). Weight = match of current scan against
    that particle's keyframes; scan-matched proposal; N_eff resampling (already
    implemented). GMapping-style.
- [ ] **3.4 Online occupancy grid** rebuilt from optimised poses after each
  optimisation. Fix in `utils/map_plot.py`: separate `l_occ`/`l_free` (currently
  symmetric ±0.55, `:117-119`), free-only update for max-range beams, grid size
  from data instead of fixed 5×5 m (`:25-28`), beam angles from `LaserScan`
  `angle_min/angle_increment` instead of `linspace(-180, 180, 360)` (which spaces
  beams 360/359°).
- [ ] **3.5 Map scaling** — keyframe spacing by overlap, pruning, float16 scans
  (currently 360 float64 per point; 0.05 m grid of the 5×5 m room = 7331 points,
  21 MB).

---

## Phase 4 — Evaluation protocol (run after each phase)

- **Worlds:** `turtlebot3_dqn_stage1` (symmetric), `stage4`, `stage4u`,
  `turtlebot3_world`, `turtlebot3_house` (multi-room, forces loops/exploration —
  the 5×5 m room is mostly visible from its centre with a 3.5 m LiDAR).
- **Routes:** 3 scripted per world (Phase 0.7).
- **Methods:**
  1. wheel odometry (`/odom_pos`)
  2. this PF, localization in a GT keyframe map
  3. this full SLAM, automatic insertion, no GT
  4. nav2 AMCL on the GT PGM
  5. slam_toolbox (online async)
  6. Cartographer 2D (already used for the PGM, thesis p.12)
  7. trivial baselines: constant grid-centre estimate, nearest-reference-point,
     single-scan map from start pose
  8. **oracle ablations:** PF with GT distances (`particle_filter.py:662-665`
     already computes them; commented update at `:184`), PF with GT nearest
     reference, PF with GT heading → attributes error to measurement model vs HF.
- **Runs:** ≥ 10 seeds per method × route × world; mean ± std, median; Wilcoxon
  for key comparisons.
- **Trajectory metrics:** APE (unaligned + SE(2)-aligned, position + yaw), RPE
  (1 m, 1 s), drift %, fraction of time with error < 0.25 m, convergence time
  from uniform init, kidnapped-robot recovery (teleport mid-run).
- **HF as place recognition:** top-1 accuracy, distance to matched point,
  precision–recall; sweep bins {10, 20, 40} × metric {L1, L2, χ², Wasserstein,
  FFT} × grid {0.1, 0.25, 0.5} m. Offline baseline: leave-one-out on
  `stage4u_grid_0.25.pkl` → median distance to match 1.41 m, only 20% adjacent.
- **Map metrics** vs corrected PGM: occupied-cell precision/recall/IoU at
  5/10/25 cm, covered area, Chamfer distance, per-keyframe pose error.
- **Sweeps:** particles {50, 250, 1000}; cone {1, 10, 15}°; LiDAR σ
  {0.01, 0.03, 0.05} m.
- **Realism layer** (node between Gazebo and filters): wheel r_L, r_R, b scaled
  ±1–3% per run; per-step noise ∝ |Δφ|; 4096-tick quantisation; LiDAR
  `σ(r) = max(0.01, 0.035·r)`, 1–2% dropouts, rotation skew (0.2 s sweep);
  wheel μ ≈ 1 (currently 1e5, `model.sdf:195-199`).
- **Runtime:** per-callback latency, rate, CPU/RAM vs map size and particle
  count; PC and ideally Raspberry Pi 4.
- **Optional:** one real TurtleBot3 run with slam_toolbox as reference.
- **Success criteria (set before running):** e.g. ATE RMSE < 0.15 m over a
  ≥ 20 m route; occupied-cell IoU ≥ 0.6; ≥ 5 Hz on RPi4. Adjust, but fix them up front.

---

## Thesis errata (text only; code is correct unless noted)

- [ ] Eq. 3.8: remove `1/N` (weights are normalised).
- [ ] Eq. 3.6 / Algorithm 1: `w = p(z|x)` → `w ← w·p(z|x)` (needed with adaptive
  resampling; code does this).
- [ ] p.23: systematic resampling offset is `U[0, 1/N)`, not "0 to N/2".
- [ ] p.29: relative error range "3.49 % to 31.98 %" but Table 4.2 max is 39.68 %.
- [ ] p.10 vs pp.20/26: `/odom` described as encoder+IMU odometry but used as
  true position (in Gazebo it is world pose).
- [ ] p.21: reference positions "derived from odometry" — §4.1 maps come from
  teleported GT poses.
- [ ] Fig. 4.1 / Summary: PF compares against all references, not only the nearest.
- [ ] §3.1 p.11: HF "updates its belief" — no recursion in code (fixed by 2.4).
- [ ] p.14: "normalise to [0, 1]" — code divides by the sum, not the range.
- [ ] Eq. 3.11: forward Euler (→ midpoint, 1.13).
- [ ] Ch. 5 claims needing evidence or rewording: "tested in an unknown
  environment", "not affected by simulation duration" (Fig. 4.19 shows growth),
  "resistant to rapid movements" (no speed experiment; odometry offset was
  injected by a pre-drive, p.26), "adaptability across scenarios" (one world + edit).
- [ ] §1.1 thesis statement: add measurable success criteria, or frame as a
  feasibility study.

---

## Open questions (ask the author before Phase 3)

1. Purpose: follow-up paper, portfolio, or thesis revision? Decides how much of
   Phase 4 is needed.
2. Back-end: Option A (pose graph) or Option B (RBPF)?
3. Keep the HF+PF architecture as the contribution, or allow replacing the PF
   measurement model entirely (2.3)?
4. Is a real TurtleBot3 available?
5. Should `ros2_ws/src/turtlebot3_simulations` changes (p3d plugin, noise, μ) go
   into the submodule fork, or into a separate world/model package in this repo
   (preferred — avoids touching submodules)?

## Suggested paper directions (if 1 = paper)

- **A.** Histogram/scan descriptors for loop closure in lightweight 2D keyframe
  SLAM: range histogram vs Scan Context vs LiDAR-Iris vs M2DP, PR curves, sim + real.
- **B.** Memory-light RBPF with shared scans and per-particle keyframe poses,
  benchmarked vs GMapping / slam_toolbox on Intel Research Lab / MIT CSAIL with
  the Kümmerle et al. (2009) relative-error metric.

---

## Log

<!-- One line per completed task: YYYY-MM-DD · task id · commit · measured effect -->
- 2026-09-13 · plan created from review of `2fc8e91`
