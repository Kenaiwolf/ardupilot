```markdown
# Fork Documentation: Kenaiwolf/ardupilot1 — Rover Vectored-Thrust Drift Compensation

**Fork base:** upstream ArduPilot `b2b1b3d2` (16 Sep 2025). All commits above it are fork work, culminating at `375952a5` (hwdef).

**Target vehicle:** 250 kg vectored-thrust boat (single steerable thruster, GPS + compass, no wheel encoders).

**Modified files:** `Rover/GCS_MAVLink_Rover.cpp`, `Rover/mode.cpp`, `Rover/mode.h`, `Rover/mode_guided.cpp`, `Rover/mode_loiter.cpp`, `libraries/APM_Control/AR_AttitudeControl.cpp/.h`, `libraries/AR_Motors/AP_MotorsUGV.cpp/.h`, new file `extra_hwdef.dat`.

---

## 1. Feature Overview

Three layered features on top of upstream Rover:

1. **Current/wind drift estimation & compensation** — two independent estimators (nav-source while driving, loiter-source while holding position) feed crab-angle/speed corrections into Auto, Guided and Loiter, with per-source gains, age-weighted seeding across mode switches, and a kill-switch at gain=0.
2. **Steering thrust floor + throttle management** — guarantees minimum thrust whenever a large heading correction is commanded, so a vectored thruster always has authority; plus cosine throttle reduction and speed-PID I-freeze during pivot turns.
3. **Vectored-thrust angle blending** — blends direct and atan-based steering-angle computation based on filtered throttle, with hysteresis deadband.

Supporting infrastructure: MAVLink named-float diagnostics, an `ATC_SPD_EXPO` nonlinear speed↔throttle curve, and a slim `extra_hwdef.dat` build.

---

## 2. `libraries/AR_Motors/AP_MotorsUGV.cpp/.h` (+352/−36, +92/−4)

### 2.1 Vectored-thrust angle blending (`MOT_VEC_BLEND_THR`, `MOT_VEC_DEADBAND`, `MOT_VEC_RESID_TC`)

In the vectored-thrust output path, the commanded (steering, throttle) pair is converted to a thruster deflection angle `steering_angle_rad`. Two mappings exist:

- **Direct mapping** — angle proportional to steering demand; best at low/zero throttle.
- **Legacy `atan()` mapping** — `atan2(steer, throttle)`; best at cruise.

The fork blends them with weight `w`:

```cpp
w = 1.0f - constrain_float(fabsf(_vec_throttle_filt) / _vec_blend_thr, 0.0f, 1.0f);
steering_angle_rad = w * direct_angle_rad + (1.0f - w) * atan_angle_rad;
// AP_MotorsUGV.cpp ~1030-1038
```

`w=1` (near-zero throttle) → pure direct mapping; `w=0` (throttle ≥ blend threshold) → pure atan mapping. The same `w` blends the boost scaler:

```cpp
const float throttle_scaler_inv = w + (1.0f - w) * cosf(steering_angle_rad);
if (!is_zero(throttle_scaler_inv)) {
    throttle /= throttle_scaler_inv;
}
// AP_MotorsUGV.cpp ~1057-1060
```

**Physics:** `throttle` is the commanded *forward* thrust component. The boost guarantees the commanded forward component `T·cos(θ) = throttle` is deliverable — `T = throttle/cos(θ)` — which simultaneously makes the lateral component `T·sin(θ) = throttle·tan(θ)` large enough to turn the boat at speed. Without it, the boat loses steering authority exactly when moving fast. The 1/cos spike near 90° is intentional (see §10).

`MOT_VEC_DEADBAND` freezes `_vec_last_steering_angle_rad` when both steering and throttle demands are near zero — prevents servo hunting. `_vec_resid_tc` sets the EMA time constant on `_vec_throttle_filt` so the blend weight doesn't chatter with throttle noise.

### 2.2 Drift estimator state store

Four slots of persistent state per estimate (nav + loiter):

```cpp
void AP_MotorsUGV::set_nav_estimate_ne(const Vector2f &drift_ne)
{
    const float gain = constrain_float(_drift_comp_gain_nav, 0.0f, 2.0f);
    _nav_estimate_ne = drift_ne * gain;      // gain applied at WRITE time
    _nav_estimate_ms = AP_HAL::millis();
    _nav_estimate_is_seeded = false;
}
// AP_MotorsUGV.cpp ~301-307
```

```cpp
void AP_MotorsUGV::seed_loiter_estimate_ne(const Vector2f &drift_ne, uint32_t source_ms)
{
    _loiter_estimate_ne = drift_ne;
    _loiter_estimate_ms = source_ms;         // ORIGINAL timestamp preserved
    _loiter_estimate_is_seeded = true;
}
// AP_MotorsUGV.cpp ~310-315
```

Write-time gain means a stored value is PID-ready; changing the gain parameter doesn't retroactively rescale stored estimates (they refresh on the next sample). The `is_seeded` flag distinguishes measured samples from handoff placeholders.

### 2.3 Source arbitration — `get_current_estimate_ne`

```cpp
const bool loiter_real = (loiter_age_ms != UINT32_MAX) && !_loiter_estimate_is_seeded;
const bool nav_real    = (nav_age_ms    != UINT32_MAX) && !_nav_estimate_is_seeded;
// a genuinely-measured estimate always wins over a seeded one, regardless of timestamp;
// between two seeds or two reals, newer wins
// AP_MotorsUGV.cpp ~357-370
```

Then a shared staleness cutoff `DRIFT_MAXAGE` (default 1800 s, 0 = never trust) rejects both. Priority rule: **real > seeded**, then **newest wins**.

### 2.4 Parameter group

All new params live in `AP_MotorsUGV` under the `MOT_`/`LOIT_`/`DRIFT_`/`SFL_` prefixes — see §9 table.

---

## 3. `libraries/APM_Control/AR_AttitudeControl.cpp/.h` (+26/−13, +11/−7)

### 3.1 Nonlinear speed→throttle feed-forward (`ATC_SPD_EXPO`)

```cpp
if (is_positive(cruise_speed) && is_positive(cruise_throttle)) {
    const float speed_ratio = fabsf(_desired_speed) / cruise_speed;
    const float expo = (_speed_thr_expo > 0.0f) ? _speed_thr_expo : 1.0f;
    throttle_base = cruise_throttle * powf(speed_ratio, expo);
    if (is_negative(_desired_speed)) {
        throttle_base = -throttle_base;
    }
}
// AR_AttitudeControl.cpp ~800-807
```

Boats have quadratic drag — a linear FF underestimates thrust at low speed ratios and overestimates above cruise. `expo` shapes the calibrated curve through the (cruise_speed, cruise_throttle) point. Value 1.0 = stock behavior; ~2.0 recommended for displacement hulls.

**Important for estimation:** the same `powf(ratio, expo)` curve is the basis for loiter method-2's inverse mapping (I-term → equivalent speed) and the drift feed-forward — one parameter governs all three, keeping them consistent.

### 3.2 Throttle-limit flag plumbing

`get_throttle_out_speed` / `get_throttle_out_stop` already took `motor_limit_low`/`motor_limit_high`; the fork OR's `steer_i_freeze` into both at the call site (`mode.cpp:352-356`). Inside, all four collapse to one bool:

```cpp
float throttle_out = _throttle_speed_pid.update_all(_desired_speed, speed, dt,
    (motor_limit_low || motor_limit_high || _throttle_limit_low || _throttle_limit_high));
throttle_out += _throttle_speed_pid.get_ff();
// AR_AttitudeControl.cpp ~810-811
```

See §4.4 for why the collapsed direction-agnostic limit is correct.

---

## 4. `Rover/mode.cpp` (+354/−37) and `Rover/mode.h` (+81/−9)

### 4.1 `update_drift_estimator(raw_heading_cd, desired_speed)` — nav-source estimator

Called from `navigate_to_waypoint` and Guided HeadingAndSpeed with the **uncompensated** commanded vector. Accumulates heading+speed samples only while:

```cpp
const bool yaw_rate_ok = fabsf(degrees(ahrs.get_yaw_rate_earth())) < drift_est_yaw_rate_dps;
// mode.cpp ~585-587 — earth-frame yaw rate (not gyro.z), so wave-induced
// roll/pitch rates don't contaminate the gate
```

`MOT_DRIFT_EST_YAWR` (default 15 °/s, 0 = off; promoted from compile-time constant `DRIFT_EST_MAX_YAW_RATE_DPS` in `mode.h`) is the yaw-rate gate. Samples collected during steady straight motion are converted into an NE drift vector and written via `set_nav_estimate_ne`. The estimator window is reset on mode entry (commit A1.2) so stale partial windows don't leak across modes.

### 4.2 `apply_drift_compensation(heading_cd, speed)` — crab-angle correction

Computes the required own-motion vector = desired track − estimated drift, then:

```cpp
const float speed_max = calc_speed_max(g.speed_cruise, 1.0f);
desired_speed = MIN(own_required_ne.length(), speed_max);
// mode.cpp ~561 — clamp magnitude only; direction stays optimal (up-current)
// even if required speed exceeds capability: pointing up-current at max speed
// is the best achievable response
```

A reversal branch handles `desired_speed < 0` (heading +180°, negative speed). In-place at the caller — which is why Guided passes a **local copy** of `_desired_yaw_cd` (see §6).

### 4.3 Seeding on mode entry — `Mode::enter()`

```cpp
// one-shot seed from Loiter, age-weighted; avoids cold (0,0) start.
// seeded sources accepted: the seed carries the ORIGINAL sample timestamp,
// so ping-pong between slots is safe - each bounce multiplies by seed_weight <= 1
// and the value keeps aging toward DRIFT_MAXAGE (converges to zero, never amplifies).
// guard: never overwrite a real (measured) destination estimate with a weighted
// seed copy - a real sample has higher priority in get_current_estimate_ne().
Vector2f nav_ne; uint32_t nav_age_ms = 0; bool nav_is_seeded = false;
const bool nav_has_real =
    g2.motors.get_nav_estimate_ne(nav_ne, nav_age_ms, nav_is_seeded) &&
    !nav_is_seeded &&
    (nav_age_ms < uint32_t(g2.motors.get_drift_max_age_s() * 1000.0f));
if (!nav_has_real &&
    g2.motors.get_loiter_estimate_ne(loiter_ne, loiter_age_ms, loiter_is_seeded) &&
    (loiter_age_ms >= DRIFT_SEED_MIN_AGE_MS) &&
    (loiter_age_ms < uint32_t(g2.motors.get_drift_max_age_s() * 1000.0f))) {
    // seed nav slot from loiter value, weighted by source age
}
// mode.cpp ~59-80
```

`DRIFT_SEED_MIN_AGE_MS = 50` (`mode.h:200`) blocks same-tick echo (Guided::_enter → start_loiter → ModeLoiter::_enter). Seeded sources are deliberately accepted so bridge `LOITER_WP_RESET` bounces (Guided→Loiter→Guided in ~1.5 s) don't cold-start the nav estimator — the loiter window almost never completes in that window.

### 4.4 `calc_throttle` — cosine reduction, drift FF, steering floor, I-freeze

Order of operations in the vectored-thrust + heading-fresh block (HEAD, after A1.2 reorder):

```cpp
// 1. Cosine Throttle Reduction: reduce forward throttle at large heading error.
// only scale forward thrust - a negative (braking) PID demand must pass
// through unchanged, otherwise a heavy boat cannot decelerate during a
// large heading change
if (is_positive(throttle_out)) {
    throttle_out *= MAX(0.0f, cosf(yaw_error_rad));
}
// mode.cpp ~390-399
```

Then drift feed-forward is **added after** the cosine scaling (A1.2 moved it — previously the FF could be zeroed out by cos() at large crab angles, defeating its purpose of holding position):

```cpp
// FF = full nonlinear curve evaluated at |drift_body.x|, NOT a local derivative —
// at target_speed=0 (loiter) dThrottle/dv ≈ 0 for expo>1 and FF would vanish
// exactly where it is needed. computed inline: get_throttle_out_speed() is a
// stateful PID loop and a second call per tick would corrupt I/D terms
if (is_positive(drift_body.x)) {
    const float drift_ratio = drift_body.x / cruise_speed;
    throttle_out += 100.0f * throttle_cruise_frac * powf(drift_ratio, expo);
}
// mode.cpp ~401-422
```

Then the steering thrust floor (applied last, so nothing downstream can remove it):

```cpp
// 2. Steering Thrust Floor: force minimum throttle to allow rotation
if (yaw_error_deg > g2.motors.get_steer_floor_deadband_deg()) {
    const float steer_throttle_floor = constrain_float(
        (yaw_error_deg - g2.motors.get_steer_floor_deadband_deg()) * g2.motors.get_steer_floor_gain(),
        0.0f, g2.motors.get_steer_floor_max_pct());
    // do not override a throttle that is already stronger than the floor in
    // the same direction; only raise the magnitude, never flip its sign
    if (fabsf(throttle_out) < steer_throttle_floor) {
        throttle_out = is_negative(throttle_out) ? -steer_throttle_floor : steer_throttle_floor;
    }
}
```

`SFL_DB=10°`, `SFL_GAIN=0.7`, `SFL_MAX=20%`, `SFL_IFRZ=45°`. Yaw error is measured against `_steering_target_yaw_cd` (the target *before* crab correction — A1.1 fixed compounding where the corrected heading fed back into the error).

**Freshness gate:** `steering_heading_fresh` = `_steering_heading_active_ms` written by `calc_steering_to_heading()` within 50 ms (`mode.h:245-250`). This scopes floor + cosine reduction to heading-driven modes — Auto, Guided, Loiter, **and Simple and Follow** (which also call `calc_steering_to_heading` — `mode_simple.cpp:30`, `mode_follow.cpp:88`; comment updated in A1.3).

**I-freeze (`SFL_IFRZ`) — reviewed, intentionally kept:**

`steer_i_freeze` is OR'd into both limit flags and collapses to one bool passed to `AC_PID::update_i`. With `limit=true`, `update_i` permits integrator change only when error opposes integrator sign — **hold-or-decay, never grow** (`AC_PID.cpp:342-347`).

This was originally flagged as a "direction-undifferentiated freeze" defect with a proposed hard-hold fix (`i_scale=0`). On deeper analysis the current behavior is correct:

1. **The speed-PID integrator is a body-frame quantity** — "forward thrust needed to hold commanded speed", valid only relative to the heading at which it was learned. Freeze only activates above `SFL_IFRZ=45°`, i.e. always during large reorientations. After a 90° turn, the current that required forward thrust becomes lateral (invisible to the speed PID); after 180°, the required thrust flips sign. A held integrator would be systematically *stale* — worse than zero.
2. **Decay acts as implicit invalidation.** During a pivot, speed error typically opposes the integrator (current/inertia carries the boat past target), so the sign rule lets the integrator bleed toward zero — a reasonable reset of a body-frame estimate. True negative windup (hard reverse after unfreeze) is already blocked: same-sign growth is never allowed, and the decay branch dead-ends at exactly zero.

The world-frame memory of current lives in the drift estimator (NE coordinates), heading-invariant and unaffected by this freeze — nothing that matters is forgotten. Kept consequence: brief speed undershoot after a large turn while I re-learns drag — small at typical `ATC_SPD_I` and pivot durations.

### 4.5 `mode.h` additions

New members: `_steer_floor_active`, `_steering_heading_active_ms`, `_drift_rising_count`, `_drift_zero_count`, `_drift_i_filt`/`_drift_i_filt_valid`, `_inside_loiter_circle`, `_throttle_nav_pct`; constants `DRIFT_SEED_MIN_AGE_MS`, `LOITER_DRIFT_RISING_TICKS`, `LOITER_DRIFT_ZERO_TICKS=5`; helpers `update_drift_estimator`, `apply_drift_compensation`, `get_drift_compensation_body`, `calc_steering_to_heading`, `start_loiter` (Guided), `get_speed_thr_expo` accessor.

---

## 5. `Rover/mode_loiter.cpp` (+303/−52)

Loiter is the second drift source and the main beneficiary of compensation.

### 5.1 Anti-drift heading (bow-into-current)

```cpp
if (g2.motors.get_current_estimate_ne(drift_ne) &&
    drift_ne.length() > g2.motors.get_loit_drift_min_mps()) {
// mode_loiter.cpp ~79-80 — LOIT_DRIFT_MIN default 0.03 m/s
```

When drift exceeds the minimum, loiter steers the bow into the estimated current instead of holding the entry heading — reduces the lateral component the thruster must continuously fight.

### 5.2 Method 1 — coast sampling

While inside the loiter circle, the controller watches `_throttle_nav_pct` (snapshot taken before the floor/FF block — `mode.cpp:367`). When drift pushes the boat and nav throttle keeps rising for `LOITER_DRIFT_RISING_TICKS` consecutive ticks, a **coast** window opens: throttle drops below `LOIT_COAST_THR` (1 %) and the GPS velocity during the coast is sampled as a drift measurement → `set_loiter_estimate_ne`.

**Rotation gate (A1.1):** sampling is rejected while the boat is actively rotating:

```cpp
const float yaw_rate_degs = fabsf(degrees(ahrs.get_yaw_rate_earth()));
// gated by LOIT_ROT_ANG (15°) and LOIT_ROT_RATE (20°/s), 0 = gate off
// rationale: a deflected bow thruster rotates the hull about its pivot point
// (~0.6 m radius at 90°), generating lateral IMU velocity r·L_pivot up to
// ~0.5 m/s that is NOT drift
```

Originally implemented with `ahrs.get_gyro().z` (body-frame); changed to `get_yaw_rate_earth()` in A1.4 so wave-induced roll/pitch rates don't false-trigger the gate — consistent with the nav estimator's gate.

**`_steer_floor_active` gate:** coast sampling is suppressed while the floor is forcing throttle (the boat isn't coasting).

**Zero-drift sample (`_drift_zero_count`, `LOITER_DRIFT_ZERO_TICKS=5`):** if the equilibrium window holds but the filtered I-term stays below the `LOIT_I_MIN` noise floor for 5 ticks, a **zero** drift estimate is accepted — without it, a ceased current would leave the last nonzero estimate pinned until `DRIFT_MAXAGE` expiry.

### 5.3 Method 2 — equilibrium I-term inference

When the loiter is in steady state (position error small, throttle stable), the speed-PID I-term is a direct measure of the thrust holding position against drift:

```cpp
// assumes bow-pull thruster convention: thrust dir = yaw + steer angle.
// for a stern-push or inverted-output thruster add +M_PI to thrust_rad.
const float steer_ang_rad = (g2.motors.get_steering() / 4500.0f)
                            * radians(g2.motors.get_vector_angle_max());
const float thrust_rad = ahrs.get_yaw_rad() + steer_ang_rad;
// drift vector = opposite of the force that holds us in place
meas_dir_ne = Vector2f{-cosf(thrust_rad), -sinf(thrust_rad)};
// mode_loiter.cpp ~192-196
```

```cpp
// clamp: if (I + FF) exceeds cruise throttle the powf() above would
// extrapolate beyond the calibrated curve - cap at the full-throttle speed
// ceiling, not cruise_speed, so currents stronger than cruise are still
// representable
const float drift_mag_mps = MIN(cruise_speed * powf((_drift_i_filt + ff_frac) / cruise_thr, 1.0f / expo),
                                calc_speed_max(g.speed_cruise, 1.0f));
// mode_loiter.cpp ~188-193
```

Tunables: `LOIT_I_EQERR` (throttle-stability window 0.10), `LOIT_I_MIN` (noise floor 0.02), `LOIT_I_ALPHA` (EMA 0.10), `LOIT_I_DISAG` (method-1/method-2 disagreement handling 0.5). The inverse `powf(…, 1/expo)` maps I-term-fraction → equivalent speed using the same `ATC_SPD_EXPO` curve as the forward direction.

### 5.4 Seeding on loiter entry

Symmetric to `Mode::enter` (§4.3): seeds the loiter slot from the nav estimate, age-weighted, accepting seeded sources, guarded against overwriting a real loiter measurement (`loiter_has_real`).

---

## 6. `Rover/mode_guided.cpp` (+30/−15)

Two changes:

**1. Boat stops via loiter, not throttle-cut.** Every "stop" path for boats routes through `start_loiter()` (falling back to `start_stop()`/`stop_vehicle()` if loiter init fails):

```cpp
if (rover.is_boat()) {
    if (!start_loiter()) {
        start_stop();
    }
} else {
    start_stop();
}
// mode_guided.cpp ~7-13 (same pattern at destination reach, timeouts, etc.)
```

A 250 kg boat carries significant momentum and is drift-dominated — cutting throttle leaves it adrift; loiter actively holds position.

**2. HeadingAndSpeed feeds estimator + applies compensation:**

```cpp
if (have_attitude_target) {
    // feed the estimator with the *uncompensated* commanded vector - using
    // corrected_heading_cd here would subtract our own correction out of the
    // measured drift, systematically under-estimating it
    update_drift_estimator(_desired_yaw_cd, _desired_speed);

    // apply drift compensation to a local copy of the heading only -
    // _desired_yaw_cd must stay clean (persistent commanded target, re-read
    // every tick), otherwise the crab-angle correction would compound
    // cycle-over-cycle since apply_drift_compensation() writes its result
    // back into whatever variable is passed in
    float corrected_heading_cd = _desired_yaw_cd;
    float corrected_speed = _desired_speed;
    apply_drift_compensation(corrected_heading_cd, corrected_speed);

    calc_steering_to_heading(corrected_heading_cd);
    calc_throttle(calc_speed_nudge(corrected_speed, is_negative(corrected_speed)), true);
}
// mode_guided.cpp ~60-77
```

The uncompensated-in / local-copy-out discipline is the key correctness invariant of the whole feature.

---

## 7. `Rover/GCS_MAVLink_Rover.cpp` (+40/−10)

`send_named_float` diagnostics on `MAVLINK_COMM_0` only (avoids N× duplication per channel): `DRIFTN`/`DRIFTE`/`DRIFTSPD` current estimate, `DRIFTNAGE`/`DRIFTNSED`/`DRIFTLAGE`/`DRIFTLSED` sample age (s) and seeded flag per slot — essential for A/B verification on water.

---

## 8. `extra_hwdef.dat` (~850 lines, new)

Slim Rover build for a low-flash FC: ~470 feature `undef`s plus explicit `define … 0` — copter/plane modes, most rangefinder/mount/camera/OSD/CAN bindings, EKF2 (`HAL_NAVEKF2_AVAILABLE 0`), airspeed, and most GPS/compass drivers removed (kept: uBlox, NMEA, IST8310).

---

## 9. Parameter summary (all new, in `AP_MotorsUGV`)

| Param | Default | Purpose |
|---|---|---|
| `MOT_VEC_BLEND_THR` | 0.3 | throttle fraction where angle blend switches direct→atan |
| `MOT_VEC_DEADBAND` | 0.03 | steering deadband, freezes last angle |
| `MOT_VEC_RESID_TC` | 0.5 s | throttle filter TC for blend weight |
| `ATC_SPD_EXPO` | 1.0 | speed↔throttle curve exponent (shared by FF, method-2, clamp) |
| `DRIFT_GAIN_NAV` / `DRIFT_GAIN_LOIT` | 1.0 | write-time gains; both 0 = full kill-switch |
| `DRIFT_MAXAGE` | 1800 s | shared staleness cutoff; 0 = never trust |
| `MOT_DRIFT_EST_YAWR` | 15 °/s | nav estimator yaw-rate gate; 0 = off |
| `SFL_DB`/`SFL_GAIN`/`SFL_MAX`/`SFL_IFRZ` | 10°/0.7/20%/45° | steering floor + I-freeze |
| `LOIT_DRIFT_MIN` | 0.03 m/s | anti-drift heading activation |
| `LOIT_COAST_THR` | 1 % | coast gate (method 1) |
| `LOIT_I_EQERR`/`I_MIN`/`I_ALPHA`/`I_DISAG` | 0.10/0.02/0.10/0.5 | method-2 window, noise floor, EMA, disagreement |
| `LOIT_ROT_ANG`/`LOIT_ROT_RATE` | 15°/20 °/s | rotation gate (method 1); 0 = off |

---

## 10. Architectural notes for reviewers

- **Kill-switch:** `DRIFT_GAIN_NAV=0 && DRIFT_GAIN_LOIT=0` restores exact stock behavior — `get_drift_compensation_body()` returns false and `calc_throttle` runs the legacy path. Estimators keep filling (useful for A/B logging comparison).
- **Estimator feedback hygiene:** every estimator input is the uncompensated command; every compensation output is applied to a local copy. Violating either rule creates a subtraction loop that under-estimates drift.
- **Seed safety:** seeds carry original timestamps → guaranteed aging; `seed_weight ≤ 1` → ping-pong decays; real estimates always outrank seeds; same-tick echo blocked by `DRIFT_SEED_MIN_AGE_MS`; real-destination overwrite blocked by the `*_has_real` guard.
- **Units discipline:** speeds m/s, headings centi-degrees, throttle 0–100 % at `calc_throttle` level vs −1..+1 inside `AR_AttitudeControl`, drift estimates always earth-frame NE.

## 11. Known-open items (deliberate)

| Item | Status |
|---|---|
| Speed-PID I-freeze hold-vs-decay | reviewed — current hold-or-decay is **correct**, see §4.4 |
| `MOT_VEC_ANGLEMAX=90` boost spike near 89° | kept — clamping removes turn authority at speed |
| Follow mode: no estimator feed | Follow→Loiter starts cold; documented |
| Slew-rate limiting on vectored angle | optional future improvement if water tests show a throttle stumble |

---

*Caveat: line numbers were taken from git HEAD blame/diffs; the repo search index lags HEAD so exact lines may drift ±a few. `extra_hwdef.dat` was summarized from diff statistics rather than a line-by-line read.*
```

Uložte ako napr. `docs/FORK_CHANGES.md` (alebo `FORK_CHANGES.md` v rootu). Markdown je GitHub-compatible — tabuľky, code fences s `cpp`, nadpisy sa renderujú správne. Ak chcete, môžem pripraviť aj slovenskú verziu vedľa nej.
# ArduPilot Project

[![Discord](https://img.shields.io/discord/674039678562861068.svg)](https://ardupilot.org/discord)

[![Test Copter](https://github.com/ArduPilot/ardupilot/workflows/test%20copter/badge.svg?branch=master)](https://github.com/ArduPilot/ardupilot/actions/workflows/test_sitl_copter.yml) [![Test Plane](https://github.com/ArduPilot/ardupilot/workflows/test%20plane/badge.svg?branch=master)](https://github.com/ArduPilot/ardupilot/actions/workflows/test_sitl_plane.yml) [![Test Rover](https://github.com/ArduPilot/ardupilot/workflows/test%20rover/badge.svg?branch=master)](https://github.com/ArduPilot/ardupilot/actions/workflows/test_sitl_rover.yml) [![Test Sub](https://github.com/ArduPilot/ardupilot/workflows/test%20sub/badge.svg?branch=master)](https://github.com/ArduPilot/ardupilot/actions/workflows/test_sitl_sub.yml) [![Test Tracker](https://github.com/ArduPilot/ardupilot/workflows/test%20tracker/badge.svg?branch=master)](https://github.com/ArduPilot/ardupilot/actions/workflows/test_sitl_tracker.yml)

[![Test AP_Periph](https://github.com/ArduPilot/ardupilot/workflows/test%20ap_periph/badge.svg?branch=master)](https://github.com/ArduPilot/ardupilot/actions/workflows/test_sitl_periph.yml) [![Test Chibios](https://github.com/ArduPilot/ardupilot/workflows/test%20chibios/badge.svg?branch=master)](https://github.com/ArduPilot/ardupilot/actions/workflows/test_chibios.yml) [![Test Linux SBC](https://github.com/ArduPilot/ardupilot/workflows/test%20Linux%20SBC/badge.svg?branch=master)](https://github.com/ArduPilot/ardupilot/actions/workflows/test_linux_sbc.yml) [![Test Replay](https://github.com/ArduPilot/ardupilot/workflows/test%20replay/badge.svg?branch=master)](https://github.com/ArduPilot/ardupilot/actions/workflows/test_replay.yml)

[![Test Unit Tests](https://github.com/ArduPilot/ardupilot/workflows/test%20unit%20tests%20and%20sitl%20building/badge.svg?branch=master)](https://github.com/ArduPilot/ardupilot/actions/workflows/test_unit_tests.yml)[![test size](https://github.com/ArduPilot/ardupilot/actions/workflows/test_size.yml/badge.svg)](https://github.com/ArduPilot/ardupilot/actions/workflows/test_size.yml)

[![Test Environment Setup](https://github.com/ArduPilot/ardupilot/actions/workflows/test_environment.yml/badge.svg?branch=master)](https://github.com/ArduPilot/ardupilot/actions/workflows/test_environment.yml)

[![Cygwin Build](https://github.com/ArduPilot/ardupilot/actions/workflows/cygwin_build.yml/badge.svg)](https://github.com/ArduPilot/ardupilot/actions/workflows/cygwin_build.yml) [![Macos Build](https://github.com/ArduPilot/ardupilot/actions/workflows/macos_build.yml/badge.svg)](https://github.com/ArduPilot/ardupilot/actions/workflows/macos_build.yml)

[![Coverity Scan Build Status](https://scan.coverity.com/projects/5331/badge.svg)](https://scan.coverity.com/projects/ardupilot-ardupilot)

[![Test Coverage](https://github.com/ArduPilot/ardupilot/actions/workflows/test_coverage.yml/badge.svg?branch=master)](https://github.com/ArduPilot/ardupilot/actions/workflows/test_coverage.yml)

[![Autotest Status](https://autotest.ardupilot.org/autotest-badge.svg)](https://autotest.ardupilot.org/)

[![OpenSSF Best Practices](https://www.bestpractices.dev/projects/10598/badge)](https://www.bestpractices.dev/projects/10598)

ArduPilot is the most advanced, full-featured, and reliable open source autopilot software available.
It has been under development since 2010 by a diverse team of professional engineers, computer scientists, and community contributors.
Our autopilot software is capable of controlling almost any vehicle system imaginable, from conventional airplanes, quad planes, multi-rotors, and helicopters to rovers, boats, balance bots, and even submarines.
It is continually being expanded to provide support for new emerging vehicle types.

## The ArduPilot project is made up of

- ArduCopter: [code](https://github.com/ArduPilot/ardupilot/tree/master/ArduCopter), [wiki](https://ardupilot.org/copter/index.html)

- ArduPlane: [code](https://github.com/ArduPilot/ardupilot/tree/master/ArduPlane), [wiki](https://ardupilot.org/plane/index.html)

- Rover: [code](https://github.com/ArduPilot/ardupilot/tree/master/Rover), [wiki](https://ardupilot.org/rover/index.html)

- ArduSub : [code](https://github.com/ArduPilot/ardupilot/tree/master/ArduSub), [wiki](http://ardusub.com/)

- Antenna Tracker : [code](https://github.com/ArduPilot/ardupilot/tree/master/AntennaTracker), [wiki](https://ardupilot.org/antennatracker/index.html)

## User Support & Discussion Forums

- Support Forum: <https://discuss.ardupilot.org/>

- Community Site: <https://ardupilot.org>

## Developer Information

- Github repository: <https://github.com/ArduPilot/ardupilot>

- Main developer wiki: <https://ardupilot.org/dev/>

- Developer discussion: <https://discuss.ardupilot.org>

- Developer chat: <https://discord.com/channels/ardupilot>

## Top Contributors

- [Flight code contributors](https://github.com/ArduPilot/ardupilot/graphs/contributors)
- [Wiki contributors](https://github.com/ArduPilot/ardupilot_wiki/graphs/contributors)
- [Most active support forum users](https://discuss.ardupilot.org/u?order=post_count&period=quarterly)
- [Partners who contribute financially](https://ardupilot.org/about/Partners)

## How To Get Involved

- The ArduPilot project is open source and we encourage participation and code contributions: [guidelines for contributors to the ardupilot codebase](https://ardupilot.org/dev/docs/contributing.html)

- We have an active group of Beta Testers to help us improve our code: [release procedures](https://ardupilot.org/dev/docs/release-procedures.html)

- Desired Enhancements and Bugs can be posted to the [issues list](https://github.com/ArduPilot/ardupilot/issues).

- Help other users with log analysis in the [support forums](https://discuss.ardupilot.org/)

- Improve the wiki and chat with other [wiki editors on Discord #documentation](https://discord.com/channels/ardupilot)

- Contact the developers on one of the [communication channels](https://ardupilot.org/copter/docs/common-contact-us.html)

## License

The ArduPilot project is licensed under the GNU General Public
License, version 3.

- [Overview of license](https://ardupilot.org/dev/docs/license-gplv3.html)

- [Full Text](https://github.com/ArduPilot/ardupilot/blob/master/COPYING.txt)

## Maintainers

ArduPilot is comprised of several parts, vehicles and boards. The list below
contains the people that regularly contribute to the project and are responsible
for reviewing patches on their specific area.

- [Andrew Tridgell](https://github.com/tridge):
  - ***Vehicle***: Plane, AntennaTracker
  - ***Board***: Pixhawk, Pixhawk2, PixRacer
- [Francisco Ferreira](https://github.com/oxinarf):
  - ***Bug Master***
- [Grant Morphett](https://github.com/gmorph):
  - ***Vehicle***: Rover
- [Willian Galvani](https://github.com/williangalvani):
  - ***Vehicle***: Sub
  - ***Board***: Navigator
- [Michael du Breuil](https://github.com/WickedShell):
  - ***Subsystem***: Batteries
  - ***Subsystem***: GPS
  - ***Subsystem***: Scripting
- [Peter Barker](https://github.com/peterbarker):
  - ***Subsystem***: DataFlash, Tools
- [Randy Mackay](https://github.com/rmackay9):
  - ***Vehicle***: Copter, Rover, AntennaTracker
- [Siddharth Purohit](https://github.com/bugobliterator):
  - ***Subsystem***: CAN, Compass
  - ***Board***: Cube*
- [Tom Pittenger](https://github.com/magicrub):
  - ***Vehicle***: Plane
- [Bill Geyer](https://github.com/bnsgeyer):
  - ***Vehicle***: TradHeli
- [Emile Castelnuovo](https://github.com/emilecastelnuovo):
  - ***Board***: VRBrain
- [Georgii Staroselskii](https://github.com/staroselskii):
  - ***Board***: NavIO
- [Gustavo José de Sousa](https://github.com/guludo):
  - ***Subsystem***: Build system
- [Julien Beraud](https://github.com/jberaud):
  - ***Board***: Bebop & Bebop 2
- [Leonard Hall](https://github.com/lthall):
  - ***Subsystem***: Copter attitude control and navigation
- [Matt Lawrence](https://github.com/Pedals2Paddles):
  - ***Vehicle***: 3DR Solo & Solo based vehicles
- [Matthias Badaire](https://github.com/badzz):
  - ***Subsystem***: FRSky
- [Mirko Denecke](https://github.com/mirkix):
  - ***Board***: BBBmini, BeagleBone Blue, PocketPilot
- [Paul Riseborough](https://github.com/priseborough):
  - ***Subsystem***: AP_NavEKF2
  - ***Subsystem***: AP_NavEKF3
- [Víctor Mayoral Vilches](https://github.com/vmayoral):
  - ***Board***: PXF, Erle-Brain 2, PXFmini
- [Amilcar Lucas](https://github.com/amilcarlucas):
  - ***Subsystem***: Marvelmind
- [Samuel Tabor](https://github.com/samuelctabor):
  - ***Subsystem***: Soaring/Gliding
- [Henry Wurzburg](https://github.com/Hwurzburg):
  - ***Subsystem***: OSD
  - ***Site***: Wiki
- [Peter Hall](https://github.com/IamPete1):
  - ***Vehicle***: Tailsitters
  - ***Vehicle***: Sailboat
  - ***Subsystem***: Scripting
- [Andy Piper](https://github.com/andyp1per):
  - ***Subsystem***: Crossfire
  - ***Subsystem***: ESC
  - ***Subsystem***: OSD
  - ***Subsystem***: SmartAudio
- [Alessandro Apostoli](https://github.com/yaapu):
  - ***Subsystem***: Telemetry
  - ***Subsystem***: OSD
- [Rishabh Singh](https://github.com/rishabsingh3003):
  - ***Subsystem***: Avoidance/Proximity
- [David Bussenschutt](https://github.com/davidbuzz):
  - ***Subsystem***: ESP32,AP_HAL_ESP32
- [Charles Villard](https://github.com/Silvanosky):
  - ***Subsystem***: ESP32,AP_HAL_ESP32
