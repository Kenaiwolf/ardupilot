Fork Documentation: Kenaiwolf/ardupilot1 vs upstream ArduPilot master
Base: upstream commit b2b1b3d2 (16 Sep 2026). HEAD: 375952a5 (2 Oct 2026). All fork work is by Kenaiwolf.

Scope: +2,139 / −183 lines across 10 files. Every functional change serves one goal: drift (current/wind) estimation and compensation for a single vectored-thrust (bow thruster) boat, plus the supporting throttle/steering control changes and a slim hardware build definition.

File	+/−	Role
libraries/AR_Motors/AP_MotorsUGV.cpp	+352/−36	drift estimate storage/arbitration, vectored-thrust output, all new parameters
libraries/AR_Motors/AP_MotorsUGV.h	+92/−4	API + parameter members
Rover/mode.cpp	+354/−37	shared drift estimator, apply_drift_compensation, steering floor, cosine reduction, drift feed-forward, seeding
Rover/mode.h	+81/−9	state members, constants
Rover/mode_loiter.cpp	+303/−52	two drift-measurement methods, anti-drift heading, handoff seeding
Rover/mode_guided.cpp	+30/−15	boats loiter instead of stop; estimator feed in HeadingAndSpeed
Rover/GCS_MAVLink_Rover.cpp	+40/−10	drift telemetry named floats
libraries/APM_Control/AR_AttitudeControl.cpp/.h	+26/−13, +11/−7	ATC_SPD_EXPO nonlinear speed↔throttle curve
extra_hwdef.dat	+850/−0	new slim build config
1. ATC_SPD_EXPO — nonlinear throttle curve (AR_AttitudeControl)
Boats have roughly quadratic water resistance, so the stock linear speed → throttle mapping misestimates thrust at low speeds. New parameter ATC_SPD_EXPO (default 1.0) feeds a power curve:

// libraries/APM_Control/AR_AttitudeControl.cpp ~798  
const float speed_ratio = fabsf(_desired_speed) / cruise_speed;  
const float expo = (_speed_thr_expo > 0.0f) ? _speed_thr_expo : 1.0f;  
throttle_base = cruise_throttle * powf(speed_ratio, expo);
Both feed-forward and inverse use this curve — and it is reused everywhere the fork converts throttle↔speed (drift FF in calc_throttle, I-term→speed in loiter method 2), keeping the model self-consistent.

2. Vectored-thrust output rework (AP_MotorsUGV)
Stock code maps (throttle, steering) to a thruster deflection via a single atan2-style mapping. Fork adds a blend between two mappings weighted by filtered throttle magnitude:

w = 1 − |throttle_filt| / MOT_VEC_BLEND_THR (clamped 0–1). Near zero throttle w→1 uses direct mapping (steering request → angle), at high throttle w→0 uses the legacy atan mapping.
steering_angle = w·direct + (1−w)·atan, then throttle /= w + (1−w)·cos(steering_angle) — the 1/cos boost preserves steering authority at speed: the commanded throttle is the forward component, so total thrust must grow with deflection angle. At MOT_VEC_ANGLEMAX=90° the boost diverges toward the singularity, but downstream _throttle_max clamping makes it safe — this was discussed and deliberately left unchanged (the boost is required for turn authority at speed; capping it would weaken turns exactly when needed).
MOT_VEC_DEADBAND freezes the last angle when steering demand is ~0 (prevents jitter/loss of heading memory).
MOT_VEC_RESID_TC = time constant of the throttle filter feeding w.
New getter get_vectored_angle_rad() exposes the actual deflection — used by the loiter rotation gate.
3. Drift estimate storage and arbitration (AP_MotorsUGV)
Two independent estimate slots — loiter-sourced and nav-sourced (Guided/Auto) — each with value, timestamp, and is_seeded flag:

// libraries/AR_Motors/AP_MotorsUGV.cpp ~290-315  
set_loiter_estimate_ne(v)  // real sample: *DRIFT_GAIN_LOIT, ts=now, seeded=false  
set_nav_estimate_ne(v)     // real sample: *DRIFT_GAIN_NAV,  ts=now, seeded=false  
seed_loiter_estimate_ne(v, src_ms) / seed_nav_estimate_ne(v, src_ms)  
                           // handoff copy: keeps ORIGINAL timestamp, seeded=true
Gains are applied at write time so stored values are PID-ready. get_current_estimate_ne() arbitration (~345-380): a real measurement always beats a seeded one regardless of age; between same-kind samples, newer wins; both slots share the DRIFT_MAXAGE staleness cutoff (0 = never trust → compensation fully off). Per-slot getters (get_nav_estimate_ne, get_loiter_estimate_ne) return (value, age_ms, is_seeded) for seeding and telemetry.

4. Rover/mode.cpp — the shared estimator and compensation path
4a. update_drift_estimator(raw_heading_cd, desired_speed) (~560-590)
A 20-second windowed estimator (DRIFT_EST_WINDOW_S) fed from navigate_to_waypoint (Auto/Guided WP) and Guided HeadingAndSpeed with the uncompensated commanded vector (feeding the corrected heading would subtract the correction out of the measurement → systematic underestimate). Gates: DRIFT_EST_MAX_GAP_MS=200 tick-gap freshness, and a yaw-rate gate now driven by the MOT_DRIFT_EST_YAWR parameter (default 15 °/s, 0 = disabled; the constant was promoted to a parameter in A1.1 and its default raised in A1.5 — mode.h:198 carries a comment noting the move). On mode entry the window is reset (_drift_est_window_start_ms=0, window_valid=false) so no stale window survives across sessions.

4b. apply_drift_compensation(heading_cd, speed) (~560-590)
Vector math: own_required_ne = ground_desired_ne − drift_ne → crab-angle heading + required own speed. Special cases:

If |own_required| < 0.05 m/s — drift alone delivers the desired track: command zero speed and face up-drift so a drift change is caught immediately.
Reversing modes flip heading +180° and negate speed.
Magnitude clamped to calc_speed_max(cruise, 1.0) — direction stays optimal (up-current) even when the required speed is unachievable.
Sailboats are excluded: sailboat.use_indirect_route() (tacking) takes the raw pre-compensation bearing — crab correction and no-go-zone logic must not be chained.
get_drift_compensation_body() converts the earth-frame estimate into body frame (x = forward component) — used by the FF and by both loiter measurement methods.

4c. calc_throttle additions (~360-435)
In order, per tick:

_throttle_nav_pct snapshot — PID demand before FF/floor, so the loiter coast gate sees true PID intent.
Cosine throttle reduction — if (is_positive(throttle_out)) throttle_out *= max(0, cos(yaw_error)). Scales only forward thrust; a negative (braking) PID demand passes unchanged so a heavy boat can decelerate during large heading changes (fix committed in 47d03dd0).
Drift feed-forward — throttle += 100·cruise_thr·(|drift_fwd|/cruise)^expo for positive forward drift component. Deliberately placed after cosine reduction (a large crab angle must not erase the thrust holding position) and computed inline — a second get_throttle_out_speed call would corrupt PID I/D state.
Steering thrust floor — above SFL_DB (10°) heading error, forces min(|throttle|, (err−DB)·SFL_GAIN) up to SFL_MAX (20%); never flips sign, only raises magnitude. Gives a vectored-thrust boat rotation authority that pure PID can't produce at zero speed.
I-freeze — above SFL_IFRZ (45°) error, sets throttle limit flags → speed-PID integrator freezes (growth blocked, decay still allowed by AC_PID::update_i).
All of 2–5 require have_vectored_thrust() && steering_heading_fresh — the freshness flag (_steering_heading_active_ms, <50 ms since last calc_steering_to_heading) auto-expires on mode switch or in rate/manual modes.

4d. Kill-switch and A/B path (~703-742)
DRIFT_GAIN_NAV=0 && DRIFT_GAIN_LOIT=0 → drift_comp_active=false → exact original stock path (calc_steering_from_turn_rate) — clean on-water A/B isolation by design.

4e. Mode-entry seeding (Mode::enter, ~53-80)
Autopilot modes seed their nav slot from the loiter slot, age-weighted (seed_weight = 1 − age/MAXAGE). HEAD version (committed 7a411e2d): seeded sources are accepted (timestamp preserved → ping-pong converges to zero, needed for bridge Guided→Loiter→Guided resets), with the nav_has_real guard — never overwrite a fresh real destination sample with a weighted seed copy.

5. Rover/mode_loiter.cpp — where the drift is actually measured
5a. Anti-drift heading
If |drift| > LOIT_DRIFT_MIN (0.03 m/s), desired yaw aims into the drift; a 10% radius hysteresis (LOITER_RADIUS_HYST, _inside_loiter_circle) prevents flapping between "aim at center" and "aim into drift" on the loiter boundary.

5b. Method 1 — coast sampling (~98-144)
Trust a velocity sample only when: distance-to-destination rising for LOITER_DRIFT_RISING_TICKS consecutive ticks (boat being pushed, not decelerating), AND |_throttle_nav_pct| < LOIT_COAST_THR (pre-floor PID demand — floor thrust must not disqualify a true coast), AND not _steer_floor_active, AND not rotating:

// ~119-128 — rotation gate (the fix to earth-frame yaw rate is committed)  
const float vec_angle_deg = fabsf(degrees(g2.motors.get_vectored_angle_rad()));  
const float yaw_rate_degs = fabsf(degrees(ahrs.get_yaw_rate_earth()));  
const bool rotating = (rot_ang_deg > 0 && vec_angle_deg > rot_ang_deg) ||  
                      (rot_rate_dps > 0 && yaw_rate_degs > rot_rate_dps);
Rationale in comment: a deflected bow thruster rotates the hull about its pivot (~0.6 m radius), producing lateral IMU velocity ω·L_pivot up to ~0.5 m/s that is not drift — a single bow thruster can never translate the hull sideways, so lateral velocity while not rotating is pure drift. Gates on cause (thruster angle > LOIT_ROT_ANG=15°) and effect (get_yaw_rate_earth() > LOIT_ROT_RATE=20°/s); 0 disables each branch. Accepted samples have the currently-applied drift-FF push subtracted before storage.

5c. Method 2 — PID I-term residual (~146-240)
During the decel-to-stop window the speed PID's I-term is the thrust holding position against drift. Equilibrium window = desired speed 0, actual ≤ stop speed, |error| ≤ LOIT_I_EQERR, no floor. Inside it:

|I| ≥ LOIT_I_MIN → EMA-filter (LOIT_I_ALPHA), convert to speed via the inverse expo curve, add back the reconstructed drift-FF fraction (I only holds the residual after FF — without this a fully compensated drift reads as I≈0 and the zero path would erase a correct estimate), clamp to calc_speed_max (the cruise_speed→calc_speed_max fix is committed at lines 188-193).
|I| < LOIT_I_MIN for LOITER_DRIFT_ZERO_TICKS (5) → accept a zero-drift sample (blended, not a jump) — otherwise the last nonzero estimate would persist until DRIFT_MAXAGE after the current dies.
Direction measured every tick: residual velocity direction while still sliding; at full standstill, drift = opposite of thrust direction (thrust_rad = yaw + steer_angle, bow-pull convention documented at lines 223-225 — fix 4 committed). New direction blends with prior estimate (0.35/0.65); disagreement gate (LOIT_I_DISAG=0.5) limits jumps to 25% of the step.
5d. Handoff seeding (_enter, ~25-42)
Mirror of Mode::enter: loiter seeds from the nav slot, age-weighted, same seeded-acceptance + has_real guard semantics.

6. Rover/mode_guided.cpp
Boats go to Loiter instead of Stop at every stop point (entry failure, waypoint reached, all submode timeouts), fallback to stop — a dead boat drifts; a loitering one holds position via the whole machinery above. (Side effect: RMB-link loss produces the Guided→Loiter→Guided bounce the seeding redesign accommodates.)
HeadingAndSpeed feeds the estimator with the uncompensated _desired_yaw_cd, then applies compensation to a local copy — _desired_yaw_cd is the persistent commanded target re-read every tick; writing the crab correction into it would compound cycle-over-cycle.
7. Rover/GCS_MAVLink_Rover.cpp
send_nav_controller_output extended with named-float telemetry: DRIFTN/DRIFTE/DRIFTSPD (current estimate, sent only when fresh) plus per-slot diagnostics DRIFTNAGE/DRIFTNSED and DRIFTLAGE/DRIFTLSED (sample age in seconds — a growing age means gates are blocking sampling — and seeded flag 0/1). Gated to MAVLINK_COMM_0 because the send function runs once per channel.

8. extra_hwdef.dat (~850 lines, new file)
Slim Rover build: ~470 feature undefs plus explicit define … 0 — copter/plane modes, most rangefinder/mount/camera/OSD/CAN bindings, EKF2 (HAL_NAVEKF2_AVAILABLE 0), airspeed, and most GPS/compass drivers removed (kept: uBlox, NMEA, IST8310). Targeted at a specific low-flash flight controller.

Parameter summary (all new, in AP_MotorsUGV)
Param	Default	Purpose
MOT_VEC_BLEND_THR	0.3	throttle fraction where angle blend switches direct→atan
MOT_VEC_DEADBAND	0.03	steering deadband, freezes last angle
MOT_VEC_RESID_TC	0.5 s	throttle filter TC for blend weight
ATC_SPD_EXPO	1.0	speed↔throttle curve exponent
DRIFT_GAIN_NAV / DRIFT_GAIN_LOIT	1.0	write-time gains; both 0 = full kill-switch
DRIFT_MAXAGE	1800 s	shared staleness cutoff; 0 = never trust
MOT_DRIFT_EST_YAWR	15 °/s	nav estimator yaw-rate gate; 0 = off
SFL_DB/SFL_GAIN/SFL_MAX/SFL_IFRZ	10°/0.7/20%/45°	steering floor + I-freeze
LOIT_DRIFT_MIN	0.03 m/s	anti-drift heading activation
LOIT_COAST_THR	1 %	coast gate (method 1)
LOIT_I_EQERR/I_MIN/I_ALPHA/I_DISAG	0.10/0.02/0.10/0.5	method-2 window, noise floor, EMA, disagreement
LOIT_ROT_ANG/LOIT_ROT_RATE	15°/20 °/s	rotation gate (method 1); 0 = off
Known-open items at HEAD (from on-water test review)
I-freeze remains direction-undifferentiated by design (AC_PID already permits shrinkage; a true hold would need i_scale=0 plumbing — deferred).
MOT_VEC_ANGLEMAX=90 boost spike near 89° — deliberately kept (authority argument won).
Follow mode has no estimator feed — Follow→Loiter still starts cold (documented, not fixed).
Caveats
Analysis is based on git HEAD blame/diffs — the search index lags HEAD, so per-line numbers may drift ±a few lines. extra_hwdef.dat was summarized from diff stat rather than read line-by-line. Minor commits (upload series, comment-only commits) were covered through their effect on final file state, not individuall

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
