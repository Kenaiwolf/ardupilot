#include "Rover.h"

Mode::Mode() :
    ahrs(rover.ahrs),
    g(rover.g),
    g2(rover.g2),
    channel_steer(rover.channel_steer),
    channel_throttle(rover.channel_throttle),
    channel_lateral(rover.channel_lateral),
    channel_roll(rover.channel_roll),
    channel_pitch(rover.channel_pitch),
    channel_walking_height(rover.channel_walking_height),
    attitude_control(g2.attitude_control)
{ }

void Mode::exit()
{
    // call sub-classes exit
    _exit();
}

bool Mode::enter()
{
    const bool ignore_checks = !rover.arming.is_armed();   // allow switching to any mode if disarmed.  We rely on the arming check to perform
    if (!ignore_checks) {

        // get EKF filter status
        nav_filter_status filt_status;
        rover.ahrs.get_filter_status(filt_status);

        // check position estimate.  requires origin and at least one horizontal position flag to be true
        const bool position_ok = rover.ekf_position_ok() && !rover.failsafe.ekf;
        if (requires_position() && !position_ok) {
            return false;
        }

        // check velocity estimate (if we have position estimate, we must have velocity estimate)
        if (requires_velocity() && !position_ok && !filt_status.flags.horiz_vel) {
            return false;
        }
    }

    bool ret = _enter();  
  
    // initialisation common to all modes  
    if (ret) {  
        // init reversed flag  
        init_reversed_flag();  
  
        // clear sailboat tacking flags  
        g2.sailboat.clear_tack();  
  
        // drift-estimate handoff: only autopilot modes (Auto/Guided/RTL/SmartRTL)  
        // seed from whatever Loiter last measured (Loiter has its own independent slot)  
        if (is_autopilot_mode() && (mode_number() != Number::LOITER)) {
    // reset drift estimator window - avoids stale window spanning across mode sessions  
    _drift_est_window_start_ms = 0;  
    _drift_est_window_valid = false;  
  
    // handoff: seed our own nav-sourced drift estimate from whatever Loiter
    // last measured, weighted down by how long ago Loiter's value was written.  
    // this avoids starting "cold" (0,0) every time we enter Guided, while still  
    // never blending a live Guided sample with a live Loiter sample (they can't  
    // run at the same time, so there is nothing to blend concurrently - this is  
    // a one-shot seed applied only at mode entry).  
    // minimum age (ms) a source estimate must have before we trust it as a genuine  
    // independent measurement rather than a same-tick echo written by the other  
    // mode's own _enter() re-entering back into us during this same call chain  
    // (e.g. ModeGuided::_enter() -> start_loiter() -> ModeLoiter::_enter() seeding  
    // back from the value we *just* wrote below).  order-independent guard.  
  
    Vector2f loiter_ne;    
    uint32_t loiter_age_ms = 0;  
    bool loiter_is_seeded = false;  
    if (g2.motors.get_loiter_estimate_ne(loiter_ne, loiter_age_ms, loiter_is_seeded) &&  
        !loiter_is_seeded && (loiter_age_ms >= DRIFT_SEED_MIN_AGE_MS) &&
        (loiter_age_ms < uint32_t(g2.motors.get_drift_max_age_s() * 1000.0f))) {    
        const float age_s = loiter_age_ms * 0.001f;    
        const float max_age_s = MAX(g2.motors.get_drift_max_age_s(), 0.1f);    
        const float seed_weight = constrain_float(1.0f - (age_s / max_age_s), 0.0f, 1.0f);    
        g2.motors.seed_nav_estimate_ne(loiter_ne * seed_weight, AP_HAL::millis() - loiter_age_ms);  
    }  
        }  
    }  
  
    return ret;  
}

// decode pilot steering and throttle inputs and return in steer_out and throttle_out arguments
// steering_out is in the range -4500 ~ +4500 with positive numbers meaning rotate clockwise
// throttle_out is in the range -100 ~ +100
void Mode::get_pilot_input(float &steering_out, float &throttle_out) const
{
    // no RC input means no throttle and centered steering
    if (rover.failsafe.bits & FAILSAFE_EVENT_THROTTLE) {
        steering_out = 0;
        throttle_out = 0;
        return;
    }

    // apply RC skid steer mixing
    switch ((PilotSteerType)g.pilot_steer_type.get())
    {
        case PilotSteerType::DEFAULT:
        case PilotSteerType::DIR_REVERSED_WHEN_REVERSING:
        default: {
            // by default regular and skid-steering vehicles reverse their rotation direction when backing up
            throttle_out = rover.channel_throttle->get_control_in();
            const float steering_dir = is_negative(throttle_out) ? -1 : 1;
            steering_out = steering_dir * rover.channel_steer->get_control_in();
            break;
        }

        case PilotSteerType::TWO_PADDLES: {
            // convert the two radio_in values from skid steering values
            // left paddle from steering input channel, right paddle from throttle input channel
            // steering = left-paddle - right-paddle
            // throttle = average(left-paddle, right-paddle)
            const float left_paddle = rover.channel_steer->norm_input_dz();
            const float right_paddle = rover.channel_throttle->norm_input_dz();

            throttle_out = 0.5f * (left_paddle + right_paddle) * 100.0f;
            steering_out = (left_paddle - right_paddle) * 0.5f * 4500.0f;
            break;
        }

        case PilotSteerType::DIR_UNCHANGED_WHEN_REVERSING: {
            throttle_out = rover.channel_throttle->get_control_in();
            steering_out = rover.channel_steer->get_control_in();
            break;
        }
    }
}

// decode pilot steering and throttle inputs and return in steer_out and throttle_out arguments
// steering_out is in the range -4500 ~ +4500 with positive numbers meaning rotate clockwise
// throttle_out is in the range -100 ~ +100
void Mode::get_pilot_desired_steering_and_throttle(float &steering_out, float &throttle_out) const
{
    // do basic conversion
    get_pilot_input(steering_out, throttle_out);

    // for skid steering vehicles, if pilot commands would lead to saturation
    // we proportionally reduce steering and throttle
    if (g2.motors.have_skid_steering()) {
        const float steer_normalised = constrain_float(steering_out / 4500.0f, -1.0f, 1.0f);
        const float throttle_normalised = constrain_float(throttle_out * 0.01f, -1.0f, 1.0f);
        const float saturation_value = fabsf(steer_normalised) + fabsf(throttle_normalised);
        if (saturation_value > 1.0f) {
            steering_out /= saturation_value;
            throttle_out /= saturation_value;
        }
    }

    // check for special case of input and output throttle being in opposite directions
    float throttle_out_limited = g2.motors.get_slew_limited_throttle(throttle_out, rover.G_Dt);
    if ((is_negative(throttle_out) != is_negative(throttle_out_limited)) &&
        (g.pilot_steer_type == PilotSteerType::DEFAULT ||
         g.pilot_steer_type == PilotSteerType::DIR_REVERSED_WHEN_REVERSING)) {
        steering_out *= -1;
    }
    throttle_out = throttle_out_limited;
}

// decode pilot steering and return steering_out and speed_out (in m/s)
void Mode::get_pilot_desired_steering_and_speed(float &steering_out, float &speed_out) const
{
    float desired_throttle;
    get_pilot_input(steering_out, desired_throttle);
    speed_out = desired_throttle * 0.01f * calc_speed_max(g.speed_cruise, g.throttle_cruise * 0.01f);
    // check for special case of input and output throttle being in opposite directions
    float speed_out_limited = g2.attitude_control.get_desired_speed_accel_limited(speed_out, rover.G_Dt);
    if ((is_negative(speed_out) != is_negative(speed_out_limited)) &&
        (g.pilot_steer_type == PilotSteerType::DEFAULT ||
         g.pilot_steer_type == PilotSteerType::DIR_REVERSED_WHEN_REVERSING)) {
        steering_out *= -1;
    }
    speed_out = speed_out_limited;
}

// decode pilot lateral movement input and return in lateral_out argument
void Mode::get_pilot_desired_lateral(float &lateral_out) const
{
    // no RC input means no lateral input
    if ((rover.failsafe.bits & FAILSAFE_EVENT_THROTTLE) || (rover.channel_lateral == nullptr)) {
        lateral_out = 0;
        return;
    }

    // get pilot lateral input
    lateral_out = rover.channel_lateral->get_control_in();
}

// decode pilot's input and return heading_out (in cd) and speed_out (in m/s)
void Mode::get_pilot_desired_heading_and_speed(float &heading_out, float &speed_out) const
{
    // get steering and throttle in the -1 to +1 range
    float desired_steering = constrain_float(rover.channel_steer->norm_input_dz(), -1.0f, 1.0f);
    float desired_throttle = constrain_float(rover.channel_throttle->norm_input_dz(), -1.0f, 1.0f);

    // handle two paddle input
    if (g.pilot_steer_type == PilotSteerType::TWO_PADDLES) {
        const float left_paddle = desired_steering;
        const float right_paddle = desired_throttle;
        desired_steering = (left_paddle - right_paddle) * 0.5f;
        desired_throttle = (left_paddle + right_paddle) * 0.5f;
    }

    // calculate angle of input stick vector
    heading_out = wrap_360_cd(rad_to_cd(atan2f(desired_steering, desired_throttle)));

    // calculate throttle using magnitude of input stick vector
    const float throttle = MIN(safe_sqrt(sq(desired_throttle) + sq(desired_steering)), 1.0f);
    speed_out = throttle * calc_speed_max(g.speed_cruise, g.throttle_cruise * 0.01f);
}

// decode pilot roll and pitch inputs and return in roll_out and pitch_out arguments
// outputs are in the range -1 to +1
void Mode::get_pilot_desired_roll_and_pitch(float &roll_out, float &pitch_out) const
{
    if (channel_roll != nullptr) {
        roll_out = channel_roll->norm_input();
    } else {
        roll_out = 0.0f;
    }
    if (channel_pitch != nullptr) {
        pitch_out = channel_pitch->norm_input();
    } else {
        pitch_out = 0.0f;
    }
}

// decode pilot walking_height inputs and return in walking_height_out arguments
// outputs are in the range -1 to +1
void Mode::get_pilot_desired_walking_height(float &walking_height_out) const
{
    if (channel_walking_height != nullptr) {
        walking_height_out = channel_walking_height->norm_input();
    } else {
        walking_height_out = 0.0f;
    }
}

// return heading (in degrees) to target destination (aka waypoint)
float Mode::wp_bearing() const
{
    if (!is_autopilot_mode()) {
        return 0.0f;
    }
    return g2.wp_nav.wp_bearing_cd() * 0.01f;
}

// return short-term target heading in degrees (i.e. target heading back to line between waypoints)
float Mode::nav_bearing() const
{
    if (!is_autopilot_mode()) {
        return 0.0f;
    }
    return g2.wp_nav.nav_bearing_cd() * 0.01f;
}

// return cross track error (i.e. vehicle's distance from the line between waypoints)
float Mode::crosstrack_error_m() const
{
    if (!is_autopilot_mode()) {
        return 0.0f;
    }
    return g2.wp_nav.crosstrack_error_m();
}

// return desired lateral acceleration
float Mode::get_desired_lat_accel() const
{
    if (!is_autopilot_mode()) {
        return 0.0f;
    }
    return g2.wp_nav.get_lat_accel();
}

// set desired location
bool Mode::set_desired_location(const Location &destination, Location next_destination )
{
    if (!g2.wp_nav.set_desired_location(destination, next_destination)) {
        return false;
    }

    // initialise distance
    _distance_to_destination = g2.wp_nav.get_distance_to_destination();
    _reached_destination = false;

    return true;
}

// get default speed for this mode (held in WP_SPEED or RTL_SPEED)
float Mode::get_speed_default(bool rtl) const
{
    if (rtl && is_positive(g2.rtl_speed)) {
        return g2.rtl_speed;
    }

    return g2.wp_nav.get_default_speed();
}

// execute the mission in reverse (i.e. backing up)
void Mode::set_reversed(bool value)
{
    g2.wp_nav.set_reversed(value);
}

// handle tacking request (from auxiliary switch) in sailboats
void Mode::handle_tack_request()
{
    // autopilot modes handle tacking
    if (is_autopilot_mode()) {
        g2.sailboat.handle_tack_request_auto();
    }
}

void Mode::calc_throttle(float target_speed, bool avoidance_enabled)
{
    // get acceleration limited target speed
    target_speed = attitude_control.get_desired_speed_accel_limited(target_speed, rover.G_Dt);

#if AP_AVOIDANCE_ENABLED
    // apply object avoidance to desired speed using half vehicle's maximum deceleration
    if (avoidance_enabled) {
        g2.avoid.adjust_speed(0.0f, 0.5f * attitude_control.get_decel_max(), ahrs.get_yaw_rad(), target_speed, rover.G_Dt);
        if (g2.sailboat.tack_enabled() && g2.avoid.limits_active()) {
            // we are a sailboat trying to avoid fence, try a tack
            if (rover.control_mode != &rover.mode_acro) {
                rover.control_mode->handle_tack_request();
            }
        }
    }
#endif  // AP_AVOIDANCE_ENABLED

    // call throttle controller and convert output to -100 to +100 range
    float throttle_out = 0.0f;

    if (g2.sailboat.sail_enabled()) {
        // sailboats use special throttle and mainsail controller
        g2.sailboat.get_throttle_and_set_mainsail(target_speed, throttle_out);
    } else {
        // call speed or stop controller
        if (is_zero(target_speed) && !rover.is_balancebot()) {
            bool stopped;
            throttle_out = 100.0f * attitude_control.get_throttle_out_stop(g2.motors.limit.throttle_lower, g2.motors.limit.throttle_upper, g.speed_cruise, g.throttle_cruise * 0.01f, rover.G_Dt, stopped);
        } else {
            bool motor_lim_low = g2.motors.limit.throttle_lower || attitude_control.pitch_limited();
            bool motor_lim_high = g2.motors.limit.throttle_upper || attitude_control.pitch_limited();
            throttle_out = 100.0f * attitude_control.get_throttle_out_speed(target_speed, motor_lim_low, motor_lim_high, g.speed_cruise, g.throttle_cruise * 0.01f, rover.G_Dt);
        }

        // if vehicle is balance bot, calculate actual throttle required for balancing
        if (rover.is_balancebot()) {
            rover.balancebot_pitch_control(throttle_out);
        }
    }

    // forward drift/current feed-forward: added directly to throttle output,
    // never to target_speed, so it doesn't shift the PID's ground-speed setpoint
    Vector2f drift_body;  
    if (get_drift_compensation_body(drift_body)) {  
        const float expo = attitude_control.get_speed_thr_expo();  
        const float cruise_speed = MAX(g.speed_cruise, 0.1f);  
        const float speed_ratio = fabsf(target_speed) / cruise_speed;  
        const float local_slope = (g.throttle_cruise * expo / cruise_speed) * powf(MAX(speed_ratio, 0.01f), expo - 1.0f);  
    // gain already applied at write-time - see get_drift_compensation_body()    
        throttle_out += drift_body.x * local_slope;  
    }  
  
    // record pure navigation/PID throttle before floor injection so drift    
    // samplers can detect true coasting regardless of steering-floor activity    
    _throttle_nav_pct = throttle_out;    
    
    // steering-to-throttle floor and runaway prevention (vectored-thrust safety fix)    
    if (g2.motors.have_vectored_thrust() &&
        _steering_heading_active_ms != 0 &&  
        (AP_HAL::millis() - _steering_heading_active_ms) < 50) {
        const float yaw_error_rad = wrap_180_cd(_desired_yaw_cd - ahrs.yaw_sensor) * (radians(1.0f) * 0.01f);  
        const float yaw_error_deg = fabsf(degrees(yaw_error_rad));  
  
        // 3. I-term Windup Suppression: Freeze integration if heading error is critical  
        // (Threshold configured to 45 degrees as standard safety limit)  
        if (yaw_error_deg > STEER_THR_FLOOR_I_FREEZE_DEG) {  
            // Signal upstream speed/position PID to stop integrating  
            g2.motors.limit.throttle_upper = true;  
            g2.motors.limit.throttle_lower = true;  
        }  
  
        // 1. Cosine Throttle Reduction: reduce forward throttle at large heading error  
        throttle_out *= MAX(0.0f, cosf(yaw_error_rad));  
  
        // 2. Steering Thrust Floor: force minimum throttle to allow rotation  
        if (yaw_error_deg > STEER_THR_FLOOR_DEADBAND_DEG) {  
            const float steer_throttle_floor = constrain_float(  
                (yaw_error_deg - STEER_THR_FLOOR_DEADBAND_DEG) * STEER_THR_FLOOR_GAIN_PCT_PER_DEG,  
                0.0f, STEER_THR_FLOOR_MAX_PCT);  
            // do not override a throttle that is already stronger than the floor in    
            // the same direction; only raise the magnitude, never flip its sign    
            if (fabsf(throttle_out) < steer_throttle_floor) {    
                throttle_out = is_negative(throttle_out) ? -steer_throttle_floor : steer_throttle_floor;    
                _steer_floor_active = true;    
            }    
        } else {    
            _steer_floor_active = false;    
        }    
    } else {    
        _steer_floor_active = false;    
    } else {    
        _steer_floor_active = false;    
    } 
  
    // send to motor    
    g2.motors.set_throttle(throttle_out);
}

// performs a controlled stop without turning
bool Mode::stop_vehicle()
{
    // call throttle controller and convert output to -100 to +100 range
    bool stopped = false;
    float throttle_out;

    // if vehicle is balance bot, calculate throttle required for balancing
    if (rover.is_balancebot()) {
        throttle_out = 100.0f * attitude_control.get_throttle_out_speed(0, g2.motors.limit.throttle_lower, g2.motors.limit.throttle_upper, g.speed_cruise, g.throttle_cruise * 0.01f, rover.G_Dt);
        rover.balancebot_pitch_control(throttle_out);
    } else {
        throttle_out = 100.0f * attitude_control.get_throttle_out_stop(g2.motors.limit.throttle_lower, g2.motors.limit.throttle_upper, g.speed_cruise, g.throttle_cruise * 0.01f, rover.G_Dt, stopped);
    }

    // relax sails if present
    g2.sailboat.relax_sails();

    // send to motor
    g2.motors.set_throttle(throttle_out);

    // do not turn while slowing down
    float steering_out = 0.0;
    if (!stopped) {
        steering_out = attitude_control.get_steering_out_rate(0.0, g2.motors.limit.steer_left, g2.motors.limit.steer_right, rover.G_Dt);
    }
    g2.motors.set_steering(steering_out * 4500.0);

    // return true once stopped
    return stopped;
}

// estimate maximum vehicle speed (in m/s)
// cruise_speed is in m/s, cruise_throttle should be in the range -1 to +1
float Mode::calc_speed_max(float cruise_speed, float cruise_throttle) const
{
    float speed_max;

    // sanity checks
    if (cruise_throttle > 1.0f || cruise_throttle < 0.05f) {
        speed_max = cruise_speed;
    } else if (is_positive(g2.speed_max)) {
        speed_max = g2.speed_max;
    } else {
        // project vehicle's maximum speed
        speed_max = (1.0f / cruise_throttle) * cruise_speed;
    }

    // constrain to 100m/s and return
    return constrain_float(speed_max, 0.0f, 100.0f);
}

// calculate pilot input to nudge speed up or down
//  target_speed should be in meters/sec
//  reversed should be true if the vehicle is intentionally backing up which allows the pilot to increase the backing up speed by pulling the throttle stick down
float Mode::calc_speed_nudge(float target_speed, bool reversed)
{
    // sanity checks
    if (g.throttle_cruise > 100 || g.throttle_cruise < 5) {
        return target_speed;
    }

    // convert pilot throttle input to speed
    float pilot_steering, pilot_throttle;
    get_pilot_input(pilot_steering, pilot_throttle);
    float pilot_speed = pilot_throttle * 0.01f * calc_speed_max(g.speed_cruise, g.throttle_cruise * 0.01f);

    // ignore pilot's input if in opposite direction to vehicle's desired direction of travel
    // note that the target_speed may be negative while reversed is true (or vice-versa)
    // while vehicle is transitioning between forward and backwards movement
    if ((is_positive(pilot_speed) && reversed) ||
        (is_negative(pilot_speed) && !reversed)) {
        return target_speed;
    }

    // return the larger of the pilot speed and the original target speed
    if (reversed) {
        return MIN(target_speed, pilot_speed);
    } else {
        return MAX(target_speed, pilot_speed);
    }
}

// retrieve a current/wind drift estimate rotated into body frame (x=forward, y=right), m/s
bool Mode::get_drift_compensation_body(Vector2f &drift_body) const
{
    if (g2.sailboat.sail_enabled()) {  
        return false;  
    } 
    Vector2f drift_ne;
    bool have_estimate = g2.motors.get_current_estimate_ne(drift_ne);
    if (!have_estimate) {
        return false;
    }
    drift_body = AP::ahrs().earth_to_body2D(drift_ne);
    return true;
}

// apply drift compensation to a desired heading (centi-degrees) and speed (m/s)
void Mode::apply_drift_compensation(float &desired_heading_cd, float &desired_speed) const  
{  
    // sailboats already have their own wind-relative heading logic  
    // (Sailboat::use_indirect_route()/calc_heading(), gated on true wind direction).  
    // For a sailboat, drift is wind-dominated, and true wind is itself computed from  
    // apparent wind + GPS ground velocity -- i.e. the same physical quantity this  
    // compensation estimates. Applying both would risk double-correction/windup.  
    // Exclude sailboats entirely rather than trying to reconcile the two.  
    if (g2.sailboat.sail_enabled()) {  
        return;  
    }
    Vector2f drift_body;
    if (!get_drift_compensation_body(drift_body)) {
        return;
    }

    // gain applied at write-time (ModeLoiter::update() / Mode::update_drift_estimator()) 
    // so the stored estimate is already the final calibrated correction - no further scaling here.  
    if (drift_body.is_zero()) {  
        return;  
    }

    // lateral component: crab-angle correction. sin(theta) = lateral / own-speed
    // floor own-speed to the drift magnitude itself (not a small fixed constant) so that
    // holding position against real current/wind actually commands enough forward speed
    // to have a chance of cancelling it, rather than just avoiding a divide-by-zero.
    // saturates (clips to +-90deg) if drift still exceeds the vehicle's achievable speed --
    // this means the vehicle physically cannot cancel the drift at the current speed target.
    const float speed_abs = MAX(fabsf(desired_speed), drift_body.length());
    {
        const float sin_theta = constrain_float(drift_body.y / speed_abs, -1.0f, 1.0f);  
        const float crab_angle_cd = degrees(asinf(sin_theta)) * 100.0f;  
        desired_heading_cd = wrap_360_cd(desired_heading_cd + crab_angle_cd);  
    }  
}  
  
// shared drift estimator - commanded vs actual NE displacement over a window  
void Mode::update_drift_estimator(float commanded_heading_cd, float commanded_speed_ms)  
{  
    const uint32_t now_ms = AP_HAL::millis();  
  
    // gate: only accumulate while conditions are steady enough to trust sample  
    const bool yaw_rate_ok = fabsf(degrees(ahrs.get_yaw_rate_earth())) < DRIFT_EST_MAX_YAW_RATE_DPS;  
    Vector2p current_pos_ne;  
    const bool pos_ok = ahrs.get_relative_position_NE_origin(current_pos_ne);  
    const bool gate_ok = yaw_rate_ok && pos_ok;  
  
    // (re)start the window if it never started, or if a gate failure invalidated it  
    if ((_drift_est_window_start_ms == 0) || !_drift_est_window_valid) {  
        if (!gate_ok) {  
            // can't even start a valid window this tick, nothing to do  
            _drift_est_window_start_ms = 0;  
            return;  
        }  
        _drift_est_window_start_ms = now_ms;  
        _drift_est_last_update_ms = now_ms;  
        _drift_est_actual_pos_start_ne_m = current_pos_ne;  
        _drift_est_predicted_disp_ne_m.zero();  
        _drift_est_ekf_reset_count = ahrs.get_position_NE_reset_count();  
        _drift_est_window_valid = true;  
        return;  
    }
  
    // fix 2: if a gate fails mid-window, invalidate the whole window instead of  
    // silently truncating it - a partially-steady window is not trustworthy  
    if (!gate_ok) {  
        _drift_est_window_valid = false;  
        _drift_est_window_start_ms = 0;  
        return;  
    }  
  
    // fix 4: invalidate window on EKF position reset or a stale tick gap -  
    // either means the accumulated predicted displacement no longer matches reality  
    if ((ahrs.get_position_NE_reset_count() != _drift_est_ekf_reset_count) ||  
        ((now_ms - _drift_est_last_update_ms) > DRIFT_EST_MAX_GAP_MS)) {
        _drift_est_window_valid = false;  
        _drift_est_window_start_ms = 0;  
        return;  
    } 
  
    // accumulate the commanded (pre-compensation) NE displacement for this tick  
    const float dt = (now_ms - _drift_est_last_update_ms) * 0.001f;  
    _drift_est_last_update_ms = now_ms;  
    const float heading_rad = radians(commanded_heading_cd * 0.01f);  
    _drift_est_predicted_disp_ne_m.x += cosf(heading_rad) * commanded_speed_ms * dt;  
    _drift_est_predicted_disp_ne_m.y += sinf(heading_rad) * commanded_speed_ms * dt;  
  
    // window not finished yet  
    if ((now_ms - _drift_est_window_start_ms) < uint32_t(DRIFT_EST_WINDOW_S * 1000.0f)) {  
        return;  
    }  
  
    // window complete: predicted displacement (from our own commands) vs actual  
    // displacement (from EKF) - the difference is what the environment did to us  
    const Vector2f actual_disp_ne_m = (current_pos_ne - _drift_est_actual_pos_start_ne_m).tofloat();  
    const Vector2f predicted_disp_ne_m = _drift_est_predicted_disp_ne_m.tofloat();  
    const float window_s = (now_ms - _drift_est_window_start_ms) * 0.001f;  
    const Vector2f drift_ne_ms = (actual_disp_ne_m - predicted_disp_ne_m) / window_s;  
  
    // fix 3: simple disagreement gate against whatever is currently stored -  
    // if the new sample disagrees too much with the current estimate, down-weight  
    // it instead of blending it in at full trust. this protects against a single  
    // noisy/short window corrupting a previously-good (e.g. Loiter-sourced) estimate.  
    Vector2f current_ne;  
    Vector2f drift_to_store = drift_ne_ms;  
    uint32_t unused_age_ms = 0;  
    bool unused_is_seeded = false;  
    if (g2.motors.get_nav_estimate_ne(current_ne, unused_age_ms, unused_is_seeded)) {  
        const float disagreement = (drift_ne_ms - current_ne).length();  
        if (disagreement > DRIFT_EST_DISAGREE_GATE_MPS) {  
            // don't discard outright - the environment may genuinely have changed -  
            // but only move a fraction of the way towards the new sample this time  
            drift_to_store = current_ne + (drift_ne_ms - current_ne) * 0.25f;  
        }  
    }  
  
    // gain (DRIFT_GAIN_NAV) is already applied inside set_nav_estimate_ne()  
    // itself, at write-time - do not apply it again here.  
    g2.motors.set_nav_estimate_ne(drift_to_store);  
  
    // start next window  
    _drift_est_window_start_ms = now_ms;  
    _drift_est_last_update_ms = now_ms;  
    _drift_est_actual_pos_start_ne_m = current_pos_ne;  
    _drift_est_predicted_disp_ne_m.zero();  
}

// high level call to navigate to waypoint
// uses wp_nav to calculate turn rate and speed to drive along the path from origin to destination
// this function updates _distance_to_destination
void Mode::navigate_to_waypoint()
{
    // apply speed nudge from pilot
    // calc_speed_nudge's "desired_speed" argument should be negative when vehicle is reversing
    // AR_WPNav nudge_speed_max argu,ent should always be positive even when reversing
    const float calc_nudge_input_speed = g2.wp_nav.get_speed_max() * (g2.wp_nav.get_reversed() ? -1.0 : 1.0);
    const float nudge_speed_max = calc_speed_nudge(calc_nudge_input_speed, g2.wp_nav.get_reversed());
    g2.wp_nav.set_nudge_speed_max(fabsf(nudge_speed_max));

    // update navigation controller
    g2.wp_nav.update(rover.G_Dt);
    _distance_to_destination = g2.wp_nav.get_distance_to_destination();

#if AP_AVOIDANCE_ENABLED
    // sailboats trigger tack if simple avoidance becomes active
    if (g2.sailboat.tack_enabled() && g2.avoid.limits_active()) {
        // we are a sailboat trying to avoid fence, try a tack
        rover.control_mode->handle_tack_request();
    }
#endif

    // pass desired speed to throttle controller, biased by any current/wind drift estimate
    // do not do simple avoidance because this is already handled in the position controller
    float desired_speed = g2.wp_nav.get_speed();  
    const float raw_heading_cd = g2.wp_nav.oa_wp_bearing_cd();  
    float desired_heading_cd = raw_heading_cd;  
    update_drift_estimator(raw_heading_cd, desired_speed);  
    apply_drift_compensation(desired_heading_cd, desired_speed);  
    calc_throttle(desired_speed, false);

    // true only when drift compensation can actually be contributing anything --  
    // used to fully restore the original calc_steering_from_turn_rate() path when  
    // there is no valid (non-stale, non-zero) drift estimate from either source,  
    // so DRIFT_GAIN_LOIT=0 and DRIFT_GAIN_NAV=0 together still act as a real  
    // kill-switch for on-water isolation testing.  gain is now baked in at  
    // write-time (see AP_MotorsUGV::set_loiter/nav_estimate_ne()), so this  
    // reads the already-corrected vector instead of a separate gain param.  
    Vector2f drift_comp_check_body;  
    const bool drift_comp_active = get_drift_compensation_body(drift_comp_check_body) &&  
        !drift_comp_check_body.is_zero();

    if (g2.sailboat.use_indirect_route(raw_heading_cd)) {  
        // sailboats use heading controller when tacking upwind (unaffected by drift gain,  
        // this path pre-dates the drift-compensation work).  use the raw, pre-drift-  
        // compensation bearing here - the drift estimator's crab-angle correction and  
        // the sailboat's no-go-zone/tack decision both react to wind, and must not be  
        // chained onto each other.  
        desired_heading_cd = g2.sailboat.calc_heading(raw_heading_cd);
        // use pivot turn rate for tacks
        const float turn_rate = g2.sailboat.tacking() ? g2.wp_nav.get_pivot_rate() : 0.0f;
        calc_steering_to_heading(desired_heading_cd, turn_rate);
    } else if (drift_comp_active) {
        // if simple avoidance is active at very low speed do not attempt to turn
#if AP_AVOIDANCE_ENABLED
        if (g2.avoid.limits_active() && (fabsf(attitude_control.get_desired_speed()) <= attitude_control.get_stop_speed())) {
            calc_steering_from_turn_rate(0.0f);
        } else
#endif
        {
            // drive to the drift-corrected heading (includes crab-angle bias from
            // apply_drift_compensation()). a rate-only controller cannot hold a
            // steady heading offset (it would integrate forever), so this must go
            // through the heading-closed-loop controller, not calc_steering_from_turn_rate().
            // wp_nav's own turn-rate solution is reused as the rate ceiling/feed-forward
            // so cornering aggressiveness stays close to stock behaviour.
            calc_steering_to_heading(desired_heading_cd, fabsf(degrees(g2.wp_nav.get_turn_rate_rads())));
        }
    } else {
        // DRIFT_GAIN_NAV == 0: exact original stock path, unmodified by any of this
        // session's work, for clean on-water A/B isolation testing
        float desired_turn_rate_rads = g2.wp_nav.get_turn_rate_rads();
#if AP_AVOIDANCE_ENABLED
        if (g2.avoid.limits_active() && (fabsf(attitude_control.get_desired_speed()) <= attitude_control.get_stop_speed())) {
            desired_turn_rate_rads = 0.0f;
        }
#endif
        calc_steering_from_turn_rate(desired_turn_rate_rads);
    }
}

// calculate steering output given a turn rate
// desired turn rate in radians/sec. Positive to the right.
void Mode::calc_steering_from_turn_rate(float turn_rate)
{
    // calculate and send final steering command to motor library
    const float steering_out = attitude_control.get_steering_out_rate(turn_rate,
                                                                      g2.motors.limit.steer_left,
                                                                      g2.motors.limit.steer_right,
                                                                      rover.G_Dt);
    set_steering(steering_out * 4500.0f);
}

/*
    calculate steering output given lateral_acceleration
*/
void Mode::calc_steering_from_lateral_acceleration(float lat_accel, bool reversed)
{
    // constrain to max G force
    lat_accel = constrain_float(lat_accel, -attitude_control.get_turn_lat_accel_max(), attitude_control.get_turn_lat_accel_max());

    // send final steering command to motor library
    const float steering_out = attitude_control.get_steering_out_lat_accel(lat_accel,
                                                                           g2.motors.limit.steer_left,
                                                                           g2.motors.limit.steer_right,
                                                                           rover.G_Dt);
    set_steering(steering_out * 4500.0f);
}

// calculate steering output to drive towards desired heading
// rate_max is a maximum turn rate in deg/s.  set to zero to use default turn rate limits
void Mode::calc_steering_to_heading(float desired_heading_cd, float rate_max_degs)  
{  
    // record the heading target and timestamp so calc_throttle() can compute a  
    // trustworthy yaw error for the vectored-thrust steering floor. writing  
    // _desired_yaw_cd here (not only in mode update() bodies) keeps it correct  
    // for callers that use a local heading target (e.g. ModeSimple).  
    _desired_yaw_cd = desired_heading_cd;  
    _steering_heading_active_ms = AP_HAL::millis();  
  
    // call heading controller  
    const float steering_out = attitude_control.get_steering_out_heading(radians(desired_heading_cd*0.01f),
                                                                         radians(rate_max_degs),
                                                                         g2.motors.limit.steer_left,
                                                                         g2.motors.limit.steer_right,
                                                                         rover.G_Dt);
    set_steering(steering_out * 4500.0f);
}

void Mode::set_steering(float steering_value)
{
    if (allows_stick_mixing() && g2.stick_mixing > 0) {
        steering_value = channel_steer->stick_mixing((int16_t)steering_value);
    }
    g2.motors.set_steering(steering_value);
}

Mode *Rover::mode_from_mode_num(const enum Mode::Number num)
{
    Mode *ret = nullptr;
    switch (num) {
    case Mode::Number::MANUAL:
        ret = &mode_manual;
        break;
    case Mode::Number::ACRO:
        ret = &mode_acro;
        break;
    case Mode::Number::STEERING:
        ret = &mode_steering;
        break;
    case Mode::Number::HOLD:
        ret = &mode_hold;
        break;
    case Mode::Number::LOITER:
        ret = &mode_loiter;
        break;
#if MODE_FOLLOW_ENABLED
    case Mode::Number::FOLLOW:
        ret = &mode_follow;
        break;
#endif
    case Mode::Number::SIMPLE:
        ret = &mode_simple;
        break;
    case Mode::Number::CIRCLE:
        ret = &g2.mode_circle;
        break;
    case Mode::Number::AUTO:
        ret = &mode_auto;
        break;
    case Mode::Number::RTL:
        ret = &mode_rtl;
        break;
    case Mode::Number::SMART_RTL:
        ret = &mode_smartrtl;
        break;
    case Mode::Number::GUIDED:
        ret = &mode_guided;
        break;
    case Mode::Number::INITIALISING:
        ret = &mode_initializing;
        break;
#if MODE_DOCK_ENABLED
    case Mode::Number::DOCK:
        ret = (Mode *)g2.mode_dock_ptr;
        break;
#endif
    default:
        break;
    }
    return ret;
}