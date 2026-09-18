#include "Rover.h"

bool ModeLoiter::_enter()
{
    // set _destination to reasonable stopping point
    if (!g2.wp_nav.get_stopping_location(_destination)) {
        return false;
    }

    // initialise desired speed to current speed
    if (!attitude_control.get_forward_speed(_desired_speed)) {
        _desired_speed = 0.0f;
    }

    // initialise heading to current heading
    _desired_yaw_cd = ahrs.yaw_sensor;

    // reset drift-estimate detection state so stale data from a previous
    // loiter session doesn't leak into this one.  Compute distance fresh
    // against the just-set _destination rather than reusing whatever the
    // previous mode last left in _distance_to_destination.
    _drift_last_distance = rover.current_loc.get_distance(_destination);
    _drift_rising_count = 0;

    return true;
}

void ModeLoiter::update()
{
    // get distance (in meters) to destination
    _distance_to_destination = rover.current_loc.get_distance(_destination);

    const float loiter_radius = g2.sailboat.tack_enabled() ? g2.sailboat.get_loiter_radius() : g2.loit_radius;

    // if within loiter radius slew desired speed towards zero and use existing desired heading
    if (_distance_to_destination <= loiter_radius) {
        // sailboats should not stop unless motoring
        const float desired_speed_within_radius = g2.sailboat.tack_enabled() ? 0.1f : 0.0f;
        _desired_speed = attitude_control.get_desired_speed_accel_limited(desired_speed_within_radius, rover.G_Dt);

        // if we have a sail but not trying to use it then point into the wind
        if (!g2.sailboat.tack_enabled() && g2.sailboat.sail_enabled()) {
            _desired_yaw_cd = degrees(g2.windvane.get_true_wind_direction_rad()) * 100.0f;
        }

        // current/wind drift-estimate sampling: only trust a sample once distance-to-
        // destination has been consistently rising (i.e. the vehicle is being pushed
        // outward, not still decelerating from its last approach) AND throttle output
        // is effectively zero (pure coast, no thrust contribution to the motion)
        if (_distance_to_destination > _drift_last_distance) {
            _drift_rising_count++;
        } else {
            _drift_rising_count = 0;
        }
        _drift_last_distance = _distance_to_destination;

        if (_drift_rising_count >= LOITER_DRIFT_RISING_TICKS &&
            fabsf(g2.motors.get_throttle()) < LOITER_DRIFT_THR_PCT) {
            Vector3f vel_ned;
            if (ahrs.get_velocity_NED(vel_ned)) {
                g2.motors.set_current_estimate_ne(Vector2f{vel_ned.x, vel_ned.y});
            }
        }
    } else {
        _drift_rising_count = 0;
        // P controller with hard-coded gain to convert distance to desired speed
        _desired_speed = MIN((_distance_to_destination - loiter_radius) * g2.loiter_speed_gain, g2.wp_nav.get_default_speed());

        // calculate bearing to destination
        _desired_yaw_cd = rover.current_loc.get_bearing_to(_destination);
        float yaw_error_cd = wrap_180_cd(_desired_yaw_cd - ahrs.yaw_sensor);
        // if destination is behind vehicle, reverse towards it
        if ((fabsf(yaw_error_cd) > 9000 && g2.loit_type == 0) || g2.loit_type == 2) {
            _desired_yaw_cd = wrap_180_cd(_desired_yaw_cd + 18000);
            yaw_error_cd = wrap_180_cd(_desired_yaw_cd - ahrs.yaw_sensor);
            _desired_speed = -_desired_speed;
        }

        // compensate for current/wind drift while driving back to the loiter center
        apply_drift_compensation(_desired_yaw_cd, _desired_speed);
        yaw_error_cd = wrap_180_cd(_desired_yaw_cd - ahrs.yaw_sensor);

        // reduce desired speed if yaw_error is large
        // 45deg of error reduces speed to 75%, 90deg of error reduces speed to 50%
        float yaw_error_ratio = 1.0f - constrain_float(fabsf(yaw_error_cd / 9000.0f), 0.0f, 1.0f) * 0.5f;
        _desired_speed *= yaw_error_ratio;
    }

    // 0 turn rate is no limit
    float turn_rate = 0.0;

    // make sure sailboats don't try and sail directly into the wind
    if (g2.sailboat.use_indirect_route(_desired_yaw_cd)) {
        _desired_yaw_cd = g2.sailboat.calc_heading(_desired_yaw_cd);
        if (g2.sailboat.tacking()) {
            // use pivot turn rate for tacks
            turn_rate = g2.wp_nav.get_pivot_rate();
        }
    }

    // run steering and throttle controllers
    calc_steering_to_heading(_desired_yaw_cd, turn_rate);
    calc_throttle(_desired_speed, true);
}

// get desired location
bool ModeLoiter::get_desired_location(Location& destination) const
{
    destination = _destination;
    return true;
}
