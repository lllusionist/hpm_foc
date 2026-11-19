#include "position_profile.h"
#include "util.h"

#define POSITION_PROFILE_SEARCH_STEPS 24U
#define POSITION_PROFILE_BRAKE_PHASES 3U

typedef struct {
	float ramp;
	float hold;
} ramp_times_t;

float position_profile_wrap_angle(float angle)
{
	while (angle > M_PI)
		angle -= M_2PI;
	while (angle < -M_PI)
		angle += M_2PI;
	return angle;
}

static position_profile_sample_t sample_at(position_profile_sample_t start,
					   float jerk, float t)
{
	float accel_delta = jerk * t;

	return (position_profile_sample_t){
		.position = start.position + t * (start.velocity +
			t * (0.5f * start.acceleration + accel_delta / 6.0f)),
		.velocity = start.velocity +
			t * (start.acceleration + 0.5f * accel_delta),
		.acceleration = start.acceleration + accel_delta,
	};
}

static ramp_times_t ramp_times(float ramp, float max_ramp)
{
	if (ramp <= max_ramp)
		return (ramp_times_t){ramp, 0.0f};
	return (ramp_times_t){max_ramp,
			      (ramp - max_ramp) * (ramp / max_ramp + 1.0f)};
}

static float distance_at_ramp(float ramp,
			      const position_profile_limits_t *limits)
{
	float velocity = limits->jerk * ramp * ramp;
	ramp_times_t up = ramp_times(ramp, limits->acceleration / limits->jerk);
	ramp_times_t down = ramp_times(ramp,
				       limits->deceleration / limits->jerk);

	return velocity * (up.ramp + down.ramp + 0.5f * (up.hold + down.hold));
}

static float peak_ramp(float distance,
		     const position_profile_limits_t *limits)
{
	float low = 0.0f;
	float high = sqrtf(limits->velocity / limits->jerk);
	unsigned int i;

	if (distance_at_ramp(high, limits) <= distance)
		return high;

	for (i = 0U; i < POSITION_PROFILE_SEARCH_STEPS; i++) {
		float mid = 0.5f * (low + high);

		if (distance_at_ramp(mid, limits) > distance)
			high = mid;
		else
			low = mid;
	}
	return low;
}

static void start_phases(position_profile_t *profile, unsigned int count,
			 bool braking)
{
	profile->phase_start = profile->reference;
	profile->elapsed = 0.0f;
	profile->phase = 0U;
	profile->phase_count = count;
	profile->braking = braking;
}

static void plan_move(position_profile_t *profile)
{
	const position_profile_limits_t *limits = &profile->limits;
	float position = profile->reference.position;
	float delta = position_profile_wrap_angle(profile->target - position);
	float distance = fabsf(delta);
	float sum = limits->velocity + limits->acceleration +
		    limits->deceleration + limits->jerk;
	float ramp, velocity, jerk, cruise_distance, cruise;
	ramp_times_t up, down;

	if (distance == 0.0f || !(limits->velocity > 0.0f) ||
	    !(limits->acceleration > 0.0f) || !(limits->deceleration > 0.0f) ||
	    !(limits->jerk > 0.0f) || !isfinite(sum))
		return;

	ramp = peak_ramp(distance, limits);
	velocity = limits->jerk * ramp * ramp;
	if (velocity == 0.0f)
		return;

	up = ramp_times(ramp, limits->acceleration / limits->jerk);
	down = ramp_times(ramp, limits->deceleration / limits->jerk);
	cruise_distance = distance - distance_at_ramp(ramp, limits);
	cruise = MAX(0.0f, cruise_distance) / velocity;
	jerk = copysignf(limits->jerk, delta);

	if (fabsf(position) > M_2PI)
		position = position_profile_wrap_angle(position);
	profile->reference.position = position;
	profile->end_position = position + delta;
	profile->phases[0] = (position_profile_phase_t){up.ramp, jerk};
	profile->phases[1] = (position_profile_phase_t){up.hold, 0.0f};
	profile->phases[2] = (position_profile_phase_t){up.ramp, -jerk};
	profile->phases[3] = (position_profile_phase_t){cruise, 0.0f};
	profile->phases[4] = (position_profile_phase_t){down.ramp, -jerk};
	profile->phases[5] = (position_profile_phase_t){down.hold, 0.0f};
	profile->phases[6] = (position_profile_phase_t){down.ramp, jerk};
	start_phases(profile, POSITION_PROFILE_PHASES, false);
}

static void plan_brake(position_profile_t *profile)
{
	const position_profile_sample_t *ref = &profile->reference;
	float speed = fabsf(ref->velocity);
	float direction = copysignf(1.0f, speed > 0.0f ? ref->velocity :
						       ref->acceleration);
	float accel = direction * ref->acceleration;
	float jerk = profile->limits.jerk;
	float peak = sqrtf(speed * jerk + 0.5f * accel * accel);
	float hold = 0.0f;
	float ramp_down;

	if (peak > profile->limits.deceleration) {
		float ramp_speed_change;

		peak = profile->limits.deceleration;
		ramp_speed_change = (peak * peak - 0.5f * accel * accel) / jerk;
		hold = (speed - ramp_speed_change) / peak;
	}
	ramp_down = MAX(0.0f, (accel + peak) / jerk);
	profile->phases[0] = (position_profile_phase_t){ramp_down,
						      -direction * jerk};
	profile->phases[1] = (position_profile_phase_t){MAX(0.0f, hold), 0.0f};
	profile->phases[2] = (position_profile_phase_t){peak / jerk,
						      direction * jerk};
	start_phases(profile, POSITION_PROFILE_BRAKE_PHASES, true);
}

void position_profile_reset(position_profile_t *profile, float angle,
			    position_profile_limits_t limits)
{
	*profile = (position_profile_t){
		.reference.position = angle,
		.target = angle,
		.limits = limits,
	};
}

bool position_profile_active(const position_profile_t *profile)
{
	return profile->phase < profile->phase_count;
}

void position_profile_set_target(position_profile_t *profile, float angle)
{
	if (angle == profile->target || !isfinite(angle) ||
	    fabsf(angle) > M_2PI ||
	    position_profile_wrap_angle(angle - profile->target) == 0.0f)
		return;

	profile->target = angle;
	if (!position_profile_active(profile))
		plan_move(profile);
	else if (!profile->braking)
		plan_brake(profile);
}

void position_profile_step(position_profile_t *profile, float dt)
{
	while (position_profile_active(profile)) {
		const position_profile_phase_t *phase =
		    &profile->phases[profile->phase];
		float remaining = phase->duration - profile->elapsed;

		if (dt < remaining) {
			float elapsed = profile->elapsed + dt;

			profile->elapsed = elapsed;
			profile->reference = sample_at(
			    profile->phase_start, phase->jerk, elapsed);
			return;
		}

		dt -= remaining;
		profile->reference = sample_at(
		    profile->phase_start, phase->jerk, phase->duration);
		profile->phase_start = profile->reference;
		profile->elapsed = 0.0f;
		profile->phase++;

		if (position_profile_active(profile))
			continue;

		profile->reference.velocity = 0.0f;
		profile->reference.acceleration = 0.0f;
		if (profile->braking) {
			profile->braking = false;
			plan_move(profile);
		} else {
			profile->reference.position = profile->end_position;
		}
	}
}
