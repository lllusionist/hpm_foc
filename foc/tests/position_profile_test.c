#include "position_profile.h"
#include "util.h"
#include <assert.h>

#define TEST_PERIOD (1.0f / 5000.0f)
#define MAX_MOVE_STEPS 25000U
#define POSITION_TOLERANCE 0.00001f
#define VELOCITY_TOLERANCE 0.00002f
#define ACCELERATION_TOLERANCE 0.0005f

static const position_profile_limits_t asymmetric_limits = {
	.velocity = 2.0f,
	.acceleration = 4.0f,
	.deceleration = 3.0f,
	.jerk = 40.0f,
};

static void step_checked(position_profile_t *profile)
{
	position_profile_sample_t before = profile->reference;
	position_profile_sample_t after;
	const position_profile_limits_t *limits = &profile->limits;
	float max_accel = MAX(limits->acceleration, limits->deceleration);
	float position_delta, velocity_delta, accel_bound;

	position_profile_step(profile, TEST_PERIOD);
	after = profile->reference;
	position_delta = position_profile_wrap_angle(
	    after.position - before.position);
	velocity_delta = after.velocity - before.velocity;
	accel_bound = (after.velocity * after.acceleration >= 0.0f) ?
		      limits->acceleration : limits->deceleration;

	assert(fabsf(after.velocity) <= limits->velocity + VELOCITY_TOLERANCE);
	assert(fabsf(after.acceleration) <=
	       accel_bound + ACCELERATION_TOLERANCE);
	assert(fabsf(after.acceleration - before.acceleration) <=
	       limits->jerk * TEST_PERIOD + ACCELERATION_TOLERANCE);
	assert(fabsf(velocity_delta - before.acceleration * TEST_PERIOD) <=
	       0.5f * limits->jerk * SQ(TEST_PERIOD) + VELOCITY_TOLERANCE);
	assert(fabsf(position_delta - before.velocity * TEST_PERIOD) <=
	       0.5f * max_accel * SQ(TEST_PERIOD) + POSITION_TOLERANCE);
}

static void finish_at(position_profile_t *profile, float target)
{
	unsigned int i;
	float error;

	for (i = 0U; i < MAX_MOVE_STEPS &&
	     position_profile_active(profile); i++)
		step_checked(profile);

	assert(!position_profile_active(profile));
	error = position_profile_wrap_angle(
	    target - profile->reference.position);
	assert(fabsf(error) < POSITION_TOLERANCE);
	assert(profile->reference.velocity == 0.0f);
	assert(profile->reference.acceleration == 0.0f);
}

static void test_short_move(void)
{
	const position_profile_limits_t limits = {
		.velocity = 40.0f * M_2PI,
		.acceleration = 50.0f * M_2PI,
		.deceleration = 50.0f * M_2PI,
		.jerk = 1000.0f * M_2PI,
	};
	const float target = 0.0001f;
	position_profile_t profile;

	position_profile_reset(&profile, 0.0f, limits);
	position_profile_set_target(&profile, target);
	assert(position_profile_active(&profile));
	assert(profile.phases[1].duration == 0.0f);
	assert(profile.phases[5].duration == 0.0f);
	finish_at(&profile, target);
	position_profile_set_target(&profile, target);
	assert(!position_profile_active(&profile));
}

static void test_shortest_path(void)
{
	position_profile_t profile;
	float start = M_2PI - 0.05f;
	const float target = 0.05f;
	unsigned int i;

	position_profile_reset(&profile, start, asymmetric_limits);
	position_profile_set_target(&profile, target);
	for (i = 0U; i < MAX_MOVE_STEPS &&
	     position_profile_active(&profile); i++) {
		step_checked(&profile);
		assert(profile.reference.velocity >= -VELOCITY_TOLERANCE);
	}
	finish_at(&profile, target);
	assert(fabsf(profile.reference.position - start - 0.1f) <
	       POSITION_TOLERANCE);

	position_profile_reset(&profile, -M_PI + 0.05f, asymmetric_limits);
	position_profile_set_target(&profile, M_PI - 0.05f);
	for (i = 0U; i < MAX_MOVE_STEPS &&
	     position_profile_active(&profile); i++) {
		step_checked(&profile);
		assert(profile.reference.velocity <= VELOCITY_TOLERANCE);
	}
	finish_at(&profile, M_PI - 0.05f);
}

static void test_retarget(float direction, unsigned int phase)
{
	position_profile_t profile;
	position_profile_sample_t before;
	unsigned int i;
	float latest = -direction * 0.3f;
	float first_target = direction * 2.7f;
	float elapsed;

	position_profile_reset(&profile, 0.0f, asymmetric_limits);
	position_profile_set_target(&profile, first_target);
	for (i = 0U; i < POSITION_PROFILE_PHASES; i++)
		assert(profile.phases[i].duration > 0.0f);

	for (i = 0U; i < phase; i++) {
		while (profile.phase == i)
			step_checked(&profile);
	}
	while (profile.elapsed < 0.5f * profile.phases[phase].duration)
		step_checked(&profile);
	assert(profile.phase == phase);

	before = profile.reference;
	position_profile_set_target(&profile, first_target);
	assert(profile.phase == phase);
	position_profile_set_target(&profile, -direction * 0.2f);
	assert(profile.braking);
	assert(profile.reference.position == before.position);
	assert(profile.reference.velocity == before.velocity);
	assert(profile.reference.acceleration == before.acceleration);
	step_checked(&profile);
	elapsed = profile.elapsed;
	position_profile_set_target(&profile, latest);
	assert(profile.braking && profile.elapsed == elapsed);
	finish_at(&profile, latest);
}

static void test_invalid_input(void)
{
	position_profile_t profile;

	position_profile_reset(&profile, 1.0f, asymmetric_limits);
	position_profile_set_target(&profile, NAN);
	assert(profile.target == 1.0f);
	assert(!position_profile_active(&profile));
	position_profile_reset(&profile, 1.0f, (position_profile_limits_t){0});
	position_profile_set_target(&profile, 2.0f);
	assert(!position_profile_active(&profile));
	assert(profile.reference.position == 1.0f);
}

int main(void)
{
	unsigned int phase;

	test_short_move();
	test_shortest_path();
	for (phase = 0U; phase < POSITION_PROFILE_PHASES; phase++) {
		test_retarget(1.0f, phase);
		test_retarget(-1.0f, phase);
	}
	test_invalid_input();
	return 0;
}
