#ifndef POSITION_PROFILE_H
#define POSITION_PROFILE_H

#include <stdbool.h>

#define POSITION_PROFILE_PHASES 7U

typedef struct {
	float position;
	float velocity;
	float acceleration;
} position_profile_sample_t;

typedef struct {
	float velocity;
	float acceleration;
	float deceleration;
	float jerk;
} position_profile_limits_t;

typedef struct {
	float duration;
	float jerk;
} position_profile_phase_t;

typedef struct {
	position_profile_sample_t reference;
	position_profile_sample_t phase_start;
	position_profile_limits_t limits;
	position_profile_phase_t phases[POSITION_PROFILE_PHASES];
	float target;
	float end_position;
	float elapsed;
	unsigned int phase;
	unsigned int phase_count;
	bool braking;
} position_profile_t;

void position_profile_reset(position_profile_t *profile, float angle,
			    position_profile_limits_t limits);
void position_profile_set_target(position_profile_t *profile, float angle);
void position_profile_step(position_profile_t *profile, float dt);
bool position_profile_active(const position_profile_t *profile);
float position_profile_wrap_angle(float angle);

#endif
