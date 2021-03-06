#include "survive_imu.h"
#include "linmath.h"
#include "math.h"
#include "survive_imu.h"
#include "survive_internal.h"
#include <assert.h>
#include <memory.h>

// Mahoney is due to https://hal.archives-ouvertes.fr/hal-00488376/document
// See also http://www.olliw.eu/2013/imu-data-fusing/#chapter41 and
// http://x-io.co.uk/open-source-imu-and-ahrs-algorithms/
static void mahony_ahrs(SurviveIMUTracker *tracker, LinmathQuat q, const LinmathVec3d _gyro,
						const LinmathVec3d _accel) {
	LinmathVec3d gyro;
	memcpy(gyro, _gyro, 3 * sizeof(FLT));

	LinmathVec3d accel;
	memcpy(accel, _accel, 3 * sizeof(FLT));

	const FLT sample_f = tracker->so->imu_freq;
	const FLT prop_gain = .5;
	const FLT int_gain = 0;

	FLT mag_accel = magnitude3d(accel);

	if (mag_accel != 0.0) {
		scale3d(accel, accel, 1. / mag_accel);

		// Equiv of q^-1 * G
		LinmathVec3d v = {q[1] * q[3] - q[0] * q[2], q[0] * q[1] + q[2] * q[3], q[0] * q[0] - 0.5 + q[3] * q[3]};

		LinmathVec3d error;
		cross3d(error, accel, v);

		if (int_gain > 0.0f) {
			LinmathVec3d fb_correction;
			scale3d(fb_correction, error, int_gain * 2. / sample_f);
			add3d(tracker->integralFB, tracker->integralFB, fb_correction);
			add3d(gyro, gyro, tracker->integralFB);
		}

		scale3d(error, error, prop_gain * 2.);
		add3d(gyro, gyro, error);
	}

	scale3d(gyro, gyro, 0.5 / sample_f);

	LinmathQuat correction = {
		(-q[1] * gyro[0] - q[2] * gyro[1] - q[3] * gyro[2]), (+q[0] * gyro[0] + q[2] * gyro[2] - q[3] * gyro[1]),
		(+q[0] * gyro[1] - q[1] * gyro[2] + q[3] * gyro[0]), (+q[0] * gyro[2] + q[1] * gyro[1] - q[2] * gyro[0])};

	quatadd(q, q, correction);
	quatnormalize(q, q);
}

static const int imu_calibration_iterations = 100;

static void RotateAccel(LinmathVec3d rAcc, const LinmathQuat rot, const LinmathVec3d accel) {
	quatrotatevector(rAcc, rot, accel);
	LinmathVec3d G = {0, 0, -1};
	add3d(rAcc, rAcc, G);
	scale3d(rAcc, rAcc, 9.8066);
	FLT m = magnitude3d(rAcc);
}

static inline FLT survive_update_kalman_variance(const SurviveIMUTracker *tracker, struct kalman_info_t *info,
												 survive_timecode timecode, FLT new_variance) {
	FLT time_diff = survive_timecode_difference(timecode, info->last_update) / (FLT)tracker->so->timebase_hz;
	FLT variance = info->variance + info->variance_per_second * time_diff;
	FLT combined_variance = new_variance + variance;
	info->last_update = timecode;

	FLT incoming_weight = combined_variance == 0 ? 1. : variance / combined_variance;

	FLT huh = incoming_weight * variance;
	info->variance = (1. - incoming_weight) * variance;

	return incoming_weight;
}

static inline void survive_update_position(SurviveIMUTracker *tracker, struct kalman_info_position_t *pos,
										   survive_timecode timecode, FLT new_variance,
										   const LinmathVec3d new_position) {
	pos->info.update_fn(tracker, timecode, &pos->info);

	FLT incoming_pose_weight = survive_update_kalman_variance(tracker, &pos->info, timecode, new_variance);
	for (int i = 0; i < 3; i++) {
		pos->v[i] += incoming_pose_weight * (new_position[i] - pos->v[i]);
		assert(!isnan(pos->v[i]));
	}
	// struct SurviveContext* ctx = tracker->so->ctx;
	// SV_INFO("PW: %f %f %f", incoming_pose_weight, norm3d(pos->v), norm3d(new_position));
}

static inline void survive_update_rotation(SurviveIMUTracker *tracker, struct kalman_info_rotation_t *rot,
										   survive_timecode timecode, FLT new_variance, const LinmathQuat new_rot) {
	rot->info.update_fn(tracker, timecode, &rot->info);

	FLT incoming_pose_weight = survive_update_kalman_variance(tracker, &rot->info, timecode, new_variance);
	quatslerp(rot->v, rot->v, new_rot, incoming_pose_weight);
}

static inline void survive_update_pose(SurviveIMUTracker *tracker, struct kalman_info_pose_t *pose,
									   survive_timecode timecode, const FLT *new_variance,
									   const LinmathPose *new_pose) {
	survive_update_position(tracker, &pose->Pos, timecode, new_variance[0], new_pose->Pos);
	survive_update_rotation(tracker, &pose->Rot, timecode, new_variance[1], new_pose->Rot);
}

void survive_imu_tracker_integrate_rotation(SurviveIMUTracker *tracker, survive_timecode timecode,
											const LinmathQuat Rot, FLT R) {
	survive_update_rotation(tracker, &tracker->pose.Rot, timecode, R, Rot);
}

void survive_update_variances(SurviveIMUTracker *tracker, uint32_t timecode) {
	if (quatiszero(tracker->last_pose.Rot.v))
		return;

	FLT time_diff =
		survive_timecode_difference(timecode, tracker->last_pose.Pos.info.last_update) / (FLT)tracker->so->timebase_hz;
	assert(time_diff < 1.0);
	FLT var_meters = 1;
	FLT var_quat = 11111;

	tracker->velocity.Pos.info.variance += var_meters * time_diff;
	tracker->velocity.Rot.info.variance += var_quat * time_diff;

	tracker->pose.Pos.info.variance += tracker->velocity.Pos.info.variance * time_diff;
	tracker->pose.Rot.info.variance += tracker->velocity.Rot.info.variance * time_diff;
}

void survive_imu_tracker_integrate_angular_velocity(SurviveIMUTracker *tracker, survive_timecode timecode,
													const LinmathQuat Rot, FLT R) {
	survive_update_rotation(tracker, &tracker->velocity.Rot, timecode, R, Rot);
}

void survive_imu_tracker_integrate_velocity(SurviveIMUTracker *tracker, survive_timecode timecode, const FLT *Rv,
											const SurvivePose *pose) {
	survive_update_pose(tracker, &tracker->velocity, timecode, Rv, pose);
}

static inline void update_pose_pos(SurviveIMUTracker *tracker, survive_timecode timecode, struct kalman_info_t *_info) {
	struct kalman_info_position_t *info = (struct kalman_info_position_t *)_info;
	_info->variance = survive_imu_tracker_predict_pos(tracker, timecode, info->v);
	_info->last_update = timecode;
}
static inline void update_pose_rot(SurviveIMUTracker *tracker, survive_timecode timecode, struct kalman_info_t *_info) {
	struct kalman_info_rotation_t *info = (struct kalman_info_rotation_t *)_info;
	_info->variance = survive_imu_tracker_predict_rot(tracker, timecode, info->v);
	_info->last_update = timecode;
}

static inline void update_vel_pos(SurviveIMUTracker *tracker, survive_timecode timecode, struct kalman_info_t *_info) {
	struct kalman_info_position_t *info = (struct kalman_info_position_t *)_info;
	_info->variance = survive_imu_tracker_predict_velocity_pos(tracker, timecode, info->v);
}
static inline void update_vel_rot(SurviveIMUTracker *tracker, survive_timecode timecode, struct kalman_info_t *_info) {
	struct kalman_info_rotation_t *info = (struct kalman_info_rotation_t *)_info;
	_info->variance = survive_imu_tracker_predict_velocity_rot(tracker, timecode, info->v);
}

void survive_imu_tracker_integrate_imu(SurviveIMUTracker *tracker, PoserDataIMU *data) {
	if (!tracker->is_initialized) {
		if (tracker->last_data.datamask == imu_calibration_iterations) {
			tracker->last_data = *data;

			const FLT up[3] = {0, 0, 1};
			quatfrom2vectors(tracker->pose.Rot.v, tracker->updir, up);
			tracker->accel_scale_bias = 1. / magnitude3d(tracker->updir);
			tracker->is_initialized = true;

			return;
		}

		tracker->last_data.datamask++;

		tracker->updir[0] += data->accel[0] / imu_calibration_iterations;
		tracker->updir[1] += data->accel[1] / imu_calibration_iterations;
		tracker->updir[2] += data->accel[2] / imu_calibration_iterations;
		return;
	}

	// survive_update_variances(tracker, data->timecode);

	for (int i = 0; i < 3; i++) {
		tracker->updir[i] = data->accel[i] * .10 + tracker->updir[i] * .90;
	}

	LinmathQuat pose_rot;
	quatcopy(pose_rot, tracker->pose.Rot.v);
	mahony_ahrs(tracker, pose_rot, data->gyro, data->accel);
	survive_imu_tracker_integrate_rotation(tracker, data->timecode, pose_rot, 1e-1);

	LinmathQuat rotVelInIMU;
	quatfromeuler(rotVelInIMU, data->gyro);
	LinmathPose new_velocity;
	quatconjugateby(new_velocity.Rot, tracker->pose.Rot.v, rotVelInIMU);

	FLT Rv[2] = {tracker->pose.Rot.info.variance + tracker->velocity.Pos.info.variance + tracker->acc_var,
				 tracker->pose.Rot.info.variance + tracker->gyro_var};

	FLT time_diff =
		survive_timecode_difference(data->timecode, tracker->last_data.timecode) / (FLT)tracker->so->timebase_hz;

	if (!isinf(Rv[0])) {
		LinmathVec3d acc;
		scale3d(acc, data->accel, tracker->accel_scale_bias);

		LinmathVec3d rAcc = {0}, avgAcc;
		RotateAccel(rAcc, tracker->pose.Rot.v, acc);

		add3d(avgAcc, rAcc, tracker->last_acc);
		// scale3d(avgAcc, avgAcc, .5 * time_diff);
		scale3d(avgAcc, rAcc, time_diff);
		add3d(new_velocity.Pos, tracker->velocity.Pos.v, avgAcc);
		copy3d(tracker->last_acc, rAcc);

		survive_imu_tracker_integrate_velocity(tracker, data->timecode, Rv, &new_velocity);
	} else {
		survive_imu_tracker_integrate_angular_velocity(tracker, data->timecode, new_velocity.Rot, Rv[1]);
	}

	SurviveContext *ctx = tracker->so->ctx;
	// SV_INFO("IMU VAR %f", tracker->pose.Pos.info.variance);

	tracker->last_data = *data;
}

FLT survive_imu_tracker_predict_velocity_pos(const SurviveIMUTracker *tracker, survive_timecode timecode, double *out) {
	FLT time_diff =
		survive_timecode_difference(timecode, tracker->velocity.Pos.info.last_update) / (FLT)tracker->so->timebase_hz;
	// scale3d(out, tracker->last_acc, time_diff);
	// add3d(out, out, tracker->velocity.Pos.v);
	copy3d(out, tracker->velocity.Pos.v);
	return tracker->velocity.Pos.info.variance + time_diff * tracker->velocity.Pos.info.variance_per_second;
}

FLT survive_imu_tracker_predict_velocity_rot(const SurviveIMUTracker *tracker, survive_timecode timecode, double *out) {
	FLT time_diff =
		survive_timecode_difference(timecode, tracker->velocity.Rot.info.last_update) / (FLT)tracker->so->timebase_hz;
	quatcopy(out, tracker->velocity.Rot.v);
	return tracker->velocity.Rot.info.variance + time_diff * tracker->velocity.Rot.info.variance_per_second;
}

FLT survive_imu_tracker_predict_pos(const SurviveIMUTracker *tracker, survive_timecode timecode, LinmathVec3d out) {
	FLT pose_time_diff =
		survive_timecode_difference(timecode, tracker->pose.Pos.info.last_update) / (FLT)tracker->so->timebase_hz;
	// assert(pose_time_diff < 1.0);

	LinmathVec3d vel, displacement;
	FLT velocity_variance = survive_imu_tracker_predict_velocity_pos(tracker, timecode, vel);
	scale3d(displacement, vel, pose_time_diff);
	add3d(out, displacement, tracker->pose.Pos.v);
	assert(norm3d(out) < 1000);
	return tracker->pose.Pos.info.variance +
		   pose_time_diff * (velocity_variance + tracker->pose.Pos.info.variance_per_second);
}

FLT survive_imu_tracker_predict_rot(const SurviveIMUTracker *tracker, survive_timecode timecode, LinmathQuat out) {
	if (quatiszero(tracker->pose.Rot.v))
		return tracker->pose.Rot.info.variance;

	FLT rot_time_diff =
		survive_timecode_difference(timecode, tracker->pose.Rot.info.last_update) / (FLT)tracker->so->timebase_hz;
	// assert(rot_time_diff < 1.0);

	LinmathQuat vel;
	FLT velocity_variance = survive_imu_tracker_predict_velocity_rot(tracker, timecode, vel);

	LinmathQuat rot_change;
	quatmultiplyrotation(rot_change, vel, rot_time_diff);
	quatrotateabout(out, rot_change, tracker->pose.Rot.v);

	return tracker->pose.Rot.info.variance +
		   rot_time_diff * (velocity_variance + tracker->pose.Rot.info.variance_per_second);
}
void survive_imu_tracker_predict(const SurviveIMUTracker *tracker, survive_timecode timecode, SurvivePose *out) {
	survive_imu_tracker_predict_pos(tracker, timecode, out->Pos);
	survive_imu_tracker_predict_rot(tracker, timecode, out->Rot);
}

void survive_imu_tracker_integrate_observation(uint32_t timecode, SurviveIMUTracker *tracker, const SurvivePose *pose,
											   const FLT *R) {
	// Kalman filter assuming:
	// F -> Identity
	// H -> Identity
	// Q / R / P -> Diagonal matrices; just treat them as such. This assumption might need some checking but it
	// makes the # of calculations needed much smaller so we may be willing to tolerate some approximation here

	survive_update_pose(tracker, &tracker->pose, timecode, R, pose);

	FLT time_diff =
		survive_timecode_difference(timecode, tracker->last_pose.Pos.info.last_update) / (FLT)tracker->so->timebase_hz;

	if (!quatiszero(tracker->last_pose.Rot.v) && time_diff != 0.) {
		SurvivePose velocity;
		quatfind(velocity.Rot, tracker->last_pose.Rot.v, tracker->pose.Rot.v);
		quatmultiplyrotation(velocity.Rot, velocity.Rot, 1. / time_diff);

		sub3d(velocity.Pos, tracker->pose.Pos.v, tracker->last_pose.Pos.v);
		scale3d(velocity.Pos, velocity.Pos, 1. / time_diff);

		SurvivePoseVariance vp = {.Pose = tracker->pose.Pos.info.variance + tracker->last_pose.Pos.info.variance,
								  .Rot = tracker->pose.Rot.info.variance + tracker->last_pose.Rot.info.variance};
		survive_imu_tracker_integrate_velocity(tracker, timecode, &vp.Pose, &velocity);
	}

	tracker->last_pose = tracker->pose;
}

STATIC_CONFIG_ITEM(POSE_POSITION_VARIANCE_SEC, "filter-pose-var-per-sec", 'f', "Position variance per second", 0.0001);
STATIC_CONFIG_ITEM(VELOCITY_POSITION_VARIANCE_SEC, "filter-vel-var-per-sec", 'f', "Velocity variance per second", 1.);

STATIC_CONFIG_ITEM(IMU_ACC_VARIANCE, "imu-acc-variance", 'f', "Variance of accelerometer", 0.1);
STATIC_CONFIG_ITEM(IMU_GYRO_VARIANCE, "imu-gyro-variance", 'f', "Variance of gyroscope", 0.1);

void survive_imu_tracker_init(SurviveIMUTracker *tracker, SurviveObject *so) {
	memset(tracker, 0, sizeof(*tracker));
	tracker->velocity.Rot.v[0] = tracker->pose.Rot.v[0] = 1.;
	tracker->so = so;

	// These are relatively high numbers to seed with; we are essentially saying
	// origin has a variance of 10m; and the quat can be varied by 4 -- which is
	// more than any actual normalized quat could be off by.
	tracker->velocity.Pos.info.variance = 10000;
	tracker->velocity.Rot.info.variance = 10000;
	survive_attach_configf(tracker->so->ctx, VELOCITY_POSITION_VARIANCE_SEC_TAG,
						   &tracker->velocity.Pos.info.variance_per_second);
	survive_attach_configf(tracker->so->ctx, VELOCITY_POSITION_VARIANCE_SEC_TAG,
						   &tracker->velocity.Rot.info.variance_per_second);

	tracker->pose.Pos.info.variance = 10000;
	tracker->pose.Rot.info.variance = 10000;
	survive_attach_configf(tracker->so->ctx, POSE_POSITION_VARIANCE_SEC_TAG,
						   &tracker->pose.Pos.info.variance_per_second);
	survive_attach_configf(tracker->so->ctx, POSE_POSITION_VARIANCE_SEC_TAG,
						   &tracker->pose.Rot.info.variance_per_second);

	tracker->pose.Pos.info.update_fn = update_pose_pos;
	tracker->pose.Rot.info.update_fn = update_pose_rot;

	tracker->velocity.Pos.info.update_fn = update_vel_pos;
	tracker->velocity.Rot.info.update_fn = update_vel_rot;

	survive_attach_configf(tracker->so->ctx, IMU_ACC_VARIANCE_TAG, &tracker->acc_var);
	survive_attach_configf(tracker->so->ctx, IMU_GYRO_VARIANCE_TAG, &tracker->gyro_var);

	struct SurviveContext *ctx = tracker->so->ctx;
	SV_INFO("Initializing Filter:");
	SV_INFO("\t%s: %f", POSE_POSITION_VARIANCE_SEC_TAG, tracker->pose.Pos.info.variance_per_second);
	SV_INFO("\t%s: %f", VELOCITY_POSITION_VARIANCE_SEC_TAG, tracker->velocity.Pos.info.variance_per_second);
	SV_INFO("\t%s: %f", IMU_ACC_VARIANCE_TAG, tracker->acc_var);
	SV_INFO("\t%s: %f", IMU_GYRO_VARIANCE_TAG, tracker->gyro_var);
}

SurvivePose survive_imu_velocity(const SurviveIMUTracker *tracker) {
	SurvivePose rtn;
	copy3d(rtn.Pos, tracker->velocity.Pos.v);
	quatcopy(rtn.Rot, tracker->velocity.Rot.v);
	return rtn;
}
