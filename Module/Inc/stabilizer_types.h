#ifndef __STABILIZER_TYPES_H__
#define __STABILIZER_TYPES_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

typedef float scalar_t;
/** Data structure used by the stabilizer subsystem.
 *  All have a timestamp to be set when the data is calculated.
 */
struct geo_info_s {
	uint32_t timestamp;
	union {
		struct {
			scalar_t roll;
			scalar_t pitch;
			scalar_t yaw;
		};
		struct {
			scalar_t x;
			scalar_t y;
			scalar_t z;
		};
	};
};

typedef struct geo_info_s attitude_t;
typedef struct geo_info_s palstance_t;
typedef struct geo_info_s position_t;
typedef struct geo_info_s velocity_t;
typedef struct geo_info_s accel_t;

/* Orientation as a quaternion */
typedef struct {
    uint32_t timestamp;
    union {
        struct {
            scalar_t q0;
            scalar_t q1;
            scalar_t q2;
            scalar_t q3;
        };
        struct {
            scalar_t x;
            scalar_t y;
            scalar_t z;
            scalar_t w;
        };
    };
} quaternion_t;

typedef union {
    struct {
        scalar_t x;
        scalar_t y;
        scalar_t z;
    };
    scalar_t v[3];
} vec3f_t;

typedef struct {
    attitude_t attitude;      // deg (legacy CF2 body coordinate system, where pitch is inverted)
    quaternion_t attitudeQuat;
    position_t position;      // m
    velocity_t velocity;      // m/s
    accel_t accel;            // Gs (but acc.z without considering gravity)
} state_t;

typedef struct {
    attitude_t attitude;
    scalar_t thrust;
} control_t;

typedef enum {
    STABILIZE_DISABLE = 0,
    STABILIZE_ABSOLUTE,
    STABILIZE_VELOCITY,
} stab_mode_t;

typedef struct {
    uint32_t timestamp;

    scalar_t thrust;
    attitude_t attitude;      // deg
    palstance_t palstance;  // deg/s
    quaternion_t attitudeQuat;
    position_t position;      // m
    velocity_t velocity;      // m/s
    accel_t acceleration;     // m/s^2
    bool velocity_body;       // true if velocity is given in body frame; false if velocity is given in world frame

    struct {
        stab_mode_t x;
        stab_mode_t y;
        stab_mode_t z;
        stab_mode_t roll;
        stab_mode_t pitch;
        stab_mode_t yaw;
    } mode;
} setpoint_t;

/*
 * Data structure used by all sensors
 */
typedef struct {
    scalar_t pressure;           // mbar
    scalar_t temperature;        // degree Celcius
    scalar_t asl;                // m (ASL = altitude above sea level)
} baro_t;

typedef struct {
    scalar_t distance;          // m
    scalar_t stdDev;            // m
    scalar_t dt;
} tof_t;

typedef struct {
    union {
        struct {
            scalar_t dpixelx;  // Accumulated pixel count x
            scalar_t dpixely;  // Accumulated pixel count y
        };
        scalar_t dpixel[2];  // Accumulated pixel count
    };
    scalar_t stdDevX;      // Measurement standard deviation
    scalar_t stdDevY;      // Measurement standard deviation
    scalar_t dt;           // Time during which pixels were accumulated
} flow_t;

typedef struct {
    vec3f_t accel;             // Gs
    vec3f_t gyro;              // deg/s
} imu_t;

// Frequencies to bo used with the RATE_DO_EXECUTE_HZ macro. Do NOT use an arbitrary number.
#define RATE_1000_HZ 1000
#define RATE_500_HZ 500
#define RATE_100_HZ 100
#define RATE_50_HZ 50
#define RATE_25_HZ 25

#define DT_1000_HZ 0.001
#define DT_500_HZ 0.002
#define DT_100_HZ 0.01
#define DT_50_HZ 0.02
#define DT_25_HZ 0.04

#define RATE_MAIN_LOOP 			RATE_1000_HZ
#define ATTITUDE_RATE 			RATE_500_HZ
#define ATTITUDE_UPDATE_DT 	    DT_500_HZ
#define POSITION_RATE 			RATE_100_HZ
#define POSITION_UPDATE_DT 	    DT_100_HZ

#define RATE_DO_EXECUTE(RATE_HZ, TICK) ((TICK % (RATE_MAIN_LOOP / RATE_HZ)) == 0)

#ifdef __cplusplus
}
#endif

#endif
