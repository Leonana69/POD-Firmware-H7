#include "kalman_update.h"
#include "utils.h"
#include "baro.h"
#include "debug.h"

static float last_tof = 0;
void kalmanCoreUpdateWithTof(kalmanCoreData_t* coreData, const tof_t *tof) {
    DATA_REGION static float h[KC_STATE_DIM] = { 0 };
    static arm_matrix_instance_f32 H = { 1, KC_STATE_DIM, h };
    
    const float suddenChangeThreshold = 1.25f;  // Threshold for detecting sudden changes
    static float accumulatedDistance = 0;

    // Initialize last_tof if it's the first valid measurement
    if (last_tof == 0 && tof->distance > 0.5) {
        last_tof = tof->distance;
    }
   
    float predictedDistance = coreData->S[KC_STATE_Z] / coreData->R[2][2]
        - 0.033 * tanf(asinf(coreData->R[2][0])); // The ToF installation is not at the center of the drone

    // Apply filtering only if there is a sudden change
    if (last_tof > 0) {
        if (fabsf(tof->distance - last_tof) / tof->dt > suddenChangeThreshold)
            accumulatedDistance += tof->distance - last_tof;

        last_tof = tof->distance;
    }

    float measurement = tof->distance - accumulatedDistance;
    // Measurement model: h = z / cos(alpha)
    h[KC_STATE_Z] = 1.0 / coreData->R[2][2];

    // Perform the Kalman scalar update
    kalmanCoreScalarUpdate(coreData, &H, measurement - predictedDistance, tof->stdDev);
}

void kalmanCoreUpdateWithFlow(kalmanCoreData_t* coreData, const flow_t *flow, const vec3f_t *gyro) {
    DATA_REGION static float hx[KC_STATE_DIM] = { 0 };
    DATA_REGION static float hy[KC_STATE_DIM] = { 0 };
    static arm_matrix_instance_f32 Hx = { 1, KC_STATE_DIM, hx };
    static arm_matrix_instance_f32 Hy = { 1, KC_STATE_DIM, hy };

    /* For PAA3905, CPI (count per inch) = 12.198 / height * (resolution + 1) / 43
     * Default resolution s 0x2A, which is 42 pixels, so CPI = 12.198 / height
     * CPM (count per meter) = 12.198 / (height * 0.0254)
     */
    // Use last_tof if available, otherwise use the state estimate
    float z_g = last_tof > 0 ? last_tof : (coreData->S[KC_STATE_Z] < 0.05f ? 0.05f : coreData->S[KC_STATE_Z]) / coreData->R[2][2];
    float z_body = z_g - 0.038 * tanf(asinf(coreData->R[2][0])); // The flow sensor is not at the center of the drone

    // Get body-frame velocities directly from state (PX/PY are body-frame velocities)
    float vx_body = coreData->S[KC_STATE_VX];
    float vy_body = coreData->S[KC_STATE_VY];
    
    // Convert gyro to rad/s (correct for sensor axis orientation if needed)
    float omegax_b = radians(gyro->x);
    float omegay_b = radians(gyro->y);

    float co = (flow->dt * 12.198) / (0.0254 * z_body);
    // X displacement in body frame prediction and update
    // predics the number of accumulated pixels in the x-direction

    // height_body = z_g / R[2][2]
    // predictedNX = (vx_body - omegay_b * height_body) * dt * CPI
    float predictedNX = co * (vx_body - omegay_b * z_body);
    float measuredNX = flow->dpixelx;

    // derive measurement equation with respect to dx and z
    memset(hx, 0, sizeof(hx));
    hx[KC_STATE_Z] = co * vx_body / (-z_body) / coreData->R[2][2];
    hx[KC_STATE_VX] = co;
    kalmanCoreScalarUpdate(coreData, &Hx, measuredNX - predictedNX, flow->stdDevX);

    float predictedNY = co * (vy_body + omegax_b * z_body);
    float measuredNY = flow->dpixely;
    memset(hy, 0, sizeof(hy));
    hy[KC_STATE_Z] = co * vy_body / (-z_body) / coreData->R[2][2];
    hy[KC_STATE_VY] = co;
    kalmanCoreScalarUpdate(coreData, &Hy, measuredNY - predictedNY, flow->stdDevY);
}

void kalmanCoreUpdateWithMotor(kalmanCoreData_t* coreData, const motor_t *motor) {
    // Inclusion of flow measurements in the EKF done by two scalar updates
    //~~~ Body rates ~~~
    // TODO check if this is feasible or if some filtering has to be done
    float dx_g = coreData->S[KC_STATE_VX];
    float dy_g = coreData->S[KC_STATE_VY];

    // ~~~ X velocity prediction and update ~~~
    // predics the number of accumulated pixels in the x-direction
    float hx[KC_STATE_DIM] = { 0 };
    arm_matrix_instance_f32 Hx = { 1, KC_STATE_DIM, hx };
    float predictedNX = dx_g;
    float measuredNX = motor->dx / motor->dt;
    hx[KC_STATE_VX] = 1;

    /*! X update */
    kalmanCoreScalarUpdate(coreData, &Hx, measuredNX - predictedNX, motor->stdDevX);

    // ~~~ Y velocity prediction and update ~~~
    float hy[KC_STATE_DIM] = { 0 };
    arm_matrix_instance_f32 Hy = { 1, KC_STATE_DIM, hy };
    float predictedNY = dy_g;
    float measuredNY = motor->dy / motor->dt;

    // derive measurement equation with respect to dy (and z?)
    hy[KC_STATE_VY] = 1;

    /*! Y update */
    kalmanCoreScalarUpdate(coreData, &Hy, measuredNY - predictedNY, motor->stdDevY);
}

void kalmanCoreUpdateWithBaro(kalmanCoreData_t* coreData, const baro_t *baro) {
    static float measNoiseBaro = 0.1f; // meters
    DATA_REGION static float h[KC_STATE_DIM] = { 0 };
    static arm_matrix_instance_f32 H = { 1, KC_STATE_DIM, h };

    h[KC_STATE_Z] = 1;

    float meas = pressureToAltitude(baro->pressure);
    kalmanCoreScalarUpdate(coreData, &H, meas - coreData->S[KC_STATE_Z], measNoiseBaro);
}
