#include "kalman_update.h"
#include "utils.h"

void kalmanCoreUpdateWithTof(kalmanCoreData_t* coreData, const tof_t *tof) {
    static float prevDistance = -1;
    static float offset = 0;

    float h[KC_STATE_DIM] = { 0 };
    arm_matrix_instance_f32 H = { 1, KC_STATE_DIM, h };

    if (fabs(coreData->R[2][2]) > 0.1 && coreData->R[2][2] > 0) {
        // tracking the offset to avoid jumps in the distance
        if (prevDistance < 0) {
            prevDistance = tof->distance;
        }
        if (fabs((prevDistance - tof->distance) / tof->dt) > 0.8f) {
            offset += tof->distance - prevDistance;
        }
        prevDistance = tof->distance;

        float angle = fabsf(acosf(coreData->R[2][2])) - radians(15.0f / 2.0f);
        if (angle < 0.0f)
            angle = 0.0f;
        float predictedDistance = coreData->S[KC_STATE_Z] / cosf(angle);
        // float predictedDistance = coreData->S[KC_STATE_Z] / coreData->R[2][2];
        float measuredDistance = tof->distance - offset; // [m]

        // equation: h = z/((R*z_b)\dot z_b) = z/cos(alpha)
        h[KC_STATE_Z] = 1.0 / cosf(angle);
        // h[KC_STATE_Z] = 1.0 / coreData->R[2][2];

        // Scalar update
        kalmanCoreScalarUpdate(coreData, &H, measuredDistance - predictedDistance, tof->stdDev);
    }
}

void kalmanCoreUpdateWithFlow(kalmanCoreData_t* coreData, const flow_t *flow, const vec3f_t *gyro) {
    static float predictedNX;
    static float predictedNY;
    static float measuredNX;
    static float measuredNY;
    // Inclusion of flow measurements in the EKF done by two scalar updates
    // ~~~ Camera constants ~~~
    float MPC = 0.1;                 // Meter per count coefficient
    float pixelNbr = 35.0;           // [pixels] (same in x and y)
    float thetapix = 0.71674f;       // 2 * sin(FOV/2) [rad] (same in x and y
    //~~~ Body rates ~~~
    // TODO check if this is feasible or if some filtering has to be done
    float omegax_b = radians(gyro->x);
    float omegay_b = radians(gyro->y);
    float dx_g = coreData->S[KC_STATE_PX];
    float dy_g = coreData->S[KC_STATE_PY];
    float z_g;
    // Saturate elevation in prediction and correction to avoid singularities
    if (coreData->S[KC_STATE_Z] < 0.1f)
        z_g = 0.1;
    else
        z_g = coreData->S[KC_STATE_Z];

    // ~~~ X velocity prediction and update ~~~
    // predics the number of accumulated pixels in the x-direction
    float hx[KC_STATE_DIM] = { 0 };
    arm_matrix_instance_f32 Hx = { 1, KC_STATE_DIM, hx };
    predictedNX = (flow->dt * pixelNbr / thetapix) * ((dx_g * coreData->R[2][2] / z_g) - omegay_b);
    measuredNX = flow->dpixelx * MPC;

    // derive measurement equation with respect to dx (and z?)
    hx[KC_STATE_Z] = (pixelNbr * flow->dt / thetapix) * ((coreData->R[2][2] * dx_g) / (-z_g * z_g));
    hx[KC_STATE_PX] = (pixelNbr * flow->dt / thetapix) * (coreData->R[2][2] / z_g);

    /*! X update */
    kalmanCoreScalarUpdate(coreData, &Hx, measuredNX - predictedNX, flow->stdDevX * MPC);

    // ~~~ Y velocity prediction and update ~~~
    float hy[KC_STATE_DIM] = { 0 };
    arm_matrix_instance_f32 Hy = { 1, KC_STATE_DIM, hy };
    predictedNY = (flow->dt * pixelNbr / thetapix) * ((dy_g * coreData->R[2][2] / z_g) + omegax_b);
    measuredNY = flow->dpixely * MPC;

    // derive measurement equation with respect to dy (and z?)
    hy[KC_STATE_Z] = (pixelNbr * flow->dt / thetapix) * ((coreData->R[2][2] * dy_g) / (-z_g * z_g));
    hy[KC_STATE_PY] = (pixelNbr * flow->dt / thetapix) * (coreData->R[2][2] / z_g);

    /*! Y update */
    kalmanCoreScalarUpdate(coreData, &Hy, measuredNY - predictedNY, flow->stdDevY * MPC);
}

void kalmanCoreUpdateWithBaro(kalmanCoreData_t* coreData, const baro_t *baro, bool isFlying) {
    static float measNoiseBaro = 2.0f; // meters
    float h[KC_STATE_DIM] = { 0 };
    arm_matrix_instance_f32 H = { 1, KC_STATE_DIM, h };

    h[KC_STATE_Z] = 1;

    float asl = (1 - powf(baro->pressure / 101315.0f, 0.28686299f)) * (273.15 + 25) / 0.0098f;
    if (!isFlying || coreData->baroReferenceHeight < 1)
        coreData->baroReferenceHeight = asl;

    float meas = (asl - coreData->baroReferenceHeight);
    kalmanCoreScalarUpdate(coreData, &H, meas - coreData->S[KC_STATE_Z], measNoiseBaro);
}
