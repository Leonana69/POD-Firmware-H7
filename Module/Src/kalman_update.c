#include "kalman_update.h"
#include "utils.h"
#include "debug.h"

// static int count = 0;

void kalmanCoreUpdateWithTof(kalmanCoreData_t* coreData, const tof_t *tof, bool isTakingOff) {
    DATA_REGION static float h[KC_STATE_DIM] = { 0 };
    static arm_matrix_instance_f32 H = { 1, KC_STATE_DIM, h };
    
    // if (count++ % 10 == 0) {
    //     DEBUG_PRINT("TOF: %.2f, %.2f\n", coreData->R[2][2], tof->distance);
    // }

    if (coreData->R[2][2] > 0.5) {
        // tracking the offset to avoid jumps in the distance after taking off
        // if (!isTakingOff) {
        //     if (coreData->tofPreviousHeight > 0.0f
        //         && fabs((coreData->tofPreviousHeight - tof->distance) / tof->dt) > 0.8f) {
        //         coreData->tofReferenceHeight += tof->distance - coreData->tofPreviousHeight;
        //     }
        // }
        coreData->tofPreviousHeight = tof->distance;
        float predictedDistance = coreData->S[KC_STATE_Z] / coreData->R[2][2];
        float measuredDistance = tof->distance;// - coreData->tofReferenceHeight; // [m]

        // equation: h = z/((R*z_b)\dot z_b) = z/cos(alpha)
        h[KC_STATE_Z] = 1.0 / coreData->R[2][2];

        // Scalar update
        kalmanCoreScalarUpdate(coreData, &H, measuredDistance - predictedDistance, tof->stdDev);
    }
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
    // Body rates
    float omegax_b = radians(gyro->x);
    float omegay_b = radians(gyro->y);
    float dx_g = coreData->S[KC_STATE_PX];
    float dy_g = coreData->S[KC_STATE_PY];

    float z_g;
    // Saturate elevation in prediction and correction to avoid singularities
    if (coreData->S[KC_STATE_Z] < 0.1f)
        z_g = 0.1f;
    else
        z_g = coreData->S[KC_STATE_Z];

    float co = flow->dt * 12.198 / 0.0254;
    // X displacement prediction and update
    // predics the number of accumulated pixels in the x-direction
    float predictedNX = co * ((dx_g * coreData->R[2][2] / z_g) - omegay_b);
    float measuredNX = flow->dpixelx;

    // derive measurement equation with respect to dx and z
    hx[KC_STATE_Z] = co * ((dx_g * coreData->R[2][2]) / (-z_g * z_g));
    hx[KC_STATE_PX] = co * (coreData->R[2][2] / z_g);

    kalmanCoreScalarUpdate(coreData, &Hx, measuredNX - predictedNX, flow->stdDevX);

    // Y displacement prediction and update
    float predictedNY = co * ((dy_g * coreData->R[2][2] / z_g) + omegax_b);
    float measuredNY = flow->dpixely;

    // derive measurement equation with respect to dy and z
    hy[KC_STATE_Z] = co * ((dy_g * coreData->R[2][2]) / (-z_g * z_g));
    hy[KC_STATE_PY] = co * (coreData->R[2][2] / z_g);

    kalmanCoreScalarUpdate(coreData, &Hy, measuredNY - predictedNY, flow->stdDevY);
}

void kalmanCoreUpdateWithMotor(kalmanCoreData_t* coreData, const motor_t *motor) {
    // Inclusion of flow measurements in the EKF done by two scalar updates
    //~~~ Body rates ~~~
    // TODO check if this is feasible or if some filtering has to be done
    float dx_g = coreData->S[KC_STATE_PX];
    float dy_g = coreData->S[KC_STATE_PY];

    // ~~~ X velocity prediction and update ~~~
    // predics the number of accumulated pixels in the x-direction
    float hx[KC_STATE_DIM] = { 0 };
    arm_matrix_instance_f32 Hx = { 1, KC_STATE_DIM, hx };
    float predictedNX = dx_g;
    float measuredNX = motor->dx / motor->dt;
    hx[KC_STATE_PX] = 1;

    /*! X update */
    kalmanCoreScalarUpdate(coreData, &Hx, measuredNX - predictedNX, motor->stdDevX);

    // ~~~ Y velocity prediction and update ~~~
    float hy[KC_STATE_DIM] = { 0 };
    arm_matrix_instance_f32 Hy = { 1, KC_STATE_DIM, hy };
    float predictedNY = dy_g;
    float measuredNY = motor->dy / motor->dt;

    // derive measurement equation with respect to dy (and z?)
    hy[KC_STATE_PY] = 1;

    /*! Y update */
    kalmanCoreScalarUpdate(coreData, &Hy, measuredNY - predictedNY, motor->stdDevY);
}

void kalmanCoreUpdateWithBaro(kalmanCoreData_t* coreData, const baro_t *baro, bool isFlying) {
    static float measNoiseBaro = 2.0f; // meters
    DATA_REGION static float h[KC_STATE_DIM] = { 0 };
    static arm_matrix_instance_f32 H = { 1, KC_STATE_DIM, h };

    h[KC_STATE_Z] = 1;

    float asl = (1 - powf(baro->pressure / 101315.0f, 0.28686299f)) * (273.15 + 25) / 0.0098f;
    if (!isFlying || coreData->baroReferenceHeight < 1)
        coreData->baroReferenceHeight = asl;

    float meas = (asl - coreData->baroReferenceHeight);
    kalmanCoreScalarUpdate(coreData, &H, meas - coreData->S[KC_STATE_Z], measNoiseBaro);
}
