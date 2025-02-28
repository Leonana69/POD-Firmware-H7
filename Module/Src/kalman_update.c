#include "kalman_update.h"
#include "utils.h"
#include "baro.h"
#include "debug.h"

static int count = 0;

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
        // coreData->tofPreviousHeight = tof->distance;
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
    // Saturate height to avoid division by zero
    float z_g = coreData->S[KC_STATE_Z] < 0.1f ? 0.1f : coreData->S[KC_STATE_Z];
    
    // Get body-frame velocities directly from state (PX/PY are body-frame velocities)
    float vx_body = coreData->S[KC_STATE_PX];
    float vy_body = coreData->S[KC_STATE_PY];
    
    // Convert gyro to rad/s (correct for sensor axis orientation if needed)
    float omegax_b = radians(gyro->x);
    float omegay_b = radians(gyro->y);

    float co = (flow->dt * 12.198) / (0.0254 * z_g);
    // X displacement in body frame prediction and update
    // predics the number of accumulated pixels in the x-direction

    // height_body = z_g / R[2][2]
    // predictedNX = (vx_body - omegay_b * height_body) * dt * CPI
    float predictedNX = co * (vx_body * coreData->R[2][2] - omegay_b * z_g);
    float measuredNX = flow->dpixelx;

    // derive measurement equation with respect to dx and z
    memset(hx, 0, sizeof(hx));
    hx[KC_STATE_Z] = co * ((vx_body * coreData->R[2][2]) / (-z_g));
    hx[KC_STATE_PX] = co * coreData->R[2][2];

    kalmanCoreScalarUpdate(coreData, &Hx, measuredNX - predictedNX, flow->stdDevX);

    float predictedNY = co * (vy_body * coreData->R[2][2] + omegax_b * z_g);
    float measuredNY = flow->dpixely;
    memset(hy, 0, sizeof(hy));
    hy[KC_STATE_Z] = co * ((vy_body * coreData->R[2][2]) / (-z_g));
    hy[KC_STATE_PY] = co * coreData->R[2][2];

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
    static float measNoiseBaro = 0.6f; // meters
    DATA_REGION static float h[KC_STATE_DIM] = { 0 };
    static arm_matrix_instance_f32 H = { 1, KC_STATE_DIM, h };

    h[KC_STATE_Z] = 1;

    float asl = pressureToAltitude(baro->pressure);
    float meas = (asl - coreData->baroReferenceHeight);
    kalmanCoreScalarUpdate(coreData, &H, meas - coreData->S[KC_STATE_Z], measNoiseBaro);
}
