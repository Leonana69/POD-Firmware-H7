/**
 * Authored by Michael Hamer (http://www.mikehamer.info), June 2016
 * Thank you to Mark Mueller (www.mwm.im) for advice during implementation,
 * and for derivation of the original filter in the below-cited paper.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * ============================================================================
 *
 * The Kalman filter implemented in this file is based on the papers:
 *
 * "Fusing ultra-wideband range measurements with accelerometers and rate gyroscopes for quadrocopter state estimation"
 * http://ieeexplore.ieee.org/xpl/articleDetails.jsp?arnumber=7139421
 *
 * and
 *
 * "Covariance Correction Step for Kalman Filtering with an Attitude"
 * http://arc.aiaa.org/doi/abs/10.2514/1.G000848
 *
 * Academic citation would be appreciated.
 *
 * BIBTEX ENTRIES:
      @INPROCEEDINGS{MuellerHamerUWB2015,
      author  = {Mueller, Mark W and Hamer, Michael and D’Andrea, Raffaello},
      title   = {Fusing ultra-wideband range measurements with accelerometers and rate gyroscopes for quadrocopter state estimation},
      booktitle = {2015 IEEE International Conference on Robotics and Automation (ICRA)},
      year    = {2015},
      month   = {May},
      pages   = {1730-1736},
      doi     = {10.1109/ICRA.2015.7139421},
      ISSN    = {1050-4729}}

      @ARTICLE{MuellerCovariance2016,
      author={Mueller, Mark W and Hehn, Markus and D’Andrea, Raffaello},
      title={Covariance Correction Step for Kalman Filtering with an Attitude},
      journal={Journal of Guidance, Control, and Dynamics},
      pages={1--7},
      year={2016},
      publisher={American Institute of Aeronautics and Astronautics}}
 *
 * ============================================================================
 *
 * MAJOR CHANGELOG:
 * 2016.06.28, Mike Hamer: Initial version
 * 2019.04.12, Kristoffer Richardsson: Refactored, separated kalman implementation from OS related functionality
 */

#include "kalman_core.h"
#include "assert.h"
#include "debug.h"
#include "freeRTOS_helper.h"
#include "utils.h"
#include <string.h>

// the reversion of pitch and roll to zero
#ifdef LPS_2D_POSITION_HEIGHT
#define ROLLPITCH_ZERO_REVERSION (0.0f)
#else
#define ROLLPITCH_ZERO_REVERSION (0.001f)
#endif


// The bounds on the covariance, these shouldn't be hit, but sometimes are... why?
#define MAX_COVARIANCE (100.0f)
#define MIN_COVARIANCE (1e-6f)
#define EPSILON        (1e-6f)

// Initial variances, uncertain of position, but know we're stationary and roughly flat
static const float stdDevInitPos_x_y = 100;
static const float stdDevInitPos_z = 1;
static const float stdDevInitVel = 0.01;
static const float stdDevInitAtt_roll_pitch = 0.01;
static const float stdDevInitAtt_yaw = 0.01;

static float procNoiseAcc_xy = 0.5f;
static float procNoiseAcc_z = 1.0f;
static float procNoiseVel = 0;
static float procNoisePos = 0;
static float procNoiseAtt = 0;
static float measNoiseGyro_roll_pitch = 0.1f; // radians per second
static float measNoiseGyro_yaw = 0.1f; // radians per second

float sqrt_f32(float x, float epsilon) {
    float result;
    arm_sqrt_f32(x, &result);
    return result + epsilon;
}

void capCovariance(kalmanCoreData_t* coreData) {
    for (int i = 0; i < KC_STATE_DIM; i++) {
        for (int j = i; j < KC_STATE_DIM; j++) {
            float p = 0.5f * coreData->P[i][j] + 0.5f * coreData->P[j][i];
            if (isnan(p) || p > MAX_COVARIANCE)
                coreData->P[i][j] = coreData->P[j][i] = MAX_COVARIANCE;
            else if (i == j && p < MIN_COVARIANCE)
                coreData->P[i][j] = coreData->P[j][i] = MIN_COVARIANCE;
            else
                coreData->P[i][j] = coreData->P[j][i] = p;
        }
    }
}

void kalmanCoreInit(kalmanCoreData_t* coreData) {
    memset(coreData, 0, sizeof(kalmanCoreData_t));
    // initially, drone is facing positive X
    coreData->q[0] = 1.0;
    // then set the initial rotation matrix to the identity. This only affects
    // the first prediction step, since in the finalization, after shifting
    // attitude errors into the attitude state, the rotation matrix is updated.
    coreData->R[0][0] = 1.0;
    coreData->R[1][1] = 1.0;
    coreData->R[2][2] = 1.0;

    // initialize state variances
    coreData->P[KC_STATE_X][KC_STATE_X] = powf(stdDevInitPos_x_y, 2);
    coreData->P[KC_STATE_Y][KC_STATE_Y] = powf(stdDevInitPos_x_y, 2);
    coreData->P[KC_STATE_Z][KC_STATE_Z] = powf(stdDevInitPos_z, 2);
    coreData->P[KC_STATE_PX][KC_STATE_PX] = powf(stdDevInitVel, 2);
    coreData->P[KC_STATE_PY][KC_STATE_PY] = powf(stdDevInitVel, 2);
    coreData->P[KC_STATE_PZ][KC_STATE_PZ] = powf(stdDevInitVel, 2);
    coreData->P[KC_STATE_D0][KC_STATE_D0] = powf(stdDevInitAtt_roll_pitch, 2);
    coreData->P[KC_STATE_D1][KC_STATE_D1] = powf(stdDevInitAtt_roll_pitch, 2);
    coreData->P[KC_STATE_D2][KC_STATE_D2] = powf(stdDevInitAtt_yaw, 2);

    coreData->Pm.numRows = KC_STATE_DIM;
    coreData->Pm.numCols = KC_STATE_DIM;
    coreData->Pm.pData = (float *)coreData->P;

    coreData->baroReferenceHeight = 0.0;
    coreData->tofReferenceHeight = 0.0;
    coreData->tofPreviousHeight = 0.0;
}

void kalmanCoreScalarUpdate(kalmanCoreData_t* coreData, arm_matrix_instance_f32 *Hm, float error, float stdMeasNoise) {
    // The Kalman gain as a column vector
    DATA_REGION static float K[KC_STATE_DIM];
    static arm_matrix_instance_f32 Km = { KC_STATE_DIM, 1, K };

    // Temporary matrices for the covariance updates
    DATA_REGION static float tmpNN1d[KC_STATE_DIM * KC_STATE_DIM];
    static arm_matrix_instance_f32 tmpNN1m = { KC_STATE_DIM, KC_STATE_DIM, tmpNN1d };

    DATA_REGION static float tmpNN2d[KC_STATE_DIM * KC_STATE_DIM];
    static arm_matrix_instance_f32 tmpNN2m = { KC_STATE_DIM, KC_STATE_DIM, tmpNN2d };

    DATA_REGION static float tmpNN3d[KC_STATE_DIM * KC_STATE_DIM];
    static arm_matrix_instance_f32 tmpNN3m = { KC_STATE_DIM, KC_STATE_DIM, tmpNN3d };

    DATA_REGION static float HTd[KC_STATE_DIM * 1];
    static arm_matrix_instance_f32 HTm = { KC_STATE_DIM, 1, HTd };

    DATA_REGION static float PHTd[KC_STATE_DIM * 1];
    static arm_matrix_instance_f32 PHTm = { KC_STATE_DIM, 1, PHTd };

    ASSERT(Hm->numRows == 1);
    ASSERT(Hm->numCols == KC_STATE_DIM);

    // ====== INNOVATION COVARIANCE ======
    // H^T
    arm_mat_trans_f32(Hm, &HTm);
    // P_n,n-1 * H^T
    arm_mat_mult_f32(&coreData->Pm, &HTm, &PHTm);
    // R_n
    float R = stdMeasNoise * stdMeasNoise;
    // (H * P_n,n-1 * H^T + R_n)
    float HPHR = R;
    // Add the element of HPH' to the above
    for (int i = 0; i < KC_STATE_DIM; i++) {
        HPHR += Hm->pData[i] * PHTd[i];
    }

    ASSERT(!isnan(HPHR));
    // ====== MEASUREMENT UPDATE: K_n = P_n,n-1 * H^T * (H * P_n,n-1 * H^T + R_n)^-1 ======
    // Calculate the Kalman gain and perform the state update
    for (int i = 0; i < KC_STATE_DIM; i++) {
        // kalman gain: K_n = (P_n,n-1 * H^T * (H * P_n,n-1 * H^T + R_n)^-1
        K[i] = PHTd[i] / HPHR; 
        // State update: x_n,n = x_n,n-1 + K_n * error
        coreData->S[i] = coreData->S[i] + K[i] * error; // state update
    }
  
    // ====== COVARIANCE UPDATE: P_n,n = (K_n * H - I) * P_n,n-1 * (K_n * H - I)^T + K_n * R_n * K_n^T ======
    // K_n * H
    arm_mat_mult_f32(&Km, Hm, &tmpNN1m);
    // K_n * H - I
    for (int i = 0; i < KC_STATE_DIM; i++)
        tmpNN1d[KC_STATE_DIM * i + i] -= 1;
    // (K_n * H - I)^T
    arm_mat_trans_f32(&tmpNN1m, &tmpNN2m);
    // (K_n * H - I) * P_n,n-1
    arm_mat_mult_f32(&tmpNN1m, &coreData->Pm, &tmpNN3m);
    // (K_n * H - I) * P_n,n-1 * (K_n * H - I)^T
    arm_mat_mult_f32(&tmpNN3m, &tmpNN2m, &coreData->Pm);

    // add the measurement variance and ensure boundedness and symmetry, but this would cause the covariance to be too small
    // for (int i = 0; i < KC_STATE_DIM; i++) {
    //     for (int j = i; j < KC_STATE_DIM; j++) {
    //         coreData->P[i][j] += K[i] * R * K[j];
    //         coreData->P[j][i] = coreData->P[i][j];
    //     }
    // }

    capCovariance(coreData);
}

void kalmanCorePredict(kalmanCoreData_t* coreData, imu_t *imuData, float dt, bool isFlying) {
  /* Here we discretize (euler forward) and linearise the quadrocopter dynamics in order
   * to push the covariance forward.
   *
   * QUADROCOPTER DYNAMICS (see paper):
   *
   * \dot{x} = R(I + [[d]])p
   * \dot{p} = f/m * e3 - [[\omega]]p - g(I - [[d]])R^-1 e3 //drag negligible
   * \dot{d} = \omega
   *
   * where [[.]] is the cross-product matrix of .
   *       \omega are the gyro measurements
   *       e3 is the column vector [0 0 1]'
   *       I is the identity
   *       R is the current attitude as a rotation matrix
   *       f/m is the mass-normalized motor force (acceleration in the body's z direction)
   *       g is gravity
   *       x, p, d are the quad's states
   * note that d (attitude error) is zero at the beginning of each iteration,
   * since error information is incorporated into R after each Kalman update.
   */

    // The state transition matrix
    DATA_REGION static float F[KC_STATE_DIM][KC_STATE_DIM];
    static arm_matrix_instance_f32 Fm = { KC_STATE_DIM, KC_STATE_DIM, (float *) F }; // linearized dynamics for covariance update;

    // Temporary matrices for the covariance updates
    DATA_REGION static float tmpNN1d[KC_STATE_DIM * KC_STATE_DIM];
    static arm_matrix_instance_f32 tmpNN1m = { KC_STATE_DIM, KC_STATE_DIM, tmpNN1d };

    DATA_REGION static float tmpNN2d[KC_STATE_DIM * KC_STATE_DIM];
    static arm_matrix_instance_f32 tmpNN2m = { KC_STATE_DIM, KC_STATE_DIM, tmpNN2d };

    vec3f_t *gyro = &imuData->gyro;
    vec3f_t *accel = &imuData->accel;

    // ====== DYNAMICS LINEARIZATION ======
    /*! State transition matrix */
    memset(F, 0, sizeof(float) * KC_STATE_DIM * KC_STATE_DIM);
    // Initialize as the identity
    for (int i = 0; i < KC_STATE_DIM; i++)
        F[i][i] = 1;

    // position from body-frame velocity
    F[KC_STATE_X][KC_STATE_PX] = coreData->R[0][0] * dt;
    F[KC_STATE_Y][KC_STATE_PX] = coreData->R[1][0] * dt;
    F[KC_STATE_Z][KC_STATE_PX] = coreData->R[2][0] * dt;

    F[KC_STATE_X][KC_STATE_PY] = coreData->R[0][1] * dt;
    F[KC_STATE_Y][KC_STATE_PY] = coreData->R[1][1] * dt;
    F[KC_STATE_Z][KC_STATE_PY] = coreData->R[2][1] * dt;

    F[KC_STATE_X][KC_STATE_PZ] = coreData->R[0][2] * dt;
    F[KC_STATE_Y][KC_STATE_PZ] = coreData->R[1][2] * dt;
    F[KC_STATE_Z][KC_STATE_PZ] = coreData->R[2][2] * dt;

    // position from attitude error
    F[KC_STATE_X][KC_STATE_D0] = (coreData->S[KC_STATE_PY] * coreData->R[0][2] - coreData->S[KC_STATE_PZ] * coreData->R[0][1]) * dt;
    F[KC_STATE_Y][KC_STATE_D0] = (coreData->S[KC_STATE_PY] * coreData->R[1][2] - coreData->S[KC_STATE_PZ] * coreData->R[1][1]) * dt;
    F[KC_STATE_Z][KC_STATE_D0] = (coreData->S[KC_STATE_PY] * coreData->R[2][2] - coreData->S[KC_STATE_PZ] * coreData->R[2][1]) * dt;

    F[KC_STATE_X][KC_STATE_D1] = (-coreData->S[KC_STATE_PX] * coreData->R[0][2] + coreData->S[KC_STATE_PZ] * coreData->R[0][0]) * dt;
    F[KC_STATE_Y][KC_STATE_D1] = (-coreData->S[KC_STATE_PX] * coreData->R[1][2] + coreData->S[KC_STATE_PZ] * coreData->R[1][0]) * dt;
    F[KC_STATE_Z][KC_STATE_D1] = (-coreData->S[KC_STATE_PX] * coreData->R[2][2] + coreData->S[KC_STATE_PZ] * coreData->R[2][0]) * dt;

    F[KC_STATE_X][KC_STATE_D2] = (coreData->S[KC_STATE_PX] * coreData->R[0][1] - coreData->S[KC_STATE_PY] * coreData->R[0][0]) * dt;
    F[KC_STATE_Y][KC_STATE_D2] = (coreData->S[KC_STATE_PX] * coreData->R[1][1] - coreData->S[KC_STATE_PY] * coreData->R[1][0]) * dt;
    F[KC_STATE_Z][KC_STATE_D2] = (coreData->S[KC_STATE_PX] * coreData->R[2][1] - coreData->S[KC_STATE_PY] * coreData->R[2][0]) * dt;

    // body-frame velocity change from attitude change (rotation)
    F[KC_STATE_PX][KC_STATE_PX] = 1.0;
    F[KC_STATE_PY][KC_STATE_PX] = -gyro->z * dt;
    F[KC_STATE_PZ][KC_STATE_PX] = gyro->y * dt;

    F[KC_STATE_PX][KC_STATE_PY] = gyro->z * dt;
    F[KC_STATE_PY][KC_STATE_PY] = 1.0;
    F[KC_STATE_PZ][KC_STATE_PY] = -gyro->x * dt;

    F[KC_STATE_PX][KC_STATE_PZ] = -gyro->y * dt;
    F[KC_STATE_PY][KC_STATE_PZ] = gyro->x * dt;
    F[KC_STATE_PZ][KC_STATE_PZ] = 1.0;

    // body-frame velocity from attitude error
    F[KC_STATE_PX][KC_STATE_D0] = 0.0;
    // delta_V_PY = -g_PZ * sin(delta_roll) * dt = -g_PZ * delta_roll * dt
    F[KC_STATE_PY][KC_STATE_D0] = -GRAVITY_EARTH * coreData->R[2][2] * dt;
    F[KC_STATE_PZ][KC_STATE_D0] = GRAVITY_EARTH * coreData->R[2][1] * dt;

    F[KC_STATE_PX][KC_STATE_D1] = GRAVITY_EARTH * coreData->R[2][2] * dt;
    F[KC_STATE_PY][KC_STATE_D1] = 0.0;
    F[KC_STATE_PZ][KC_STATE_D1] = -GRAVITY_EARTH * coreData->R[2][0] * dt;

    F[KC_STATE_PX][KC_STATE_D2] = -GRAVITY_EARTH * coreData->R[2][1] * dt;
    F[KC_STATE_PY][KC_STATE_D2] = GRAVITY_EARTH * coreData->R[2][0] * dt;
    F[KC_STATE_PZ][KC_STATE_D2] = 0.0;

    // attitude error from attitude error
    /**
     * At first glance, it may not be clear where the next values come from, since they do not appear directly in the
     * dynamics. In this prediction step, we skip the step of first updating attitude-error, and then incorporating the
     * new error into the current attitude (which requires a rotation of the attitude-error covariance). Instead, we
     * directly update the body attitude, however still need to rotate the covariance, which is what you see below.
     *
     * This comes from a second order approximation to:
     * Sigma_post = exps(-d) Sigma_pre exps(-d)'
     *            ~ (I + [[-d]] + [[-d]]^2 / 2) Sigma_pre (I + [[-d]] + [[-d]]^2 / 2)'
     * where d is the attitude error expressed as Rodriges parameters, ie. d0 = 1/2*gyro.x*dt under the assumption that
     * d = [0,0,0] at the beginning of each prediction step and that gyro.x is constant over the sampling period
     *
     * As derived in "Covariance Correction Step for Kalman Filtering with an Attitude"
     * http://arc.aiaa.org/doi/abs/10.2514/1.G000848
     */
    float d0 = gyro->x * dt / 2;
    float d1 = gyro->y * dt / 2;
    float d2 = gyro->z * dt / 2;

    F[KC_STATE_D0][KC_STATE_D0] = 1 - d1 * d1 / 2 - d2 * d2 / 2;
    F[KC_STATE_D0][KC_STATE_D1] = d2 + d0 * d1 / 2;
    F[KC_STATE_D0][KC_STATE_D2] = -d1 + d0 * d2 / 2;

    F[KC_STATE_D1][KC_STATE_D0] = -d2 + d0 * d1 / 2;
    F[KC_STATE_D1][KC_STATE_D1] = 1 - d0 * d0 / 2 - d2 * d2 / 2;
    F[KC_STATE_D1][KC_STATE_D2] = d0 + d1 * d2 / 2;

    F[KC_STATE_D2][KC_STATE_D0] = d1 + d0 * d2 / 2;
    F[KC_STATE_D2][KC_STATE_D1] = -d0 + d1 * d2 / 2;
    F[KC_STATE_D2][KC_STATE_D2] = 1 - d0 * d0 / 2 - d1 * d1 / 2;


    // ====== COVARIANCE UPDATE: P_n+1,n = F * P_n,n * F^T + Q ======
    // F * P_n,n
    arm_mat_mult_f32(&Fm, &coreData->Pm, &tmpNN1m);
    // F^T
    arm_mat_trans_f32(&Fm, &tmpNN2m); 
    // F * P_n,n * F^T
    arm_mat_mult_f32(&tmpNN1m, &tmpNN2m, &coreData->Pm);
    // Process noise Q is added after the return from the prediction step

    // ====== PREDICTION STEP: S_n+1,n = F * S_n,n + G * u_n + w_n ======
    // The control input u_n depends on whether we're on the ground, or in flight.
    // When flying, the accelerometer directly measures thrust (hence is useless to estimate body angle while flying)
    float dx, dy, dz;
    float tmpSPX, tmpSPY, tmpSPZ;
    float dt2_2 = dt * dt / 2.0f;
    if (isFlying) {
        // only acceleration in z direction
        // Use accelerometer and not commanded thrust, as this has proper physical units

        // position updates in the body frame (will be rotated to inertial frame)
        dx = coreData->S[KC_STATE_PX] * dt;
        dy = coreData->S[KC_STATE_PY] * dt;
        dz = coreData->S[KC_STATE_PZ] * dt + accel->z * dt2_2; // thrust can only be produced in the body's Z direction

        // position update
        coreData->S[KC_STATE_X] += coreData->R[0][0] * dx + coreData->R[0][1] * dy + coreData->R[0][2] * dz;
        coreData->S[KC_STATE_Y] += coreData->R[1][0] * dx + coreData->R[1][1] * dy + coreData->R[1][2] * dz;
        coreData->S[KC_STATE_Z] += coreData->R[2][0] * dx + coreData->R[2][1] * dy + coreData->R[2][2] * dz - GRAVITY_EARTH * dt2_2;

        // keep previous time step's state for the update
        tmpSPX = coreData->S[KC_STATE_PX];
        tmpSPY = coreData->S[KC_STATE_PY];
        tmpSPZ = coreData->S[KC_STATE_PZ];

        // body-velocity update: accelerometers - gyros cross velocity - gravity in body frame
        coreData->S[KC_STATE_PX] += dt * (gyro->z * tmpSPY - gyro->y * tmpSPZ - GRAVITY_EARTH * coreData->R[2][0]);
        coreData->S[KC_STATE_PY] += dt * (-gyro->z * tmpSPX + gyro->x * tmpSPZ - GRAVITY_EARTH * coreData->R[2][1]);
        coreData->S[KC_STATE_PZ] += dt * (accel->z + gyro->y * tmpSPX - gyro->x * tmpSPY - GRAVITY_EARTH * coreData->R[2][2]);
    } else {
        // Acceleration can be in any direction, as measured by the accelerometer. This occurs, eg. in freefall or while being carried.
        // position updates in the body frame (will be rotated to inertial frame)
        dx = coreData->S[KC_STATE_PX] * dt + accel->x * dt2_2;
        dy = coreData->S[KC_STATE_PY] * dt + accel->y * dt2_2;
        dz = coreData->S[KC_STATE_PZ] * dt + accel->z * dt2_2; // thrust can only be produced in the body's Z direction

        // position update
        coreData->S[KC_STATE_X] += coreData->R[0][0] * dx + coreData->R[0][1] * dy + coreData->R[0][2] * dz;
        coreData->S[KC_STATE_Y] += coreData->R[1][0] * dx + coreData->R[1][1] * dy + coreData->R[1][2] * dz;
        coreData->S[KC_STATE_Z] += coreData->R[2][0] * dx + coreData->R[2][1] * dy + coreData->R[2][2] * dz - GRAVITY_EARTH * dt2_2;

        // keep previous time step's state for the update
        tmpSPX = coreData->S[KC_STATE_PX];
        tmpSPY = coreData->S[KC_STATE_PY];
        tmpSPZ = coreData->S[KC_STATE_PZ];

        // body-velocity update: accelerometers - gyros cross velocity - gravity in body frame
        coreData->S[KC_STATE_PX] += dt * (accel->x + gyro->z * tmpSPY - gyro->y * tmpSPZ - GRAVITY_EARTH * coreData->R[2][0]);
        coreData->S[KC_STATE_PY] += dt * (accel->y - gyro->z * tmpSPX + gyro->x * tmpSPZ - GRAVITY_EARTH * coreData->R[2][1]);
        coreData->S[KC_STATE_PZ] += dt * (accel->z + gyro->y * tmpSPX - gyro->x * tmpSPY - GRAVITY_EARTH * coreData->R[2][2]);
    }

    // attitude update (rotate by gyroscope), we do this in quaternions
    // this is the gyroscope angular velocity integrated over the sample period
    float dtwx = dt * gyro->x;
    float dtwy = dt * gyro->y;
    float dtwz = dt * gyro->z;

    // compute the quaternion values in [w,x,y,z] order
    float angle = sqrt_f32(dtwx * dtwx + dtwy * dtwy + dtwz * dtwz, EPSILON);
    float ca = arm_cos_f32(angle / 2.0f);
    float sa = arm_sin_f32(angle / 2.0f);
    float dq[4] = {ca , sa * dtwx / angle , sa * dtwy / angle , sa * dtwz / angle};

    float tmpq0, tmpq1, tmpq2, tmpq3;
    // rotate the quad's attitude by the delta quaternion vector computed above
    tmpq0 = dq[0] * coreData->q[0] - dq[1] * coreData->q[1] - dq[2] * coreData->q[2] - dq[3] * coreData->q[3];
    tmpq1 = dq[1] * coreData->q[0] + dq[0] * coreData->q[1] + dq[3] * coreData->q[2] - dq[2] * coreData->q[3];
    tmpq2 = dq[2] * coreData->q[0] - dq[3] * coreData->q[1] + dq[0] * coreData->q[2] + dq[1] * coreData->q[3];
    tmpq3 = dq[3] * coreData->q[0] + dq[2] * coreData->q[1] - dq[1] * coreData->q[2] + dq[0] * coreData->q[3];

    /* This reversion would cause yaw estimation diminish to zero */
    // if (!isFlying) {
    //     float keep = 1.0f - ROLLPITCH_ZERO_REVERSION;
    //     tmpq0 = keep * tmpq0 + ROLLPITCH_ZERO_REVERSION * 1.0;
    //     tmpq1 = keep * tmpq1 + ROLLPITCH_ZERO_REVERSION * 0;
    //     tmpq2 = keep * tmpq2 + ROLLPITCH_ZERO_REVERSION * 0;
    //     tmpq3 = keep * tmpq3 + ROLLPITCH_ZERO_REVERSION * 0;
    // }

    // normalize and store the result
    float norm = sqrt_f32(tmpq0 * tmpq0 + tmpq1 * tmpq1 + tmpq2 * tmpq2 + tmpq3 * tmpq3, EPSILON);

    coreData->q[0] = tmpq0 / norm;
    coreData->q[1] = tmpq1 / norm;
    coreData->q[2] = tmpq2 / norm;
    coreData->q[3] = tmpq3 / norm;
}

void kalmanCoreAddProcessNoise(kalmanCoreData_t* coreData, float dt) {
    if (dt > 0) {
        // Add process noise on position
        coreData->P[KC_STATE_X][KC_STATE_X] += powf(procNoiseAcc_xy * dt * dt, 2) 
                                             + powf(procNoiseVel * dt, 2) 
                                             + powf(procNoisePos, 2);
        coreData->P[KC_STATE_Y][KC_STATE_Y] += powf(procNoiseAcc_xy * dt * dt, 2) 
                                             + powf(procNoiseVel * dt, 2) 
                                             + powf(procNoisePos, 2);
        coreData->P[KC_STATE_Z][KC_STATE_Z] += powf(procNoiseAcc_z * dt * dt, 2) 
                                             + powf(procNoiseVel * dt, 2) 
                                             + powf(procNoisePos, 2);

        // Add process noise on velocity
        coreData->P[KC_STATE_PX][KC_STATE_PX] += powf(procNoiseAcc_xy * dt, 2) 
                                               + powf(procNoiseVel, 2);
        coreData->P[KC_STATE_PY][KC_STATE_PY] += powf(procNoiseAcc_xy * dt, 2) 
                                               + powf(procNoiseVel, 2);
        coreData->P[KC_STATE_PZ][KC_STATE_PZ] += powf(procNoiseAcc_z * dt, 2) 
                                               + powf(procNoiseVel, 2);

        // Add process noise on attitude
        coreData->P[KC_STATE_D0][KC_STATE_D0] += powf(measNoiseGyro_roll_pitch * dt, 2) 
                                               + powf(procNoiseAtt, 2);
        coreData->P[KC_STATE_D1][KC_STATE_D1] += powf(measNoiseGyro_roll_pitch * dt, 2) 
                                               + powf(procNoiseAtt, 2);
        coreData->P[KC_STATE_D2][KC_STATE_D2] += powf(measNoiseGyro_yaw * dt, 2) 
                                               + powf(procNoiseAtt, 2);
    }

    // Cap covariance values to ensure numerical stability
    capCovariance(coreData);
}

void kalmanCoreFinalize(kalmanCoreData_t* coreData) {
    // Matrix to rotate the attitude covariances once updated
    DATA_REGION static float F[KC_STATE_DIM][KC_STATE_DIM];
    static arm_matrix_instance_f32 Fm = { KC_STATE_DIM, KC_STATE_DIM, (float *)F };

    // Temporary matrices for the covariance updates
    DATA_REGION static float tmpNN1d[KC_STATE_DIM * KC_STATE_DIM];
    static arm_matrix_instance_f32 tmpNN1m = { KC_STATE_DIM, KC_STATE_DIM, tmpNN1d };

    DATA_REGION static float tmpNN2d[KC_STATE_DIM * KC_STATE_DIM];
    static arm_matrix_instance_f32 tmpNN2m = { KC_STATE_DIM, KC_STATE_DIM, tmpNN2d };

    // Incorporate the attitude error (Kalman filter state) with the attitude
    float v0 = coreData->S[KC_STATE_D0];
    float v1 = coreData->S[KC_STATE_D1];
    float v2 = coreData->S[KC_STATE_D2];

    // Move attitude error into attitude if any of the angle errors are large enough
    if ((fabsf(v0) > 1e-4f || fabsf(v1) > 1e-4f || fabsf(v2) > 1e-4f) && (fabsf(v0) < 10 && fabsf(v1) < 10 && fabsf(v2) < 10)) {
        float angle = sqrt_f32(v0 * v0 + v1 * v1 + v2 * v2, EPSILON);
        float ca = arm_cos_f32(angle / 2.0f);
        float sa = arm_sin_f32(angle / 2.0f);
        float dq[4] = { ca, sa * v0 / angle, sa * v1 / angle, sa * v2 / angle };

        // rotate the quad's attitude by the delta quaternion vector computed above
        float tmpq0 = dq[0] * coreData->q[0] - dq[1] * coreData->q[1] - dq[2] * coreData->q[2] - dq[3] * coreData->q[3];
        float tmpq1 = dq[1] * coreData->q[0] + dq[0] * coreData->q[1] + dq[3] * coreData->q[2] - dq[2] * coreData->q[3];
        float tmpq2 = dq[2] * coreData->q[0] - dq[3] * coreData->q[1] + dq[0] * coreData->q[2] + dq[1] * coreData->q[3];
        float tmpq3 = dq[3] * coreData->q[0] + dq[2] * coreData->q[1] - dq[1] * coreData->q[2] + dq[0] * coreData->q[3];

        // normalize and store the result
        float norm = sqrt_f32(tmpq0 * tmpq0 + tmpq1 * tmpq1 + tmpq2 * tmpq2 + tmpq3 * tmpq3, EPSILON);
        coreData->q[0] = tmpq0 / norm;
        coreData->q[1] = tmpq1 / norm;
        coreData->q[2] = tmpq2 / norm;
        coreData->q[3] = tmpq3 / norm;

    /** Rotate the covariance, since we've rotated the body
        *
        * This comes from a second order approximation to:
        * Sigma_post = exps(-d) Sigma_pre exps(-d)'
        *            ~ (I + [[-d]] + [[-d]]^2 / 2) Sigma_pre (I + [[-d]] + [[-d]]^2 / 2)'
        * where d is the attitude error expressed as Rodriges parameters, ie. d = tan(|v|/2)*v/|v|
        *
        * As derived in "Covariance Correction Step for Kalman Filtering with an Attitude"
        * http://arc.aiaa.org/doi/abs/10.2514/1.G000848
        */

        float d0 = v0 / 2; // the attitude error vector (v0,v1,v2) is small,
        float d1 = v1 / 2; // so we use a first order approximation to d0 = tan(|v0|/2)*v0/|v0|
        float d2 = v2 / 2;

        memset(F, 0, sizeof(float) * KC_STATE_DIM * KC_STATE_DIM);
        F[KC_STATE_X][KC_STATE_X] = 1;
        F[KC_STATE_Y][KC_STATE_Y] = 1;
        F[KC_STATE_Z][KC_STATE_Z] = 1;

        F[KC_STATE_PX][KC_STATE_PX] = 1;
        F[KC_STATE_PY][KC_STATE_PY] = 1;
        F[KC_STATE_PZ][KC_STATE_PZ] = 1;

        F[KC_STATE_D0][KC_STATE_D0] = 1 - d1 * d1 / 2 - d2 * d2 / 2;
        F[KC_STATE_D0][KC_STATE_D1] = d2 + d0 * d1 / 2;
        F[KC_STATE_D0][KC_STATE_D2] = -d1 + d0 * d2 / 2;

        F[KC_STATE_D1][KC_STATE_D0] = -d2 + d0 * d1 / 2;
        F[KC_STATE_D1][KC_STATE_D1] = 1 - d0 * d0 / 2 - d2 * d2 / 2;
        F[KC_STATE_D1][KC_STATE_D2] = d0 + d1 * d2 / 2;

        F[KC_STATE_D2][KC_STATE_D0] = d1 + d0 * d2 / 2;
        F[KC_STATE_D2][KC_STATE_D1] = -d0 + d1 * d2 / 2;
        F[KC_STATE_D2][KC_STATE_D2] = 1 - d0 * d0 / 2 - d1 * d1 / 2;

        arm_mat_trans_f32(&Fm, &tmpNN1m); // F'
        arm_mat_mult_f32(&Fm, &coreData->Pm, &tmpNN2m); // FP
        arm_mat_mult_f32(&tmpNN2m, &tmpNN1m, &coreData->Pm); // FPF'
    }

    // convert the new attitude to a rotation matrix, such that we can rotate body-frame velocity and acc
    coreData->R[0][0] = coreData->q[0] * coreData->q[0] + coreData->q[1] * coreData->q[1] - coreData->q[2] * coreData->q[2] - coreData->q[3] * coreData->q[3];
    coreData->R[0][1] = 2 * coreData->q[1] * coreData->q[2] - 2 * coreData->q[0] * coreData->q[3];
    coreData->R[0][2] = 2 * coreData->q[1] * coreData->q[3] + 2 * coreData->q[0] * coreData->q[2];

    coreData->R[1][0] = 2 * coreData->q[1] * coreData->q[2] + 2 * coreData->q[0] * coreData->q[3];
    coreData->R[1][1] = coreData->q[0] * coreData->q[0] - coreData->q[1] * coreData->q[1] + coreData->q[2] * coreData->q[2] - coreData->q[3] * coreData->q[3];
    coreData->R[1][2] = 2 * coreData->q[2] * coreData->q[3] - 2 * coreData->q[0] * coreData->q[1];

    coreData->R[2][0] = 2 * coreData->q[1] * coreData->q[3] - 2 * coreData->q[0] * coreData->q[2];
    coreData->R[2][1] = 2 * coreData->q[2] * coreData->q[3] + 2 * coreData->q[0] * coreData->q[1];
    coreData->R[2][2] = coreData->q[0] * coreData->q[0] - coreData->q[1] * coreData->q[1] - coreData->q[2] * coreData->q[2] + coreData->q[3] * coreData->q[3];

    // reset the attitude error
    coreData->S[KC_STATE_D0] = 0;
    coreData->S[KC_STATE_D1] = 0;
    coreData->S[KC_STATE_D2] = 0;

    // enforce symmetry of the covariance matrix, and ensure the values stay bounded
    capCovariance(coreData);
}

void kalmanCoreExternalizeState(const kalmanCoreData_t* coreData, state_t *state, const vec3f_t *accel, uint32_t tick) {
    // position state is already in world frame
    state->position = (position_t) {
        .x = coreData->S[KC_STATE_X],
        .y = coreData->S[KC_STATE_Y],
        .z = coreData->S[KC_STATE_Z],
    };

    // velocity is in body frame and needs to be rotated to world frame
    state->velocity = (velocity_t) {
        .x = coreData->R[0][0] * coreData->S[KC_STATE_PX] + coreData->R[0][1] * coreData->S[KC_STATE_PY] + coreData->R[0][2] * coreData->S[KC_STATE_PZ],
        .y = coreData->R[1][0] * coreData->S[KC_STATE_PX] + coreData->R[1][1] * coreData->S[KC_STATE_PY] + coreData->R[1][2] * coreData->S[KC_STATE_PZ],
        .z = coreData->R[2][0] * coreData->S[KC_STATE_PX] + coreData->R[2][1] * coreData->S[KC_STATE_PY] + coreData->R[2][2] * coreData->S[KC_STATE_PZ],
    };

    // Accelerometer measurements are in the body frame and need to be rotated to world frame.
    // Furthermore, the legacy code requires acc.z to be acceleration without gravity.
    // Finally, note that these accelerations are in Gs, and not in m/s^2, hence - 1 for removing gravity
    state->accel = (accel_t) {
        .x = coreData->R[0][0] * accel->x + coreData->R[0][1] * accel->y + coreData->R[0][2] * accel->z,
        .y = coreData->R[1][0] * accel->x + coreData->R[1][1] * accel->y + coreData->R[1][2] * accel->z,
        .z = coreData->R[2][0] * accel->x + coreData->R[2][1] * accel->y + coreData->R[2][2] * accel->z - 1.0,
    };

    // convert the new attitude into Euler YPR
    float yaw = atan2f(
        2 * (coreData->q[1] * coreData->q[2] + coreData->q[0] * coreData->q[3]),
        coreData->q[0] * coreData->q[0] + coreData->q[1] * coreData->q[1] - coreData->q[2] * coreData->q[2] - coreData->q[3] * coreData->q[3]
    );
    float pitch = asinf(-2 * (coreData->q[1] * coreData->q[3] - coreData->q[0] * coreData->q[2]));
    float roll = atan2f(
        2 * (coreData->q[2] * coreData->q[3]+coreData->q[0] * coreData->q[1]),
        coreData->q[0] * coreData->q[0] - coreData->q[1] * coreData->q[1] - coreData->q[2] * coreData->q[2] + coreData->q[3] * coreData->q[3]
    );

    // Save attitude, adjusted for the legacy CF2 body coordinate system
    state->attitude = (attitude_t) {
        .roll = degrees(roll),
        .pitch = degrees(pitch),
        .yaw = degrees(yaw)
    };

    // Save quaternion, hopefully one day this could be used in a better controller.
    // Note that this is not adjusted for the legacy coordinate system
    state->attitudeQuat = (quaternion_t) {
        .w = coreData->q[0],
        .x = coreData->q[1],
        .y = coreData->q[2],
        .z = coreData->q[3]
    };
}

bool kalmanCoreCheckBounds(kalmanCoreData_t* coreData) {
    float maxPosition = 100;
    float maxVelocity = 10;
    for (int i = 0; i < 3; i++) {
        if (fabsf(coreData->S[KC_STATE_X + i]) > maxPosition) {
            return false;
        }
        if (fabsf(coreData->S[KC_STATE_PX + i]) > maxVelocity) {
            return false;
        }
    }
    return true;
}