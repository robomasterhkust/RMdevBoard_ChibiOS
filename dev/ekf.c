//
// Created by Yang Shaohui on 2018.03.31.
//

#include "math.h"

#include "ekf.h"
#include "canBusProcess.h"
#include "mpu6500.h"

static volatile EKF_vec ekf_vec;
static volatile EKF_mat ekf_mat;
static volatile EKF_conf ekf_conf;

EKF_vec *get_ekf_vec(void) {
    return &ekf_vec;
}

EKF_mat *get_ekf_mat(void) {
    return &ekf_mat;
}

EKF_conf *get_ekf_conf(void) {
    return &ekf_conf;
}

void init_matrix_diagonal(float **a, int size, int diag) {
    for (int i = 0; i < size; i++) {
        for (int j = 0; j < size; j++) {
            if (i == j)
                a[i][j] = diag;
            else
                a[i][j] = 0;
        }
    }
}

void EKF_init(void) {
    UWB_canStruct *uwb = can_getUWB();
    PIMUStruct imu = imu_get();

    float variance_q = 1.0;
    float variance_r = 1.0;
    init_matrix_diagonal(ekf_mat.n_cov_Q_const, PROCESS_NOISE_N_NUM, variance_q);
    init_matrix_diagonal(ekf_mat.v_cov_R_const, MEASUREMENT_NOISE_V_NUM, variance_r);

    ekf_vec.state_x_mu[x_p] = (float) uwb->x_world_cm / 100;
    ekf_vec.state_x_mu[y_p] = (float) uwb->y_world_cm / 100;
    ekf_vec.state_x_mu[theta] = (float) uwb->theta_world_deg_100 / (100 * 2 * M_PI);
    ekf_vec.state_x_mu[x_v] = 0;
    ekf_vec.state_x_mu[y_v] = 0;
    ekf_vec.state_x_mu[b_g] = imu->_gyroBias[Z];
    ekf_vec.state_x_mu[b_ax] = imu->_accelBias[X];
    ekf_vec.state_x_mu[b_ay] = imu->_accelBias[Y];

    init_matrix_diagonal(ekf_mat.x_cov_Sigma, STATE_X_NUM, 1);
    ekf_conf.t_prev = chVTGetSystemTimeX();
}

void EKF_predict(void) {
    UWB_canStruct *uwb = can_getUWB();
    PIMUStruct imu = imu_get();

    ekf_vec.obs_z[x_m] = (float) uwb->x_world_cm / 100;
    ekf_vec.obs_z[y_m] = (float) uwb->y_world_cm / 100;
    ekf_vec.obs_z[theta_m] = (float) uwb->theta_world_deg_100 / (100 * 2 * M_PI);

    ekf_vec.input_u[omega_m] = imu->gyroData[Z];
    ekf_vec.input_u[a_mx] = imu->accelData[X];
    ekf_vec.input_u[a_my] = imu->accelData[Y];

    uint32_t t_curr = chVTGetSystemTimeX();
    uint32_t delta_t = ST2S(t_curr - ekf_conf.t_prev);
    ekf_conf.t_prev = t_curr;

    float state_x_mu_bar[STATE_X_NUM];
    float f_mu_t_minus_1_u_t_n_0[STATE_X_NUM];

    f_mu_t_minus_1_u_t_n_0[x_p] = ekf_vec.state_x_mu[x_v];
    f_mu_t_minus_1_u_t_n_0[y_p] = ekf_vec.state_x_mu[y_v];
    f_mu_t_minus_1_u_t_n_0[theta] = ekf_vec.input_u[omega_m] - ekf_vec.state_x_mu[b_g];
    f_mu_t_minus_1_u_t_n_0[x_v] = cos(ekf_vec.state_x_mu[theta]) * (ekf_vec.input_u[a_mx] - ekf_vec.state_x_mu[b_ax]) -
                                  sin(ekf_vec.state_x_mu[theta]) * (ekf_vec.input_u[a_my] - ekf_vec.state_x_mu[b_ay]);
    f_mu_t_minus_1_u_t_n_0[y_v] = sin(ekf_vec.state_x_mu[theta]) * (ekf_vec.input_u[a_mx] - ekf_vec.state_x_mu[b_ax]) +
                                  cos(ekf_vec.state_x_mu[theta]) * (ekf_vec.input_u[a_my] - ekf_vec.state_x_mu[b_ay]);
    f_mu_t_minus_1_u_t_n_0[b_g] = 0;
    f_mu_t_minus_1_u_t_n_0[b_ax] = 0;
    f_mu_t_minus_1_u_t_n_0[b_ay] = 0;

    for (int i = 0; i < STATE_X_NUM; i++)
        state_x_mu_bar[i] = ekf_vec.state_x_mu[i] + delta_t * f_mu_t_minus_1_u_t_n_0[i];

    float x_cov_Sigma_bar[STATE_X_NUM][STATE_X_NUM];


}

void EKF_update(void) {

}