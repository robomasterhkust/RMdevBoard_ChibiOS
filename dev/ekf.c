//
// Created by huier on 4/3/18.
//

#include <canBusProcess.h>
#include <gsl/gsl_matrix.h>
#include "ekf_new.h"
#include "mpu6500.h"

static ekf_state ekfState;
static ekf_config ekfConfig;

void init_ekf(void) {
    ekfState.x_mu = gsl_vector_alloc(STATE_X_NUM);
    ekfState.x_mu_bar = gsl_vector_alloc(STATE_X_NUM);
    ekfState.x_cov = gsl_matrix_alloc(STATE_X_NUM, STATE_X_NUM);
    ekfState.x_cov_bar = gsl_matrix_alloc(STATE_X_NUM, STATE_X_NUM);
    ekfConfig.Q_cov = gsl_matrix_alloc(PROCESS_NOISE_N_NUM, PROCESS_NOISE_N_NUM);
    ekfConfig.R_cov = gsl_matrix_alloc(MEASUREMENT_NOISE_V_NUM, MEASUREMENT_NOISE_V_NUM);
    ekfConfig.C = gsl_matrix_alloc(OBS_Z_NUM, STATE_X_NUM);

    UWB_canStruct *uwb = can_getUWB();
    PIMUStruct imu = imu_get();

    double q_var = 1.0;
    double r_var = 1.0;
    double x_var = 1.0;

    gsl_vector_set(ekfState.x_mu, x_p, uwb->x_world_cm / 100);
    gsl_vector_set(ekfState.x_mu, y_p, uwb->y_world_cm / 100);
    gsl_vector_set(ekfState.x_mu, theta, uwb->theta_world_deg_100 / (2 * M_PI * 100));
    gsl_vector_set(ekfState.x_mu, x_v, 0);
    gsl_vector_set(ekfState.x_mu, y_v, 0);
    gsl_vector_set(ekfState.x_mu, b_g, imu->_gyroBias[Z]);
    gsl_vector_set(ekfState.x_mu, b_ax, imu->_accelBias[X]);
    gsl_vector_set(ekfState.x_mu, b_ay, imu->_accelBias[Y]);

    gsl_matrix_set_identity(ekfState.x_cov);
    gsl_matrix_scale(ekfState.x_cov, x_var);
    gsl_matrix_set_identity(ekfConfig.R_cov);
    gsl_matrix_scale(ekfConfig.R_cov, r_var);
    gsl_matrix_set_identity(ekfConfig.Q_cov);
    gsl_matrix_scale(ekfConfig.Q_cov, q_var);

    gsl_matrix_set_identity(ekfConfig.C);

    ekfConfig.t_prev = chVTGetSystemTime();
}

void predict_ekf(void) {
    PIMUStruct imu = imu_get();

    gsl_vector *u_input = gsl_vector_alloc(INPUT_U_NUM);
    gsl_vector_set(u_input, omega_m, imu->gyroData[Z]);
    gsl_vector_set(u_input, a_mx, imu->accelData[X]);
    gsl_vector_set(u_input, a_my, imu->accelData[Y]);

    uint32_t t_curr = chVTGetSystemTimeX();
    double delta_t = (double) ST2S(t_curr - ekfConfig.t_prev);
    ekfConfig.t_prev = t_curr;

    gsl_vector *f_temp = gsl_vector_alloc(STATE_X_NUM);

    double temp_theta = gsl_vector_get(ekfState.x_mu, theta);
    double temp_ax = gsl_vector_get(u_input, a_mx) - gsl_vector_get(ekfState.x_mu, b_ax);
    double temp_ay = gsl_vector_get(u_input, a_my) - gsl_vector_get(ekfState.x_mu, b_ay);

    gsl_vector_set(f_temp, x_p, gsl_vector_get(ekfState.x_mu, x_v));
    gsl_vector_set(f_temp, y_p, gsl_vector_get(ekfState.x_mu, y_v));
    gsl_vector_set(f_temp, theta, gsl_vector_get(u_input, omega_m) - gsl_vector_get(ekfState.x_mu, b_g));
    gsl_vector_set(f_temp, x_v, cos(temp_theta) * temp_ax - sin(temp_theta) * temp_ay);
    gsl_vector_set(f_temp, y_v, sin(temp_theta) * temp_ax + sin(temp_theta) * temp_ay);
    gsl_vector_set(f_temp, b_g, 0);
    gsl_vector_set(f_temp, b_ax, 0);
    gsl_vector_set(f_temp, b_ay, 0);

    gsl_vector_scale(f_temp, delta_t);
    gsl_vector_add(f_temp, ekfState.x_mu);
    gsl_vector_memcpy(ekfState.x_mu_bar, f_temp);
    gsl_vector_free(f_temp);

    double F_t_data[] = {1, 0, 0, delta_t, 0, 0, 0, 0,
                         0, 1, 0, 0, delta_t, 0, 0, 0,
                         0, 0, 1, 0, 0, -delta_t, 0, 0,
                         0, 0, (-sin(temp_theta) * temp_ax - cos(temp_theta) * temp_ay) * delta_t, 1, 0, 0,
                         -cos(temp_theta) * delta_t, sin(temp_theta) * delta_t,
                         0, 0, (cos(temp_ax) * temp_ax - sin(temp_theta) * temp_ay) * delta_t, 0, 1, 0,
                         -sin(temp_theta) * delta_t, -cos(temp_theta) * delta_t,
                         0, 0, 0, 0, 0, 1, 0, 0,
                         0, 0, 0, 0, 0, 0, 1, 0,
                         0, 0, 0, 0, 0, 0, 0, 1};
    double V_t_data[] = {0, 0, 0, 0, 0, 0,
                         0, 0, 0, 0, 0, 0,
                         -delta_t, 0, 0, 0, 0, 0,
                         0, -cos(temp_theta) * delta_t, sin(temp_theta) * delta_t, 0, 0, 0,
                         0, -sin(temp_theta) * delta_t, -cos(temp_theta) * delta_t, 0, 0, 0,
                         0, 0, 0, delta_t, 0, 0,
                         0, 0, 0, 0, delta_t, 0,
                         0, 0, 0, 0, 0, delta_t};
    gsl_matrix_view F_t = gsl_matrix_view_array(F_t_data, STATE_X_NUM, STATE_X_NUM);
    gsl_matrix_view V_t = gsl_matrix_view_array(V_t_data, STATE_X_NUM, PROCESS_NOISE_N_NUM);

    gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, &F_t.matrix, ekfState.x_cov, 0.0, ekfState.x_cov_bar);
    gsl_matrix_transpose(&F_t.matrix);
    gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, ekfState.x_cov_bar, &F_t.matrix, 0.0, ekfState.x_cov_bar);

    gsl_matrix *temp = gsl_matrix_alloc(STATE_X_NUM, PROCESS_NOISE_N_NUM);
    gsl_matrix *temp1 = gsl_matrix_alloc(STATE_X_NUM, STATE_X_NUM);
    gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, &V_t.matrix, ekfConfig.Q_cov, 0.0, temp);
    gsl_matrix_transpose(&V_t.matrix);
    gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, temp, &V_t.matrix, 0.0, temp1);
    gsl_matrix_free(temp);

    gsl_matrix_add(ekfState.x_cov_bar, temp1);
    gsl_matrix_free(temp1);
    gsl_vector_free(u_input);

}

void update_ekf(void) {
    gsl_matrix *K_t = gsl_matrix_alloc(STATE_X_NUM, OBS_Z_NUM);
    gsl_matrix *temp = gsl_matrix_alloc(OBS_Z_NUM, STATE_X_NUM);

    gsl_matrix *temp1 = gsl_matrix_alloc(OBS_Z_NUM, OBS_Z_NUM);
    gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, ekfConfig.C, ekfState.x_cov_bar, 0.0, temp);
    gsl_matrix_transpose(ekfConfig.C);
    gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, temp, ekfConfig.C, 0.0, temp1);
    gsl_matrix_transpose(ekfConfig.C);
    gsl_matrix_add(temp1, ekfConfig.R_cov);
    gsl_matrix *inv_temp1 = gsl_matrix_alloc(OBS_Z_NUM, OBS_Z_NUM);
    gsl_permutation *p = gsl_permutation_alloc(3);
    int s;
    gsl_linalg_LU_decomp(temp1, p, &s);
    gsl_linalg_LU_invert(temp1, p, inv_temp1);
    gsl_permutation_free(p);
    gsl_matrix_free(temp1);

    gsl_matrix_transpose(temp);
    gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, temp, inv_temp1, 0.0, K_t);
    gsl_matrix_transpose(temp);

    gsl_matrix_scale(temp, -1);
    gsl_matrix *temp2 = gsl_matrix_alloc(STATE_X_NUM, STATE_X_NUM);
    gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, K_t, temp, 0.0, temp2);
    gsl_matrix_free(temp);
    gsl_matrix_add(temp2, ekfState.x_cov_bar);
    gsl_matrix_memcpy(ekfState.x_cov, temp2);
    gsl_matrix_free(temp2);

    UWB_canStruct *uwb = can_getUWB();
    gsl_vector *z_obs = gsl_vector_alloc(OBS_Z_NUM);
    gsl_vector_set(z_obs, x_m, uwb->x_world_cm / 100);
    gsl_vector_set(z_obs, y_m, uwb->y_world_cm / 100);
    gsl_vector_set(z_obs, theta_m, uwb->theta_world_deg_100 / (2 * M_PI * 100));
    gsl_vector_view g_temp = gsl_vector_subvector(ekfState.x_mu_bar, 0, OBS_Z_NUM);
    gsl_vector_sub(z_obs, &g_temp.vector);
    gsl_blas_dgemv(CblasNoTrans, 1.0, K_t, z_obs, 1.0, ekfState.x_mu_bar);
    gsl_vector_free(z_obs);
    gsl_vector_memcpy(ekfState.x_mu, ekfState.x_mu_bar);
}