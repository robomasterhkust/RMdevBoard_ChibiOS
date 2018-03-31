//
// Created by Yang Shaohui on 2018.03.31.
//

#include "stdint.h"

#ifndef RMDEVBOARD_CHIBIOS_EKF_H
#define RMDEVBOARD_CHIBIOS_EKF_H



#define STATE_X_NUM                     8U
#define input_u_NUM                     3U
#define obs_z_NUM                       3U
#define PROCESS_NOISE_N_NUM             6U
#define MEASUREMENT_NOISE_V_NUM         3U

typedef enum {
    x_p = 0,
    y_p = 1,
    theta = 2,
    x_v = 3,
    y_v = 4,
    b_g = 5,
    b_ax = 6,
    b_ay = 7
} x_state_index;

typedef enum {
    n_g = 0,
    n_ax = 1,
    n_ay = 2,
    n_bg = 3,
    n_bax = 4,
    n_bay = 5
} n_pro_noise_index;

typedef enum {
    x_m = 0,
    y_m = 1,
    theta_m = 2
} z_observe_index;

typedef enum {
    omega_m = 0,
    a_mx = 1,
    a_my = 2
} u_input_index;

typedef enum {
    n_x = 0,
    n_y = 1,
    n_theta = 2
} v_obs_noise_index;

typedef struct {
    float state_x_mu[STATE_X_NUM];
    // x_m, y_m, theta_rad, x_vel_m_s, y_vel_m_s, bias_gyro_rad_s, bias_acc_x_m_s2, bias_acc_y_m_s2
    float input_u[input_u_NUM];
    // omega_measure, acc_measure_x, acc_measure_y
    float obs_z[obs_z_NUM];
    // x_measure, y_measure, theta_measure_rad
} EKF_vec;

typedef struct {
    float n_cov_Q_const[PROCESS_NOISE_N_NUM][PROCESS_NOISE_N_NUM];
    float v_cov_R_const[MEASUREMENT_NOISE_V_NUM][MEASUREMENT_NOISE_V_NUM];
    float A_t[STATE_X_NUM][STATE_X_NUM];
    float U_t[PROCESS_NOISE_N_NUM][PROCESS_NOISE_N_NUM];
    float x_cov_Sigma[STATE_X_NUM][STATE_X_NUM];
} EKF_mat;

typedef struct {
    uint32_t t_prev; // in system ticks

} EKF_conf;

EKF_vec *get_ekf_vec(void);

EKF_mat *get_ekf_mat(void);

EKF_conf *get_ekf_conf(void);

void EKF_init(void);

void EKF_predict(void);

void EKF_update(void);

#endif RMDEVBOARD_CHIBIOS_EKF_H