//
// Created by huier on 4/3/18.
//

#ifndef RMDEVBOARD_CHIBIOS_EKF_NEW_H
#define RMDEVBOARD_CHIBIOS_EKF_NEW_H

#include "gsl/gsl_blas.h"
#include "gsl/gsl_linalg.h"
#include "stdint.h"

#define STATE_X_NUM                     8U
#define INPUT_U_NUM                     3U
#define OBS_Z_NUM                       3U
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
    gsl_vector *x_mu;
    gsl_vector *x_mu_bar;
    gsl_matrix *x_cov;
    gsl_matrix *x_cov_bar;
} ekf_state;

typedef struct {
    gsl_matrix *Q_cov;
    gsl_matrix *R_cov;
    gsl_matrix *C;
    uint32_t t_prev; //in system ticks
} ekf_config;

ekf_config* get_ekf_config(void);

ekf_state* get_ekf_state(void);

void init_ekf(void);

void update_ekf(void);

void predict_ekf(void);

#endif RMDEVBOARD_CHIBIOS_EKF_NEW_H