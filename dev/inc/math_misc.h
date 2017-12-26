#ifndef _MATH_MISC_H_
#define _MATH_MISC_H_
#include "hal.h"
#include <math.h>

#define GRAV               9.80665f

#define FLT_EPSILON        1.1920929e-07F
#define M_PI_2_F    (float)(M_PI/2)

/**
 * @source pixhawk/src/lib/mathlib/math/filter/LowPassFilter2p.cpp
 *
 * Data structure for a IIR second-order sections form filter
 * b_0 + b_1 * z^-1 + b_2 * z^-2
 * -----------------------------
 *   1 + a_1 * z^-1 + a_2 * z^-2
 */
typedef struct {
  float a1;
  float a2;
  float b0;
  float b1;
  float b2;

  float data[2];
} lpfilterStruct;

static inline void bound(float* input, const float max)
{
  if(*input > max)
    *input = max;
  else if(*input < -max)
    *input = -max;
}

static inline float boundOutput(const float input, const float max)
{
  float output;
  if(input < max && input > -max)
    output = input;
  else if(input > max)
    output = max;
  else
    output = -max;

  return output;
}

static inline float vector_norm(const float v[], const uint8_t length)
{
  uint8_t i;
  float norm = 0.0f;
  for (i = 0; i < length; i++)
    norm += v[i]*v[i];
  return sqrtf(norm);
}

static inline void vector_normalize(float v[], const uint8_t length)
{
  uint8_t i;
  float norm = vector_norm(v, length);
  for (i = 0; i < length; i++)
    v[i] /= norm;
}

static inline void vector3_cross(const float a[3], const float b[3],
  float result[3])
{
  result[0] = a[1]*b[2] - a[2]*b[1];
  result[1] = a[2]*b[0] - a[0]*b[2];
  result[2] = a[0]*b[1] - a[1]*b[0];
}

static inline float vector3_projection(const float u[3], const float v[3])
{
  return (u[0]*v[0]+u[1]*v[1]+u[2]*v[2])/vector_norm(v,3);
}

static inline float norm_vector3_projection(const float u[3], const float v[3])
{
  float norm = vector_norm(v,3);
  return (u[0]*v[0]+u[1]*v[1]+u[2]*v[2])/(norm*norm);
}


/**
 * @ brief: inverse know square matrix using functions in LAPACK
 * @ dependency:s
 * * LU decomoposition of a general matrix
 * void dgetrf_(int* M, int *N, double* A, int* lda, int* IPIV, int* INFO);
 *
 * * generate inverse of a matrix given its LU decomposition
 * void dgetri_(int* N, double* A, int* lda, int* IPIV, double* WORK, int* lwork, int* INFO);
 */
static inline uint8_t matrix_invert3(const float src[3][3], float dst[3][3])
{
    float det = src[0][0] * (src[1][1] * src[2][2] - src[1][2] * src[2][1]) -
                src[0][1] * (src[1][0] * src[2][2] - src[1][2] * src[2][0]) +
                src[0][2] * (src[1][0] * src[2][1] - src[1][1] * src[2][0]);

    if (fabsf(det) < FLT_EPSILON)
      return false;           // Singular matrix

    dst[0][0] = (src[1][1] * src[2][2] - src[1][2] * src[2][1]) / det;
    dst[1][0] = (src[1][2] * src[2][0] - src[1][0] * src[2][2]) / det;
    dst[2][0] = (src[1][0] * src[2][1] - src[1][1] * src[2][0]) / det;
    dst[0][1] = (src[0][2] * src[2][1] - src[0][1] * src[2][2]) / det;
    dst[1][1] = (src[0][0] * src[2][2] - src[0][2] * src[2][0]) / det;
    dst[2][1] = (src[0][1] * src[2][0] - src[0][0] * src[2][1]) / det;
    dst[0][2] = (src[0][1] * src[1][2] - src[0][2] * src[1][1]) / det;
    dst[1][2] = (src[0][2] * src[1][0] - src[0][0] * src[1][2]) / det;
    dst[2][2] = (src[0][0] * src[1][1] - src[0][1] * src[1][0]) / det;

    return true;
}

static inline void q_derivative(const float q[4], const float v[3],
  float dq[4])
{
  dq[0] = 0.5 * (v[0] * -q[1] + v[1] * -q[2] + v[2] * -q[3]);
  dq[1] = 0.5 * (v[0] *  q[0] + v[1] * -q[3] + v[2] *  q[2]);
  dq[2] = 0.5 * (v[0] *  q[3] + v[1] *  q[0] + v[2] * -q[1]);
  dq[3] = 0.5 * (v[0] * -q[2] + v[1] *  q[1] + v[2] *  q[0]);
}

/**
 * create Euler angles vector from the quaternion
 */
static inline void quarternion2euler(const float q[4], float euler_angle[3])
{
  euler_angle[0] = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]), 1.0f - 2.0f * (q[1] * q[1] + q[2] * q[2]));
  euler_angle[1] = asinf(2.0f * (q[0] * q[2] - q[3] * q[1]));
  euler_angle[2] = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]), 1.0f - 2.0f * (q[2] * q[2] + q[3] * q[3]));
}

/**
 * @source from MatrixMath.cpp from Arduino
 * @brief for known size matrix multiplication, no check
 * A = input matrix (m x p)
 * x = input vector (p x n)
 * m = number of rows in A
 * n = number of columns in A = number of rows in x
 * b = output matrix = A*x
 */
static inline void matrix33_multiply_vector3(const float A[3][3], const float x[3], float b[3])
{
  uint8_t i, j;

  for (i = 0; i < 3; i++)
  {
    b[i] = 0.0f;
    for (j = 0; j < 3; j++)
      b[i] += A[i][j] * x[j];
  }
}

static inline void rotm2quarternion(const float rotm[3][3], float q[4])
{
  float tr = rotm[0][0] + rotm[1][1] + rotm[2][2];

  if (tr > 0.0f) {
    float s = sqrtf(tr + 1.0f);
    q[0] = s * 0.5f;
    s = 0.5f / s;
    q[1] = (rotm[2][1] - rotm[1][2]) * s;
    q[2] = (rotm[0][2] - rotm[2][0]) * s;
    q[3] = (rotm[1][0] - rotm[0][1]) * s;

  } else {
    /* Find maximum diagonal element in dcm
    * store index in dcm_i */
    int dcm_i = 0;

    uint8_t i;
    for (i = 1; i < 3; i++) {
      if (rotm[i][i] > rotm[dcm_i][dcm_i]) {
        dcm_i = i;
      }
    }

    int dcm_j = (dcm_i + 1) % 3;
    int dcm_k = (dcm_i + 2) % 3;
    float s = sqrtf((rotm[dcm_i][dcm_i] - rotm[dcm_j][dcm_j] -
         rotm[dcm_k][dcm_k]) + 1.0f);
    q[dcm_i + 1] = s * 0.5f;
    s = 0.5f / s;
    q[dcm_j + 1] = (rotm[dcm_i][dcm_j] + rotm[dcm_j][dcm_i]) * s;
    q[dcm_k + 1] = (rotm[dcm_k][dcm_i] + rotm[dcm_i][dcm_k]) * s;
    q[0] = (rotm[dcm_k][dcm_j] - rotm[dcm_j][dcm_k]) * s;
  }

  vector_normalize(q,4);
}

static inline void rotm2eulerangle(const float rotm[3][3], float euler_angle[3])
{
  euler_angle[1] = asinf(-rotm[2][0]);

  if (fabsf(euler_angle[1] - M_PI_2_F) < 1.0e-3f) {
    euler_angle[0] = 0.0f;
    euler_angle[2] = atan2f(rotm[1][2] - rotm[0][1], rotm[0][2] + rotm[1][1]) + euler_angle[0];

  } else if (fabsf(euler_angle[1] + M_PI_2_F) < 1.0e-3f) {
    euler_angle[0] = 0.0f;
    euler_angle[2] = atan2f(rotm[1][2] - rotm[0][1], rotm[0][2] + rotm[1][1]) - euler_angle[0];

  } else {
    euler_angle[0] = atan2f(rotm[2][1], rotm[2][2]);
    euler_angle[2] = atan2f(rotm[1][0], rotm[0][0]);
  }
}

void lpfilter_init(lpfilterStruct* const lp,
  const float sample_freq, const float cutoff_freq);

float lpfilter_apply(lpfilterStruct* const lp, const float input);

#endif
