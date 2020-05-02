/*
 * control.h
 *
 *  Created on: 12.07.2018
 *  Author: Stefan Węgrzyn
 */

#ifndef CONTROL_H_
#define CONTROL_H_

// ----------------------------

void DWT_Init();
void delayUS_DWT(uint32_t us);

// ----------------------------

void homing(void);

// ----------------------------

float cnt2rad_su(uint32_t cnt);
#define PI 3.14159f

void swingup(void);

// ----------------------------

// structure containing state
struct __attribute__((packed,aligned(1))) fullState
{
	float theta;
	float omega;
	float x;
	float v;
}extern xk;

struct __attribute__((packed,aligned(1))) state
{
	float theta;
	float x;
}xkm1, xkm2;

// control signal [V]
extern float u;

// structure containing LQR regulator gains
struct __attribute__((packed,aligned(1))) LQRGain
{
	float theta;
	float omega;
	float x;
	float v;
};

// structure containing PID regulator gains
struct __attribute__((packed,aligned(1))) PIDGain
{
	float Kp;
	float Ki;
	float Kd;
};

// PID regulator coefficients calculated according to set values of PID gains
float PID_c1_x, PID_c2_x, PID_c3_x, PID_c1_theta, PID_c2_theta, PID_c3_theta;

// Kalman filter gains for pendulum state estimation
struct __attribute__((packed,aligned(1))) PendKalmanGain
{
	float theta;
	float omega;
};

// Kalman filter gains for cart state estimation
struct __attribute__((packed,aligned(1))) CartKalmanGain
{
	float x;
	float v;
};

// Kalman filter state observer gains
struct __attribute__((packed,aligned(1))) ObserverKalmanGain
{
	float theta;
	float omega;
	float x;
	float v;
};

enum CTR_MODE {
	MODE_NO_CONTROL = 0, MODE_LQR_KALMAN_OBSERVER, MODE_LQR_DUAL_KALMAN, MODE_PID, MODE_EXTERNAL_CONTROL
};

// structure containing configuration parameters
struct __attribute__((packed,aligned(1))) Config
{

	struct LQRGain LQR;
	struct PIDGain PID_theta;
	struct PIDGain PID_x;
	struct PendKalmanGain Kpend;
	struct CartKalmanGain Kcart;
	struct ObserverKalmanGain Kf;
	uint8_t ctrMode;
}extern param;

void state_Init(void);
void param_Init(void);

// define measurements resolution
#define thetaRes 4000.0f	// 4000 [imp] / PI [rad]
#define xRes 128000.0f	// 128000 [imp] / 1 [m]

float cnt2rad(uint32_t cnt);
float cnt2m(uint32_t cnt);

// define max control signal [V]
#define umax 24.0f
// define PWM resolution
#define PWMRes 1000.0f
// define PWM duty cycle offset
#define PWMOffset 110.0f

// declare global 'x' reference value
extern float x_ref;

void LQR(void);

// define sampling period [s]
#define Ts 0.0001f

void PID_Init(void);

void PID(void);

void DualKalman(void);

void KalmanObserver(void);

void PIDMeas(void);

void PendDownMeas(void);

void setCtrSig(void);

#endif /* CONTROL_H_ */
