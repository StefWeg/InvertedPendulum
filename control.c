/*
 * control.c
 *
 *  Created on: 12.07.2018
 *  Author: Stefan Węgrzyn
 */

#include "stm32l476xx.h"
#include "control.h"


void DWT_Init()
{
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk << DWT_CTRL_CYCCNTENA_Pos;	// enable the counter
}

#pragma GCC push_options
#pragma GCC optimize("03")
void delayUS_DWT(uint32_t us)
{
	volatile uint32_t cycles = (SystemCoreClock/1000000L)*us;
	volatile uint32_t start = DWT->CYCCNT;
	do {
	} while(DWT->CYCCNT - start < cycles);
}
#pragma GCC pop_options

/*
 * Homing implementation
 * - Move cart in limit-switch direction
 * - Zero 'x' counter on limit switch
 * - Wait for pendulum to stop swinging
 * - Prepare 'theta' counter for swing-up
 * Function uses 'delayUS_DWT' function and requires executing 'DWT_Init' before use
 */
void homing(void)
{
	TIM1->CCR1 = 250;								// PWM duty cycle set to 250/1000 (output PA8 - D7)
	GPIOA->ODR &= ~(uint32_t)(GPIO_ODR_OD9);		// Direction pin INA set to low (output PA9 - D8)
	GPIOA->ODR |= GPIO_ODR_OD10;					// Direction pin INB set to high (output PA10 - D2)

	while(GPIOB->IDR &= GPIO_IDR_ID9);	// PB9 (D14) is kept pulled-up until limit switch is reached and pin is grounded

	TIM1->CCR1 = 0; 			// PWM duty cycle set to 0/1000 (output PA8 - D7)

	volatile uint32_t prevRead = 0xFFFF;	// variable containing last 'theta' reading
	volatile uint32_t prevRead2 = 0xFFFF;	// variable containing last 'theta' reading
	do
	{
		prevRead2 = prevRead;
		prevRead = TIM3->CNT;
		delayUS_DWT(300000L);			// wait 0.3 [s]
	} while (!((TIM3->CNT == prevRead) && (prevRead == prevRead2)));	// wait until pendulum stops swinging

	TIM3->CNT = 4000;			// TIM3 counter ('theta') set to 4000 (4000/8000 = pi [rad]) [prepared for swing-up]
	TIM2->CNT = 0; 				// TIM2 counter ('x') set to 0 (0/256000 = -1 [m])
}

/*
 * Function converts 'theta' counter reading to value in radians (range 0 <-> 2pi)
 * [VERSION FOR SWINGUP ALGORITHM]
 */
float cnt2rad_su(uint32_t cnt)
{
	return (float)(cnt*PI/thetaRes);	// cnt * PI / 4000
}

/*
 * Pendulum swing-up implementation
 * - Move pendulum from the limit switch position
 * - Stop cart on -0.5m from center
 * - Wait for pendulum to stop swinging
 * - Execute swing-up algorithm until pendulum less then 15 deg from upright position
 * Function internally uses 'cnt2rad_su' function
 * Function uses 'delayUS_DWT' function and requires executing 'DWT_Init' before use
 */
void swingup(void)
{
	GPIOA->ODR |= GPIO_ODR_OD9;						// Direction pin INA set to high (output PA9 - D8)
	GPIOA->ODR &= ~(uint32_t)(GPIO_ODR_OD10);		// Direction pin INB set to low (output PA10 - D2)

	TIM1->CCR1 = 1000;	// PWM duty cycle set to 1000/1000 [100%] (output PA8 - D7)

	while(TIM2->CNT < 90000);

	GPIOA->ODR &= ~(uint32_t)(GPIO_ODR_OD9);		// Direction pin INA set to low (output PA9 - D8)
	GPIOA->ODR |= GPIO_ODR_OD10;					// Direction pin INB set to high (output PA10 - D2)

	TIM1->CCR1 = 500;	// PWM duty cycle set to 500/1000 [50%] (output PA8 - D7)

	float theta = cnt2rad_su(TIM3->CNT);	// first 'theta' reading
	float x = cnt2m(TIM2->CNT);				// first 'x' reading

	// apply swing-up algorithm unless pendulum is 15 [deg] from upright position
	while((theta>(PI/12.0f)) && (theta<(23*PI/12.0f)))
	{
		while (!((TIM3->CR1 & TIM_CR1_DIR) && (theta <= 21.0f*PI/18.0f))) // react 30 [deg] before
		{
			theta = cnt2rad_su(TIM3->CNT);
			x = cnt2m(TIM2->CNT);
			if (!(theta>(PI/12.0f)) && (theta<(23*PI/12.0f))) break;	// procedure termination condition
			if ((x < -0.60) || (x > 0.60)) break;						// soft stop
		}
		if (!(theta>(PI/12.0f)) && (theta<(23*PI/12.0f))) break;		// repeat to leave procedure loop
		if ((x < -0.60) || (x > 0.60))									// repeat to leave procedure loop
		{
			GPIOA->ODR &= ~(uint32_t)(GPIO_ODR_OD9);		// Direction pin INA set to low (output PA9 - D8)
			GPIOA->ODR &= ~(uint32_t)(GPIO_ODR_OD10);		// Direction pin INB set to low (output PA10 - D2)
			break;
		}

		GPIOA->ODR &= ~(uint32_t)(GPIO_ODR_OD9);			// Direction pin INA set to low (output PA9 - D8)
		GPIOA->ODR |= GPIO_ODR_OD10;						// Direction pin INB set to high (output PA10 - D2)

		delayUS_DWT(300000L);

		GPIOA->ODR &= ~(uint32_t)(GPIO_ODR_OD9);			// Direction pin INA set to low (output PA9 - D8)
		GPIOA->ODR &= ~(uint32_t)(GPIO_ODR_OD10);			// Direction pin INB set to low (output PA10 - D2)

		while (!((!(TIM3->CR1 & TIM_CR1_DIR)) && (theta >= 15.0f*PI/18.0f)))	// react 30 [deg] before
		{
			theta = cnt2rad_su(TIM3->CNT);
			x = cnt2m(TIM2->CNT);
			if (!(theta>(PI/12.0f)) && (theta<(23*PI/12.0f))) break;	// procedure termination condition
			if ((x < -0.60) || (x > 0.60)) break;						// soft stop
		}
		if (!(theta>(PI/12.0f)) && (theta<(23*PI/12.0f))) break;		// repeat to leave procedure loop
		if ((x < -0.60) || (x > 0.60))									// repeat to leave procedure loop
		{
			GPIOA->ODR &= ~(uint32_t)(GPIO_ODR_OD9);		// Direction pin INA set to low (output PA9 - D8)
			GPIOA->ODR &= ~(uint32_t)(GPIO_ODR_OD10);		// Direction pin INB set to low (output PA10 - D2)
			break;
		}

		GPIOA->ODR |= GPIO_ODR_OD9;							// Direction pin INA set to high (output PA9 - D8)
		GPIOA->ODR &= ~(uint32_t)(GPIO_ODR_OD10);			// Direction pin INB set to low (output PA10 - D2)

		delayUS_DWT(300000L);

		GPIOA->ODR &= ~(uint32_t)(GPIO_ODR_OD9);			// Direction pin INA set to low (output PA9 - D8)
		GPIOA->ODR &= ~(uint32_t)(GPIO_ODR_OD10);			// Direction pin INB set to low (output PA10 - D2)
	}

	if (theta <= (PI/12.0f)) {TIM3->CNT = TIM3->CNT + 4000;}		// change zero point for angle measurement (0 in down position)
	else {TIM3->CNT = TIM3->CNT - 4000;}
}

/*
 * Global 'xk', 'xkm1' and 'xkm2' structures initialization
 * Global 'x_ref' value initialization
 */
void state_Init(void)
{
	xk.theta = 0;
	xk.omega = 0;
	xk.x = 0;
	xk.v = 0;

	xkm1.theta = 0;
	xkm1.x = 0;

	xkm2.theta = 0;
	xkm2.x = 0;

	x_ref = 0;
}

/*
 * Global 'param' structure initialization
 */
void param_Init(void)
{
	param.LQR.theta = -124.9717;
	param.LQR.omega = -27.8199;
	param.LQR.x = -10.0000;
	param.LQR.v = -39.9853;

	param.PID_theta.Kp = -87;
	param.PID_theta.Ki = -273;
	param.PID_theta.Kd = 0;

	param.PID_x.Kp = -3.5;
	param.PID_x.Ki = -0.5;
	param.PID_x.Kd = 0;

	param.Kpend.theta = 0.0303;
	param.Kpend.omega = 4.6075;

	param.Kcart.x = 0.2191;
	param.Kcart.v = 270.5957;

	param.Kf.theta = -378.7497;
	param.Kf.omega = -489.9568;
	param.Kf.x = 0.9995;
	param.Kf.v = 47.6313;

	param.ctrMode = MODE_EXTERNAL_CONTROL;
}

/*
 * Function converts 'theta' counter reading to value in radians (range -pi <-> pi [rad])
 */
float cnt2rad(uint32_t cnt)
{
	return (float)((cnt*PI/thetaRes)-PI);	// (cnt * PI / 4000) - PI
}

/*
 * Function converts 'x' counter reading to value in meters (range -1 <-> 1 [m])
 */
float cnt2m(uint32_t cnt)
{
	return (float)((cnt/xRes)-0.67);		// (cnt * 1 / 128000) - 1
}

/*
 * Function returns value of LQR control signal in volts [V]
 */
void LQR(void)
{
	u = -param.LQR.theta*xk.theta -param.LQR.omega*xk.omega -param.LQR.x*(xk.x-x_ref) -param.LQR.v*xk.v;
	if (u > umax) u = umax;				// control signal restriction
	else if (u < -umax) u = -umax;		// (needed for proper estimation in 'MODE_LQR_KALMAN_OBSERVER' mode)
}

/*
 * Function calculates PID regulator coefficients according to set values of PID gains
 */
void PID_Init(void)
{
	PID_c1_x = param.PID_x.Kp + param.PID_x.Ki*Ts + param.PID_x.Kd/Ts;
	PID_c2_x = - param.PID_x.Kp - 2*param.PID_x.Kd/Ts;
	PID_c3_x = param.PID_x.Kd/Ts;

	PID_c1_theta = param.PID_theta.Kp + param.PID_theta.Ki*Ts + param.PID_theta.Kd/Ts;
	PID_c2_theta = - param.PID_theta.Kp - 2*param.PID_theta.Kd/Ts;
	PID_c3_theta = param.PID_theta.Kd/Ts;
}

/*
 * Function returns value of PID control signal in volts [V]
 */
void PID(void)
{
	volatile static float ukm1 = 0;		// previous control signal
	volatile float dux = PID_c1_x*(x_ref-xk.x) + PID_c2_x*(x_ref-xkm1.x) + PID_c3_x*(x_ref-xkm2.x);
	volatile float dut = PID_c1_theta*(-xk.theta) + PID_c2_theta*(-xkm1.theta) + PID_c3_theta*(-xkm2.theta);
	u = ukm1 + dux + dut;
	if (u > umax) u = umax;				// anti wind-up
	else if (u < -umax) u = -umax;
	ukm1 = u;							// prepare 'u' for next function execution
}

/*
 * Function implements dual Kalman filter algorithm for pendulum and cart separately
 * Function changes values of the global variables xk, xkm1, xkm2
 */
void DualKalman(void)
{
	volatile static float est_theta = 0, est_omega = 0, est_x = 0, est_v = 0;

	volatile float priori_theta = est_theta + Ts*est_omega;
	volatile float priori_omega = est_omega;
	volatile float priori_x = est_x + Ts*est_v;
	volatile float priori_v = est_v;

	volatile float e_theta = cnt2rad(TIM3->CNT) - priori_theta;
	volatile float e_x = cnt2m(TIM2->CNT) - priori_x;

	est_theta = priori_theta + param.Kpend.theta*e_theta;
	est_omega = priori_omega + param.Kpend.omega*e_theta;
	est_x = priori_x + param.Kcart.x*e_x;
	est_v = priori_v + param.Kcart.v*e_x;

	xk.theta = est_theta;
	xk.omega = est_omega;
	xk.x = est_x;
	xk.v = est_v;
}

/*
 * Function implements single Kalman filter algorithm for pendulum on cart
 * Function changes values of the global variables xk, xkm1, xkm2
 */
void KalmanObserver(void)
{
	volatile static float est_theta = 0, est_omega = 0, est_x = 0, est_v = 0;

	volatile float priori_theta = est_theta + Ts*est_omega;
	volatile float priori_omega = 0.0027f*est_theta + est_omega + 0.0026f*est_v - 0.00015053f*u;
	volatile float priori_x = est_x + Ts*est_v;
	volatile float priori_v = -0.00042631f*est_theta + 0.9986f*est_v + 0.000079207f*u;

	volatile float e_x = cnt2m(TIM2->CNT) - priori_x;

	est_theta = priori_theta + param.Kf.theta*e_x;
	est_omega = priori_omega + param.Kf.omega*e_x;
	est_x = priori_x + param.Kf.x*e_x;
	est_v = priori_v + param.Kf.v*e_x;

	xk.theta = est_theta;
	xk.omega = est_omega;
	xk.x = est_x;
	xk.v = est_v;
}

/*
 * Function determines the values of 'theta' and 'x' based only on measurements
 * Function changes values of the global variables xk, xkm1, xkm2
 */
void PIDMeas(void)
{
	xkm2.theta = xkm1.theta;
	xkm2.x = xkm1.x;

	xkm1.theta = xk.theta;
	xkm1.x = xk.x;

	xk.theta = cnt2rad(TIM3->CNT);
	xk.x = cnt2m(TIM2->CNT);
}

/*
 * Function determines the values of 'theta' and 'x' based only on measurements
 * Function calculates 'theta' for pendulum directed down
 */
void PendDownMeas(void)
{
	xk.theta = cnt2rad_su(TIM3->CNT) - PI;
	xk.x = cnt2m(TIM2->CNT);
}

/*
 * Function sets PWM duty cycle and DIR pin according to given control signal [V]
 */
void setCtrSig(void)
{
	// set PWM duty cycle (output PA8 - D7)
	if ((u > 0.0f) && (u < umax)) {TIM1->CCR1 = (uint32_t)((u/umax)*(PWMRes-PWMOffset) + PWMOffset);}
	else if ((u < 0.0f) && (u > -umax)) {TIM1->CCR1 = (uint32_t)((-u/umax)*(PWMRes-PWMOffset) + PWMOffset);}
	else if (u == 0.0f) {TIM1->CCR1 = 0;}						// set PWM duty cycle to 0 (output PA8 - D7)
	else TIM1->CCR1 = (uint32_t)(PWMRes-PWMOffset) + PWMOffset;	// set maximum PWM duty cycle (output PA8 - D7)
	if (u >= 0)
	{
		GPIOA->ODR |= GPIO_ODR_OD9;						// Direction pin INA set to high (output PA9 - D8)
		GPIOA->ODR &= ~(uint32_t)(GPIO_ODR_OD10);		// Direction pin INB set to low (output PA10 - D2)
	}
	else
	{
		GPIOA->ODR &= ~(uint32_t)(GPIO_ODR_OD9);		// Direction pin INA set to low (output PA9 - D8)
		GPIOA->ODR |= GPIO_ODR_OD10;					// Direction pin INB set to high (output PA10 - D2)
	}
}
