/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l4xx_hal.h"

/* USER CODE BEGIN Includes */
#include "control.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
static void NVIC_Init(void);

static void Encoder_Init(void);

static void limitSwitch_Init(void);

static void DMA_Init(void);
static void USART_Init(void);

static void PWM_Init(void);

static void ControlLoop_Init(void);

static void MeasOutput_Init(void);

// structure containing configuration parameters (defined in 'control.h')
struct Config param;

// structure containing measurement data
struct __attribute__((packed,aligned(1))) Meas
{
	float theta;
	float x;
	float u;
} measVal;
// packed: use as little memory as possible, aligned(1): align to 1 byte

// global 'x' reference value (declared in 'control.h')
float x_ref;
// global structure containing current state (declared in 'control.h')
struct fullState xk;
// global control signal value [V] (declared in 'control.h')
float u;

// re-initialization flag
struct flags
{
	uint8_t paramUpload : 1;		// parameters upload
	uint8_t reInit : 1;				// initialization
	uint8_t impulseTestStart : 1;	// impulse test
	uint8_t impulseTestEnd : 1;		// impulse test end
	uint8_t disturbanceSim : 1;		// disturbance simulation
	uint8_t refChange : 1;			// indicates reference value change during measurements output
};

struct flags status;

//	variable storing control mode setting (used to restore setting after turning off control)
uint8_t savedCtrMode = MODE_NO_CONTROL;


/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  DWT_Init();
  Encoder_Init();
  limitSwitch_Init();
  DMA_Init();
  USART_Init();
  PWM_Init();
  ControlLoop_Init();
  MeasOutput_Init();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();

  /* USER CODE BEGIN 2 */
  status.paramUpload = 0;
  status.reInit = 0;
  status.impulseTestStart = 0;
  status.impulseTestEnd = 0;
  status.refChange = 0;

  state_Init();
  param_Init();
  PID_Init();

  NVIC_Init();

  homing();
  swingup();

  NVIC_EnableIRQ(TIM4_IRQn);				// enable control interrupts
  param.ctrMode = MODE_LQR_DUAL_KALMAN;		// default control mode
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

	  if(status.reInit != 0)				// flag set via USART2 communication
	  {
		  status.reInit = 0;				// re-initialization flag reset

		  USART2->CR1 &= ~(uint32_t)(USART_CR1_RXNEIE);	// RXNE (read register not empty) interrupt disabled

		  u = 0;
		  if(param.ctrMode == MODE_NO_CONTROL) param.ctrMode = MODE_LQR_KALMAN_OBSERVER;	// avoid situation when pendulum is not controlled after swing-up
		  state_Init();						// initialize state (leave parameters set earlier)
		  homing();							// conduct cart & pendulum homing
		  swingup();						// conduct pendulum swing-up

		  NVIC_EnableIRQ(TIM4_IRQn);		// enable control loop
		  USART2->CR1 |= USART_CR1_RXNEIE;	// RXNE (read register not empty) interrupt enabled
	  }
	  else if(status.impulseTestStart != 0)		// flag set via USART2 communication
	  {
		  status.impulseTestStart = 0;			// impulse test flag reset

		  // homing
		  state_Init();							// initialize state (leave parameters set earlier)
		  homing();								// conduct cart & pendulum homing

		  // start sending measurements (enable DMA1 channel 7 (USART2_TX))
		  USART2->CR3 |= USART_CR3_DMAT;						// DMA mode enabled for transmission (sending requests)
		  DMA1_Channel7->CCR &= ~(uint32_t)(DMA_CCR_EN);		// disable DMA1 CH7 (USART2_TX)
		  DMA1_Channel7->CMAR = (uint32_t)&(measVal);			// "Meas" structure instance
		  DMA1_Channel7->CCR &= ~(uint32_t)(DMA_CCR_TCIE);		// transfer complete interrupt disabled
		  DMA1_Channel7->CCR |= DMA_CCR_EN;						// activate DMA1 CH7 (USART2_TX)

		  // turn on control loop to enable measurements
		  PendDownMeas();
		  u = 0;
		  savedCtrMode = param.ctrMode;							// save control mode to recover it later
		  param.ctrMode = MODE_EXTERNAL_CONTROL;				// set external control mode ('u' changed manually)

		  // enable interrupts
		  NVIC_EnableIRQ(TIM5_IRQn);							// enable TIM5 update interrupt (meas loop)
		  NVIC_EnableIRQ(TIM4_IRQn);							// enable control loop

		  // generate impulse
		  delayUS_DWT(1000000L);							// wait 1 [s] before impulse generation

		  u = 24;
		  setCtrSig();											// set max control signal

		  delayUS_DWT(300000L);								// wait 0.3 [s] (impulse duration)

		  u = 0;
		  setCtrSig();											// disable control signal

		  // impulse test handling finished in TIM5_IRQn
		  status.impulseTestEnd = 1;							// impulse test end flag set
	  }
	  else if(status.disturbanceSim != 0)		// flag set via USART2 communication
	  {
		  status.disturbanceSim = 0;			// disturbance simulation flag reset ("trolley hit")

		  savedCtrMode = param.ctrMode;					// save control mode to recover it later
		  param.ctrMode = MODE_EXTERNAL_CONTROL;		// set external control mode ('u' changed manually)

		  // generate impulse
		  u = 24;
		  setCtrSig();									// set max control signal

		  delayUS_DWT(100000L);							// wait 0.1 [s] (impulse duration)

		  param.ctrMode = savedCtrMode;			// restore control mode setting (system recovers from distortion)
	  }

	  // soft stop implementation
	  if ((xk.x < -0.60) || (xk.x > 0.60))
	  {
		  if (param.ctrMode != MODE_NO_CONTROL) savedCtrMode = param.ctrMode;	// save control mode setting
		  param.ctrMode = MODE_NO_CONTROL;										// disable control
	  }
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the main internal regulator output voltage 
    */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
     PA2   ------> USART2_TX
     PA3   ------> USART2_RX
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USART_TX_Pin USART_RX_Pin */
  GPIO_InitStruct.Pin = USART_TX_Pin|USART_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void USART2_IRQHandler(void)
{
	if (USART2->ISR & USART_ISR_RXNE)	// if RXNE (read register not empty) flag caused interrupt
	{
		uint8_t cmd = USART2->RDR;			// read received data
		USART2->RQR |= USART_RQR_RXFRQ;		// clear RXNE (read register not empty) flag

		if (cmd == 'r')				// read config parameters
		{
			USART2->CR1 &= ~(uint32_t)(USART_CR1_RXNEIE);	// RXNE (read register not empty) interrupt disabled

			// enable DMA1 channel 7 (USART2_TX)
			USART2->CR3 |= USART_CR3_DMAT;						// DMA mode enabled for transmission (sending requests)
			USART2->ICR |= USART_ICR_TCCF;						// clear TC (transmission complete) flag
			DMA1_Channel7->CCR &= ~(uint32_t)(DMA_CCR_EN);		// disable DMA1 CH7 (USART2_TX)
			DMA1_Channel7->CMAR = (uint32_t)&(param);			// "Config" structure instance
			DMA1_Channel7->CNDTR = sizeof(struct Config);		// set number of data to transfer
			DMA1_Channel7->CCR |= DMA_CCR_EN;					// activate DMA1 CH7 (USART2_TX)

			DMA1_Channel7->CCR |= DMA_CCR_TCIE;				// transfer complete interrupt enabled
		}

		else if (cmd == 'w')		// write config parameters
		{
			USART2->CR1 &= ~(uint32_t)(USART_CR1_RXNEIE);	// RXNE (read register not empty) interrupt disabled

			status.paramUpload = 1;							// set parameters upload flag

			// enable DMA1 channel 6 (USART2_RX)
			USART2->CR3 |= USART_CR3_DMAR;						// DMA mode enabled for reception (sending requests)
			DMA1_Channel6->CCR &= ~(uint32_t)(DMA_CCR_EN);		// disable DMA1 CH6 (USART2_RX)
			DMA1_Channel6->CMAR = (uint32_t)&(param);			// "Config" structure instance
			DMA1_Channel6->CNDTR = sizeof(struct Config);		// set number of data to transfer
			DMA1_Channel6->CCR |= DMA_CCR_EN;					// activate DMA1 CH6 (USART2_RX)

			DMA1_Channel6->CCR |= DMA_CCR_TCIE;				// transfer complete interrupt enabled
		}

		if (cmd == 'g')				// get 'x' reference value
		{
			USART2->CR1 &= ~(uint32_t)(USART_CR1_RXNEIE);	// RXNE (read register not empty) interrupt disabled

			// enable DMA1 channel 7 (USART2_TX)
			USART2->CR3 |= USART_CR3_DMAT;						// DMA mode enabled for transmission (sending requests)
			USART2->ICR |= USART_ICR_TCCF;						// clear TC (transmission complete) flag
			DMA1_Channel7->CCR &= ~(uint32_t)(DMA_CCR_EN);		// disable DMA1 CH7 (USART2_TX)
			DMA1_Channel7->CMAR = (uint32_t)&(x_ref);			// 'x' reference float instance
			DMA1_Channel7->CNDTR = sizeof(float);				// set number of data to transfer
			DMA1_Channel7->CCR |= DMA_CCR_EN;					// activate DMA1 CH7 (USART2_TX)

			DMA1_Channel7->CCR |= DMA_CCR_TCIE;				// transfer complete interrupt enabled
		}

		else if (cmd == 's')		// set 'x' reference value
		{
			USART2->CR1 &= ~(uint32_t)(USART_CR1_RXNEIE);	// RXNE (read register not empty) interrupt disabled

			// enable DMA1 channel 6 (USART2_RX)
			USART2->CR3 |= USART_CR3_DMAR;						// DMA mode enabled for reception (sending requests)
			DMA1_Channel6->CCR &= ~(uint32_t)(DMA_CCR_EN);		// disable DMA1 CH6 (USART2_RX)
			DMA1_Channel6->CMAR = (uint32_t)&(x_ref);			// 'x' reference float instance
			DMA1_Channel6->CNDTR = sizeof(float);				// set number of data to transfer
			DMA1_Channel6->CCR |= DMA_CCR_EN;					// activate DMA1 CH6 (USART2_RX)

			DMA1_Channel6->CCR |= DMA_CCR_TCIE;				// transfer complete interrupt enabled
		}

		else if (cmd == 'm')		// start sending measurement data
		{

			// enable DMA1 channel 7 (USART2_TX)
			USART2->CR3 |= USART_CR3_DMAT;						// DMA mode enabled for transmission (sending requests)
			DMA1_Channel7->CCR &= ~(uint32_t)(DMA_CCR_EN);		// disable DMA1 CH7 (USART2_TX)
			DMA1_Channel7->CMAR = (uint32_t)&(measVal);			// "Meas" structure instance
			DMA1_Channel7->CCR &= ~(uint32_t)(DMA_CCR_TCIE);	// transfer complete interrupt disabled
			DMA1_Channel7->CCR |= DMA_CCR_EN;					// activate DMA1 CH7 (USART2_TX)

			USART2->CR1 &= ~(uint32_t)(USART_CR1_RXNEIE);		// RXNE (read register not empty) interrupt disabled

			NVIC_EnableIRQ(TIM5_IRQn);							// enable TIM5 update interrupt
		}

		else if (cmd == 'i')		// reinitialize pendulum (swing-up)
		{
			NVIC_DisableIRQ(TIM4_IRQn);							// disable control loop
			USART2->CR1 &= ~(uint32_t)(USART_CR1_RXNEIE);		// RXNE (read register not empty) interrupt disabled
			status.reInit = 1;									// re-initialization flag set
		}

		else if (cmd == 't')		// impulse test identification
		{
			NVIC_DisableIRQ(TIM4_IRQn);							// disable control loop
			USART2->CR1 &= ~(uint32_t)(USART_CR1_RXNEIE);		// RXNE (read register not empty) interrupt disabled
			status.impulseTestStart = 1;						// impulse test flag set
		}

		else if (cmd == 'd')		// disable control
		{
			if (param.ctrMode != MODE_NO_CONTROL) savedCtrMode = param.ctrMode;		// save control mode setting
			param.ctrMode = MODE_NO_CONTROL;										// disable control
		}

		else if (cmd == 'e')		// enable control
		{
			if (savedCtrMode != MODE_NO_CONTROL) param.ctrMode = savedCtrMode;		// restore control mode setting
			else param.ctrMode = MODE_LQR_DUAL_KALMAN;
		}

		else if (cmd == 'h')		// trolley "hit" imitation
		{
			status.disturbanceSim = 1;		// disturbance simulation flag set (conducted in main loop)
		}

	}
}

void DMA1_Channel7_IRQHandler(void)
{
	if (DMA1->ISR & DMA_ISR_TCIF7)			// if RXNE TC (transfer complete) flag caused interrupt
	{
		DMA1->IFCR |= DMA_IFCR_CTCIF7;				// transfer complete flag clear for channel 7
		USART2->CR3 &= ~(uint32_t)(USART_CR3_DMAT);	// DMA mode disabled for transmission (stop sending DMA requests)
		USART2->CR1 |= USART_CR1_RXNEIE;			// RXNE (read register not empty) interrupt enabled
	}
}

void DMA1_Channel6_IRQHandler(void)
{
	if (DMA1->ISR & DMA_ISR_TCIF6)			// if RXNE TC (transfer complete) flag caused interrupt
	{
		DMA1->IFCR |= DMA_IFCR_CTCIF6;								// transfer complete flag clear for channel 6
		USART2->CR3 &= ~(uint32_t)(USART_CR3_DMAR);					// DMA mode disabled for reception (stop sending DMA requests)
		if (status.paramUpload != 0)				// parameters upload finished
		{
			status.paramUpload = 0;									// reset parameters upload flag
			PID_Init();												// recalculate PID gains for new given parameters
		}
		if (status.refChange == 0) USART2->CR1 |= USART_CR1_RXNEIE;	// RXNE (read register not empty) interrupt enabled
		else
		{
			status.refChange = 0;									// reset reference value change flag if set earlier
			USART2->RQR |= USART_RQR_RXFRQ;							// clear RXNE (read register not empty) flag
		}
	}
}

void TIM5_IRQHandler(void)				// every 10 ms
{
	if (TIM5->SR & TIM_SR_UIF)			// if UIF update flag caused interrupt
	{
		TIM5->SR &= ~(uint32_t)(TIM_SR_UIF);	// clear interrupt flag

		/* data processing on PC side
		measVal.x = TIM2->CNT; 		// TIM2 counter (x/256000)
		measVal.theta = TIM3->CNT; 	// TIM3 counter (x/8000)
		measVal.PWM = TIM1->CCR1;	// TIM1 capture/compare CH1 (x/1000 PWM duty cycle)
		*/

		measVal.theta = xk.theta; 	// current 'theta' value
		measVal.x = xk.x; 			// current 'x' value
		measVal.u = u;				// current control signal 'u'

		// enable DMA1 channel 7 (USART2_TX)
		USART2->ICR |= USART_ICR_TCCF;						// clear TC (transmission complete) flag
		DMA1_Channel7->CCR &= ~(uint32_t)(DMA_CCR_EN);		// disable DMA1 CH7 (USART2_TX)
		DMA1_Channel7->CNDTR = sizeof(struct Meas);			// set number of data to transfer
		DMA1_Channel7->CCR |= DMA_CCR_EN;					// activate DMA1 CH7 (USART2_TX)

		if((USART2->ISR & USART_ISR_RXNE) && (status.refChange == 0))
		{
			uint8_t cmd = USART2->RDR;

			if(cmd == 'f') 					// finish measurement output
			{
				NVIC_DisableIRQ(TIM5_IRQn);						// disable TIM5 update interrupt

				// impulse test finish (force re-initialization)
				if(status.impulseTestEnd != 0)	// flag set via USART2 communication
				{
					status.impulseTestEnd = 0;			// impulse test end flag reset

					NVIC_DisableIRQ(TIM4_IRQn);			// disable control loop
					param.ctrMode = savedCtrMode;		// restore control mode setting

					status.reInit = 1;				// re-initialization flag set
				}

				DMA1->IFCR |= DMA_IFCR_CTCIF7;					// transfer complete flag clear for channel 7
				DMA1_Channel7->CCR |= DMA_CCR_TCIE;				// transfer complete interrupt enabled
			}
			else if(cmd == 's')				// set new 'x' reference value
			{
				// enable DMA1 channel 6 (USART2_RX)
				USART2->CR3 |= USART_CR3_DMAR;						// DMA mode enabled for reception (sending requests)
				DMA1_Channel6->CCR &= ~(uint32_t)(DMA_CCR_EN);		// disable DMA1 CH6 (USART2_RX)
				DMA1_Channel6->CMAR = (uint32_t)&(x_ref);			// 'x' reference float instance
				DMA1_Channel6->CNDTR = sizeof(float);				// set number of data to transfer
				DMA1_Channel6->CCR |= DMA_CCR_EN;					// activate DMA1 CH6 (USART2_RX)
				DMA1_Channel6->CCR |= DMA_CCR_TCIE;					// transfer complete interrupt enabled

				status.refChange = 1;		// reference value change flag set
			}
			else if (cmd == 'd')			// disable control
			{
				if (param.ctrMode != MODE_NO_CONTROL) savedCtrMode = param.ctrMode;		// save control mode setting
				param.ctrMode = MODE_NO_CONTROL;										// disable control
			}
			else if (cmd == 'e')			// enable control
			{
				if (savedCtrMode != MODE_NO_CONTROL) param.ctrMode = savedCtrMode;		// restore control mode setting
				else param.ctrMode = MODE_LQR_DUAL_KALMAN;
			}
			else if (cmd == 'h')		// trolley "hit" imitation
			{
				status.disturbanceSim = 1;		// disturbance simulation flag set (conducted in main loop)
			}

			USART2->RQR |= USART_RQR_RXFRQ;		// clear RXNE (read register not empty) flag
		}
	}
}

void TIM4_IRQHandler(void)				// every 0.1 ms
{
	if (TIM4->SR &= TIM_SR_UIF)			// if UIF flag caused interrupt
	{
		TIM4->SR &= ~(uint32_t)(TIM_SR_UIF);	// clear interrupt flag

		// state measurements / estimation + control signal value determination
		if(param.ctrMode == MODE_LQR_KALMAN_OBSERVER) {		// LQR + Kalman filter as full state observer
			KalmanObserver();
			LQR();
		}
		else if(param.ctrMode == MODE_LQR_DUAL_KALMAN)	{	// LQR + two separate Kalman filters for cart and pendulum
			DualKalman();
			LQR();
		}
		else if (param.ctrMode == MODE_PID) {	// PID + no estimation (pure measurements)
			PIDMeas();
			PID();
		}
		else if (param.ctrMode == MODE_EXTERNAL_CONTROL) {
			PendDownMeas();
		}
		else {							// no control law (pure measurements)
			PIDMeas();
			u = 0;						// control signal set to 0 V
		}

		setCtrSig(); 					// generating control signal
	}
}

static void NVIC_Init(void)
{
	NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_2);		// 4 group priorities & 4 subpriorities
	NVIC_SetPriority(USART2_IRQn, 0b1010);				// USART2 read register not empty (group 3, priority 2)
	NVIC_SetPriority(DMA1_Channel7_IRQn, 0b1001);		// USART2_TX DMA transfer complete (group 3, priority 1)
	NVIC_SetPriority(DMA1_Channel6_IRQn, 0b1000);		// USART2_RX DMA transfer complete (group 3, priority 0)
	NVIC_SetPriority(TIM5_IRQn, 0b0100);				// TIM5 interrupt - meas output (group 1, priority 0)
	NVIC_SetPriority(TIM4_IRQn, 0b0000);				// TIM4 interrupt - control loop (group 0, priority 0)
	NVIC_EnableIRQ(USART2_IRQn);
	NVIC_EnableIRQ(DMA1_Channel7_IRQn);
	NVIC_EnableIRQ(DMA1_Channel6_IRQn);
	NVIC_DisableIRQ(TIM5_IRQn);							// disabled (enabled via USART2 communication)
	NVIC_DisableIRQ(TIM4_IRQn);							// enabled externally in program
}

static void Encoder_Init(void)
{
	// PA0 (A0) PA1 (A1)  (General purpose TIM2 32-bit)
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;					// GPIOA clock enable
	RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;					// TIM2 clock enable

	GPIOA->AFR[0] &= ~(uint32_t)(GPIO_AFRL_AFRL0);
	GPIOA->AFR[0] &= ~(uint32_t)(GPIO_AFRL_AFRL1);
	GPIOA->AFR[0] |= GPIO_AF1_TIM2 << GPIO_AFRL_AFSEL0_Pos; // 0b0001 << 0; // port A, pin 0, alternative function AF2 (TIM2_CH1)
	GPIOA->AFR[0] |= GPIO_AF1_TIM2 << GPIO_AFRL_AFSEL1_Pos; // 0b0001 << 4; // port A, pin 1, alternative function AF2 (TIM2_CH2)

	GPIOA->PUPDR &= ~(uint32_t)(GPIO_PUPDR_PUPDR0);
	GPIOA->PUPDR &= ~(uint32_t)(GPIO_PUPDR_PUPDR1);
	GPIOA->PUPDR |= GPIO_NOPULL << GPIO_PUPDR_PUPD0_Pos; 	// 0b00 << 0; // portA, pin 0, no pull-up, pull-down
	GPIOA->PUPDR |= GPIO_NOPULL << GPIO_PUPDR_PUPD1_Pos; 	// 0b00 << 2; // portA, pin 1, no pull-up, pull-down

	GPIOA->MODER &= ~(uint32_t)(GPIO_MODER_MODE0);
	GPIOA->MODER &= ~(uint32_t)(GPIO_MODER_MODE1);
	GPIOA->MODER |= GPIO_MODE_AF_PP << GPIO_MODER_MODE0_Pos; // 0b10 << 0; // port A, pin 0, alternate function mode (push-pull)
	GPIOA->MODER |= GPIO_MODE_AF_PP << GPIO_MODER_MODE1_Pos; // 0b10 << 2; // port A, pin 1, alternate function mode (push-pull)

	GPIOA->OSPEEDR &= ~(uint32_t)(GPIO_OSPEEDR_OSPEED0);
	GPIOA->OSPEEDR &= ~(uint32_t)(GPIO_OSPEEDR_OSPEED1);
	GPIOA->OSPEEDR |= GPIO_SPEED_FREQ_LOW << GPIO_OSPEEDR_OSPEED0_Pos; // 0b00 << 12; // port A, pin 0, low output speed
	GPIOA->OSPEEDR |= GPIO_SPEED_FREQ_LOW << GPIO_OSPEEDR_OSPEED1_Pos; // 0b00 << 14; // port A, pin 1, low output speed

	TIM2->ARR = 256000;									// wartosc rejestru autoreload (2000 impulsy x 4 zbocza x 32 obroty = 256000 = 2 m)
	TIM2->CCMR1 = TIM_CCMR1_CC2S_0 | TIM_CCMR1_CC1S_0; 	// IC1 (input capture channel 1) mapped on TI1, IC2 mapped on TI2 (reakcja na zbocze opadajace i narastajace)
	TIM2->CCER = TIM_CCER_CC1P | TIM_CCER_CC2P;			// ustawienie polaryzacji wejsc
	TIM2->SMCR |= TIM_SMCR_SMS_1 | TIM_SMCR_SMS_0;		// wlaczenie trybu enkodera
	TIM2->CR1 = TIM_CR1_CEN;							// TIM2 enable

	// PA6 (D12) PA7 (D11)  (General purpose TIM3 16-bit)
	RCC->APB1ENR1 |= RCC_APB1ENR1_TIM3EN;					// TIM3 clock enable

	GPIOA->AFR[0] &= ~(uint32_t)(GPIO_AFRL_AFRL6);
	GPIOA->AFR[0] &= ~(uint32_t)(GPIO_AFRL_AFRL7);
	GPIOA->AFR[0] |= GPIO_AF2_TIM3 << GPIO_AFRL_AFSEL6_Pos; // 0b0010 << 24; // port A, pin 6, alternative function AF2 (TIM3_CH1)
	GPIOA->AFR[0] |= GPIO_AF2_TIM3 << GPIO_AFRL_AFSEL7_Pos; // 0b0010 << 28; // port A, pin 7, alternative function AF2 (TIM3_CH2)

	GPIOA->PUPDR &= ~(uint32_t)(GPIO_PUPDR_PUPDR6);
	GPIOA->PUPDR &= ~(uint32_t)(GPIO_PUPDR_PUPDR7);
	GPIOA->PUPDR |= GPIO_NOPULL << GPIO_PUPDR_PUPD6_Pos; 	// 0b00 << 12; // portA, pin 6, no pull-up, pull-down
	GPIOA->PUPDR |= GPIO_NOPULL << GPIO_PUPDR_PUPD7_Pos; 	// 0b00 << 14; // portA, pin 7, no pull-up, pull-down

	GPIOA->MODER &= ~(uint32_t)(GPIO_MODER_MODE6);
	GPIOA->MODER &= ~(uint32_t)(GPIO_MODER_MODE7);
	GPIOA->MODER |= GPIO_MODE_AF_PP << GPIO_MODER_MODE6_Pos; // 0b10 << 12; // port A, pin 6, alternate function mode (push-pull)
	GPIOA->MODER |= GPIO_MODE_AF_PP << GPIO_MODER_MODE7_Pos; // 0b10 << 14; // port A, pin 7, alternate function mode (push-pull)

	GPIOA->OSPEEDR &= ~(uint32_t)(GPIO_OSPEEDR_OSPEED6);
	GPIOA->OSPEEDR &= ~(uint32_t)(GPIO_OSPEEDR_OSPEED7);
	GPIOA->OSPEEDR |= GPIO_SPEED_FREQ_LOW << GPIO_OSPEEDR_OSPEED6_Pos; // 0b00 << 12; // port A, pin 6, low output speed
	GPIOA->OSPEEDR |= GPIO_SPEED_FREQ_LOW << GPIO_OSPEEDR_OSPEED7_Pos; // 0b00 << 14; // port A, pin 7, low output speed

	TIM3->ARR = 8000;									// wartosc rejestru autoreload (2000 impulsy x 4 zbocza = 8000 = full rotation)
	TIM3->CCMR1 = TIM_CCMR1_CC2S_0 | TIM_CCMR1_CC1S_0; 	// IC1 (input capture channel 1) mapped on TI1, IC2 mapped on TI2 (reakcja na zbocze opadajace i narastajace)
	TIM3->CCER = TIM_CCER_CC1P | TIM_CCER_CC2P;			// ustawienie polaryzacji wejsc
	TIM3->SMCR |= TIM_SMCR_SMS_1 | TIM_SMCR_SMS_0;		// wlaczenie trybu enkodera
	TIM3->CR1 = TIM_CR1_CEN;							// TIM3 enable
}

static void limitSwitch_Init(void)
{
	// PB9 (D14) limit switch digital input (internal pull-up)
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;						// GPIOB clock enable

	GPIOB->AFR[1] &= ~(uint32_t)(GPIO_AFRH_AFRH1);				// port B, pin 9, no alternative function

	GPIOB->PUPDR &= ~(uint32_t)(GPIO_PUPDR_PUPDR9);
	GPIOB->PUPDR |= GPIO_NOPULL << GPIO_PUPDR_PUPD9_Pos; 		// 0b00 << 18; // port B, pin 9, no pull-up no pull-down

	GPIOB->MODER &= ~(uint32_t)(GPIO_MODER_MODE9);
	GPIOB->MODER |= GPIO_MODE_INPUT << GPIO_MODER_MODE9_Pos; 	// 0b00 << 18; // port B, pin 9, input mode
}

static void DMA_Init(void)
{
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;		// DMA1 clock enable

	// CONFIG PARAMETRS OUTPUT TRANSFER

		// disable DMA1 channel 7
		DMA1_Channel7->CCR &= ~(uint32_t)(DMA_CCR_EN);

		// mapping DMA1 CH7
		DMA1_CSELR->CSELR &= ~(uint32_t)(DMA_CSELR_C7S);
		DMA1_CSELR->CSELR |= DMA_REQUEST_2 << DMA_CSELR_C7S_Pos;	// requests from USART2_TX

		// set peripheral register address
		DMA1_Channel7->CPAR = (uint32_t)&(USART2->TDR);	// TransmitDataRegister of USART2

		// set memory address
		DMA1_Channel7->CMAR = (uint32_t)&(param);		// "Config" structure instance

		// set number of data to transfer
		DMA1_Channel7->CNDTR = sizeof(struct Config);	// number of data equal to size of "Config" structure

		// set channel priority
		DMA1_Channel7->CCR &= ~(uint32_t)(DMA_CCR_PL);
		DMA1_Channel7->CCR |= DMA_PRIORITY_MEDIUM << DMA_CCR_PL_Pos;	// 0b01 << 12; // medium priority level

		// set data transfer direction
		DMA1_Channel7->CCR |= DMA_CCR_DIR;	// read from memory

		// set circular mode
		DMA1_Channel7->CCR &= ~(uint32_t)(DMA_CCR_CIRC);	// circular mode disabled

		// set peripheral and memory increment mode
		DMA1_Channel7->CCR &= ~(uint32_t)(DMA_CCR_PINC);	// peripheral increment disabled
		DMA1_Channel7->CCR |= DMA_CCR_MINC;					// memory increment enabled

		// set peripheral and memory data size
		DMA1_Channel7->CCR &= ~(uint32_t)(DMA_CCR_PSIZE);
		DMA1_Channel7->CCR |= DMA_PDATAALIGN_BYTE << DMA_CCR_PSIZE_Pos; 	// 0b00 << 8; // 8 bits (byte)
		DMA1_Channel7->CCR &= ~(uint32_t)(DMA_CCR_MSIZE);
		DMA1_Channel7->CCR |= DMA_MDATAALIGN_BYTE << DMA_CCR_MSIZE_Pos; 	// 0b00 << 10; // 8 bits (byte)

		// set interrupt
		DMA1_Channel7->CCR &= ~(uint32_t)(DMA_CCR_TCIE);	// transfer complete interrupt disabled


	// CONFIG PARAMETRS INPUT TRANSFER

		// disable DMA1 channel 6
		DMA1_Channel6->CCR &= ~(uint32_t)(DMA_CCR_EN);

		// mapping DMA1 CH6
		DMA1_CSELR->CSELR &= ~(uint32_t)(DMA_CSELR_C6S);
		DMA1_CSELR->CSELR |= DMA_REQUEST_2 << DMA_CSELR_C6S_Pos;	// requests from USART2_RX

		// set peripheral register address
		DMA1_Channel6->CPAR = (uint32_t)&(USART2->RDR);	// ReceiveDataRegister of USART2

		// set memory address
		DMA1_Channel6->CMAR = (uint32_t)&(param);		// "Config" structure instance

		// set number of data to transfer
		DMA1_Channel6->CNDTR = sizeof(struct Config);	// number of data equal to size of "Config" structure

		// set channel priority
		DMA1_Channel6->CCR &= ~(uint32_t)(DMA_CCR_PL);
		DMA1_Channel6->CCR |= DMA_PRIORITY_HIGH << DMA_CCR_PL_Pos;	// 0b10 << 12; // medium priority level

		// set data transfer direction
		DMA1_Channel6->CCR &= ~(uint32_t)(DMA_CCR_DIR);		// read from peripheral

		// set circular mode
		DMA1_Channel6->CCR &= ~(uint32_t)(DMA_CCR_CIRC);	// circular mode disabled

		// set peripheral and memory increment mode
		DMA1_Channel6->CCR &= ~(uint32_t)(DMA_CCR_PINC);	// peripheral increment disabled
		DMA1_Channel6->CCR |= DMA_CCR_MINC;					// memory increment enabled

		// set peripheral and memory data size
		DMA1_Channel6->CCR &= ~(uint32_t)(DMA_CCR_PSIZE);
		DMA1_Channel6->CCR |= DMA_PDATAALIGN_BYTE << DMA_CCR_PSIZE_Pos; 	// 0b00 << 8; // 8 bits (byte)
		DMA1_Channel6->CCR &= ~(uint32_t)(DMA_CCR_MSIZE);
		DMA1_Channel6->CCR |= DMA_MDATAALIGN_BYTE << DMA_CCR_MSIZE_Pos; 	// 0b00 << 10; // 8 bits (byte)

		// set interrupt
		DMA1_Channel6->CCR &= ~(uint32_t)(DMA_CCR_TCIE);	// transfer complete interrupt disabled

}

static void USART_Init(void)
{
	RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN; // USART2 clock enable
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;	// GPIOA clock enable

	GPIOA->AFR[0] &= ~(uint32_t)(GPIO_AFRL_AFRL2);
	GPIOA->AFR[0] &= ~(uint32_t)(GPIO_AFRL_AFRL3);
	GPIOA->AFR[0] |= GPIO_AF7_USART2 << GPIO_AFRL_AFSEL2_Pos; // 0b0111 << 8; // port A, pin 2, alternative function AF7 (USART2_TX)
	GPIOA->AFR[0] |= GPIO_AF7_USART2 << GPIO_AFRL_AFSEL3_Pos; // 0b0111 << 12; // port A, pin 3, alternative function AF7 (USART2_RX)

	GPIOA->PUPDR &= ~(uint32_t)(GPIO_PUPDR_PUPDR3);
	GPIOA->PUPDR |= GPIO_PULLDOWN << GPIO_PUPDR_PUPD3_Pos; // 0b10 << 6; // portA, pin 3, pull-down

	GPIOA->MODER &= ~(uint32_t)(GPIO_MODER_MODE2);
	GPIOA->MODER &= ~(uint32_t)(GPIO_MODER_MODE3);
	GPIOA->MODER |= GPIO_MODE_AF_PP << GPIO_MODER_MODE2_Pos; // 0b10 << 4; // port A, pin 2, alternate function mode (push-pull)
	GPIOA->MODER |= GPIO_MODE_AF_PP << GPIO_MODER_MODE3_Pos; // 0b10 << 6; // port A, pin 3, alternate function mode (push-pull)

	GPIOA->OSPEEDR &= ~(uint32_t)(GPIO_OSPEEDR_OSPEED2);
	GPIOA->OSPEEDR &= ~(uint32_t)(GPIO_OSPEEDR_OSPEED3);
	GPIOA->OSPEEDR |= GPIO_SPEED_FREQ_VERY_HIGH << GPIO_OSPEEDR_OSPEED2_Pos; // 0b11 << 4; // port A, pin 2, very high output speed
	GPIOA->OSPEEDR |= GPIO_SPEED_FREQ_VERY_HIGH << GPIO_OSPEEDR_OSPEED3_Pos; // 0b11 << 6; // port A, pin 3, very high output speed

	// setting word length
	// M bits in USART2_CR1 left "00": 8-bit character length

	// OVER8 bit in USART2_CR1 left "0": oversampling by 16
	// ONEBIT bit in USART2_CR3 left "0": three sampling majority vote method

	// setting baud rate (oversampling by 16):
	// USARTDIV = 80 000 000 Hz / 115200 baud = 694
	USART2->BRR = 694;

	// setting stop bits
	// STOP bits in USART2_CR2 left "00": 1 stop bit

	// setting parity control
	// PCE bit in USART2_CR1 left "0": no parity bit
	// PC bit in USART2_CR1 left "0": even parity

	// enable USART
	// USART2->CR1 = 0;
	USART2->CR1 = USART_CR1_UE;

	// DMA request enable
	USART2->CR1 |= USART_CR1_RXNEIE;	// RXNE (read register not empty) interrupt enabled

	// enable transmission
	USART2->CR1 |= USART_CR1_TE | USART_CR1_RE;

}

static void PWM_Init(void)
{
	// PWM  (Advanced control TIM1 16-bit)
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;			// GPIOA clock enable

	GPIOA->AFR[1] &= ~(uint32_t)(GPIO_AFRH_AFRH0);
	GPIOA->AFR[1] |= GPIO_AF1_TIM1 << GPIO_AFRH_AFSEL8_Pos; 					// 0b0001 << 0; // port A, pin8, alternative function AF1 (TIM1_CH1)

	GPIOA->PUPDR &= ~(uint32_t)(GPIO_PUPDR_PUPDR8);
	GPIOA->PUPDR |= GPIO_NOPULL << GPIO_PUPDR_PUPD8_Pos; 						// 0b00 << 16; // portA, pin 8, no pull-up, pull-down

	GPIOA->MODER &= ~(uint32_t)(GPIO_MODER_MODE8);
	GPIOA->MODER |= GPIO_MODE_AF_PP << GPIO_MODER_MODE8_Pos; 					// 0b01 << 16; // port A, pin 8, output mode (push-pull)

	GPIOA->OSPEEDR &= ~(uint32_t)(GPIO_OSPEEDR_OSPEED8);
	GPIOA->OSPEEDR |= GPIO_SPEED_FREQ_VERY_HIGH << GPIO_OSPEEDR_OSPEED8_Pos; 	// 0b11 << 16; // port A, pin 8, very high output speed

	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;				// TIM1 clock enable

	TIM1->PSC = 4;								    // 80 MHz / 20 MHz = 4 (tic freq 20 MHz)
	TIM1->ARR = 1000;								// autoreload value -> resolution (cycle freq 20 kHz)
	TIM1->CCR1 = 500;								// capture compare value -> duty cycle
	TIM1->CCMR1 &= ~(uint32_t)(TIM_CCMR1_OC1M);
	TIM1->CCMR1 |= 0b0110 << TIM_CCMR1_OC1M_Pos; 	// output compare 1 - PWM mode 1
	TIM1->CCMR1 |= TIM_CCMR1_OC1PE; 				// preload register on TIM1CCR1 enabled (TIM1CCR1 is buffered)
	TIM1->CR1 |= TIM_CR1_ARPE;						// auto-reload preload enable (TIM1ARR register is buffered)
	TIM1->CR1 &= ~(uint16_t)(TIM_CR1_DIR);			// counter used as upcounter
	TIM1->EGR |= TIM_EGR_UG;						// reinitialize the counter - prescaler counter too (auto cleared by hardware)
	TIM1->CCER &= ~(uint32_t)(TIM_CCER_CC1P);		// capture/compare 1 output polarity - active high
	TIM1->BDTR |= TIM_BDTR_MOE;						// OC1 output enabled if CC1E bit set in CCER reg)
	TIM1->CCER |= TIM_CCER_CC1E;					// capture/compare 1 output enabled on the corresponding pin (PA8 - D7)
	TIM1->CR1 |= TIM_CR1_CEN;						// TIM1 enabled

	// PA9 (D8) direction digital output signal (INA)
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;									// GPIOA clock enable

	GPIOA->AFR[1] &= ~(uint32_t)(GPIO_AFRH_AFRH1);							// port A, pin 9, no alternative function

	GPIOA->PUPDR &= ~(uint32_t)(GPIO_PUPDR_PUPDR9);
	GPIOA->PUPDR |= GPIO_NOPULL << GPIO_PUPDR_PUPD9_Pos; 					// 0b00 << 18; // portA, pin 9, no pull-up, pull-down

	GPIOA->MODER &= ~(uint32_t)(GPIO_MODER_MODE9);
	GPIOA->MODER |= GPIO_MODE_OUTPUT_PP << GPIO_MODER_MODE9_Pos; 			// 0b01 << 18; // port A, pin 9, output mode (push-pull)

	GPIOA->OSPEEDR &= ~(uint32_t)(GPIO_OSPEEDR_OSPEED9);
	GPIOA->OSPEEDR |= GPIO_SPEED_FREQ_HIGH << GPIO_OSPEEDR_OSPEED9_Pos; 	// 0b10 << 18; // port A, pin 9, high output speed

	GPIOA->ODR |= GPIO_ODR_OD9;												// pin set to high

	// PA10 (D2) direction digital output signal (INB)
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;									// GPIOA clock enable

	GPIOA->AFR[1] &= ~(uint32_t)(GPIO_AFRH_AFRH2);							// port A, pin 10, no alternative function

	GPIOA->PUPDR &= ~(uint32_t)(GPIO_PUPDR_PUPDR10);
	GPIOA->PUPDR |= GPIO_NOPULL << GPIO_PUPDR_PUPD10_Pos; 					// 0b00 << 20; // portA, pin 10, no pull-up, pull-down

	GPIOA->MODER &= ~(uint32_t)(GPIO_MODER_MODE10);
	GPIOA->MODER |= GPIO_MODE_OUTPUT_PP << GPIO_MODER_MODE10_Pos; 			// 0b01 << 20; // port A, pin 10, output mode (push-pull)

	GPIOA->OSPEEDR &= ~(uint32_t)(GPIO_OSPEEDR_OSPEED10);
	GPIOA->OSPEEDR |= GPIO_SPEED_FREQ_HIGH << GPIO_OSPEEDR_OSPEED10_Pos; 	// 0b10 << 20; // port A, pin 10, high output speed

	GPIOA->ODR |= GPIO_ODR_OD10;											// pin set to high
}

static void ControlLoop_Init(void)
{
	// General purpose TIM4 16-bit
	RCC->APB1ENR1 |= RCC_APB1ENR1_TIM4EN;			// TIM4 clock enable

	TIM4->PSC = 8;									// 80 MHz / 10 MHz = 8 (tic freq 10 MHz)
	TIM4->ARR = 1000;								// autoreload value -> resolution (cycle freq 10kHz, T = 100us)

	//TIM4->CR1 |= TIM_CR1_ARPE;					// auto-reload preload enable (TIM1ARR register is buffered)

	TIM4->CR1 &= ~(uint16_t)(TIM_CR1_DIR);			// counter used as upcounter
	TIM4->EGR |= TIM_EGR_UG;						// reinitialize the counter - prescaler counter too (auto cleared by hardware)

	TIM4->DIER |= TIM_DIER_UIE;						// update interrupt enabled

	TIM4->CR1 |= TIM_CR1_CEN;						// TIM4 enabled

}

static void MeasOutput_Init(void)
{
	// General purpose TIM5 32-bit
	RCC->APB1ENR1 |= RCC_APB1ENR1_TIM5EN;			// TIM5 clock enable

	// for baud rate 115200 [bit/s] = 14.4 [B/ms] -> sampling period 10 ms (100 Hz)

	TIM5->PSC = 800;								// 80 MHz / 0.1 MHz = 800 (tic freq 100 kHz)
	TIM5->ARR = 1000;								// autoreload value -> resolution (cycle freq 100 Hz, T = 10 ms)

	//TIM5->CR1 |= TIM_CR1_ARPE;					// auto-reload preload enable (TIM1ARR register is buffered)

	TIM5->CR1 &= ~(uint16_t)(TIM_CR1_DIR);			// counter used as upcounter
	TIM5->EGR |= TIM_EGR_UG;						// reinitialize the counter - prescaler counter too (auto cleared by hardware)

	TIM5->DIER |= TIM_DIER_UIE;						// update interrupt enabled

	TIM5->CR1 |= TIM_CR1_CEN;						// TIM5 enabled

}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
