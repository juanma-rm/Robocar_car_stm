/******************************************************************************
 * Project: Robocar
 * Application: on board app
 * Platform: STM Nucleo-F767ZI
 * @file motor_ctrl.c
 * @version v1.0
 * @brief File containing functions related to motor control (PWM + PID)
 *
 *
 *
 * Requirements:
 * 	1)
 *
 * Warnings!
 * 	1) TIM3_CH3 is used here. Do not manipulate it from any other part of the code!
 *
 ******************************************************************************/

/*******************************************************************************
 * INCLUSIONS
 ******************************************************************************/

// Std

// Freertos and hardware
#include "cmsis_os.h"
#include "semphr.h"
#include "queue.h"
// Project
#include "motor_driver.h"
#include "systools.h"

/*******************************************************************************
 * HARDWARE RESOURCES
 ******************************************************************************/

// DIRECTION_L
#define DIR_FORW_L_PIN		GPIO_PIN_10
#define DIR_FORW_L_PORT		GPIOE
#define DIR_FORW_L_CLK_EN()	__HAL_RCC_GPIOE_CLK_ENABLE()
#define DIR_BACK_L_PIN		GPIO_PIN_12
#define DIR_BACK_L_PORT		GPIOE
#define DIR_BACK_L_CLK_EN()	__HAL_RCC_GPIOE_CLK_ENABLE()
// DIRECTION_R
#define DIR_FORW_R_PIN		GPIO_PIN_14
#define DIR_FORW_R_PORT		GPIOE
#define DIR_FORW_R_CLK_EN()	__HAL_RCC_GPIOE_CLK_ENABLE()
#define DIR_BACK_R_PIN		GPIO_PIN_15
#define DIR_BACK_R_PORT		GPIOE
#define DIR_BACK_R_CLK_EN()	__HAL_RCC_GPIOE_CLK_ENABLE()
// PMW_L
#define PWM_L_PIN		GPIO_PIN_8
#define PWM_L_PORT		GPIOC
#define PWM_L_CLK_EN()	__HAL_RCC_GPIOC_CLK_ENABLE()
#define PWM_L_GPIO_AF	GPIO_AF2_TIM3
#define PWM_L_TIMER_ID	TIM3
#define PWM_L_TIMER_CH	TIM_CHANNEL_3
#define PWM_L_TIM_EN()	__HAL_RCC_TIM3_CLK_ENABLE()
#define PWM_L_TIM_DIS()	__HAL_RCC_TIM3_CLK_DISABLE()
// PWM_R
#define PWM_R_PIN		GPIO_PIN_1
#define PWM_R_PORT		GPIOB
#define PWM_R_CLK_EN()	__HAL_RCC_GPIOB_CLK_ENABLE()
#define PWM_R_GPIO_AF	GPIO_AF2_TIM3
#define PWM_R_TIMER_ID	TIM3
#define PWM_R_TIMER_CH	TIM_CHANNEL_4
#define PWM_R_TIM_EN()	__HAL_RCC_TIM3_CLK_ENABLE()
#define PWM_R_TIM_DIS()	__HAL_RCC_TIM3_CLK_DISABLE()
// ENCODER_L
#define ENCODER_L_FORW_PIN		GPIO_PIN_12
#define ENCODER_L_FORW_PORT		GPIOD
#define ENCODER_L_FORW_CLK_EN()	__HAL_RCC_GPIOD_CLK_ENABLE()
#define ENCODER_L_BACK_PIN		GPIO_PIN_13
#define ENCODER_L_BACK_PORT		GPIOD
#define ENCODER_L_BACK_CLK_EN()	__HAL_RCC_GPIOD_CLK_ENABLE()
#define ENCODER_L_GPIO_AF		GPIO_AF2_TIM4
#define ENCODER_L_TIMER_ID		TIM4
#define ENCODER_L_TIM_EN()		__HAL_RCC_TIM4_CLK_ENABLE()
#define ENCODER_L_TIM_DIS()		__HAL_RCC_TIM4_CLK_DISABLE()
// ENCODER_R
#define ENCODER_R_FORW_PIN		GPIO_PIN_6
#define ENCODER_R_FORW_PORT		GPIOC
#define ENCODER_R_FORW_CLK_EN()	__HAL_RCC_GPIOC_CLK_ENABLE()
#define ENCODER_R_BACK_PIN		GPIO_PIN_7
#define ENCODER_R_BACK_PORT		GPIOC
#define ENCODER_R_BACK_CLK_EN()	__HAL_RCC_GPIOC_CLK_ENABLE()
#define ENCODER_R_GPIO_AF		GPIO_AF3_TIM8
#define ENCODER_R_TIMER_ID		TIM8
#define ENCODER_R_TIM_EN()		__HAL_RCC_TIM8_CLK_ENABLE()
#define ENCODER_R_TIM_DIS()		__HAL_RCC_TIM8_CLK_DISABLE()

/*******************************************************************************
 * DATA STRUCTS
 ******************************************************************************/

typedef struct {
	GPIO_TypeDef *forw_gpio_port;
	unsigned int forw_gpio_pin;
	GPIO_TypeDef *back_gpio_port;
	unsigned int back_gpio_pin;
} dir_struct;

typedef struct {
	GPIO_TypeDef *gpio_port;
	unsigned int gpio_pin;
	uint8_t gpio_af;
	TIM_HandleTypeDef timer_config;
	TIM_TypeDef *timer_id;
	unsigned int tim_pwm_channel;
	unsigned int period;
	TIM_OC_InitTypeDef config_out_comp;
} pwm_struct;

typedef struct {
	GPIO_TypeDef *forw_gpio_port;
	unsigned int forw_gpio_pin;
	GPIO_TypeDef *back_gpio_port;
	unsigned int back_gpio_pin;
	uint8_t gpio_af;
	TIM_HandleTypeDef timer_config;
	TIM_TypeDef *timer_id;
	unsigned int last_measurement;
} encoder_struct;

/*******************************************************************************
 * GLOBAL VARIABLES & MACROS
 ******************************************************************************/

/* dir */
static dir_struct dir_L = {
	.forw_gpio_port = DIR_FORW_L_PORT,
	.forw_gpio_pin = DIR_FORW_L_PIN,
	.back_gpio_port = DIR_BACK_L_PORT,
	.back_gpio_pin = DIR_BACK_L_PIN,
};
static dir_struct dir_R = {
	.forw_gpio_port = DIR_FORW_R_PORT,
	.forw_gpio_pin = DIR_FORW_R_PIN,
	.back_gpio_port = DIR_BACK_R_PORT,
	.back_gpio_pin = DIR_BACK_R_PIN,
};

/* PWM */
#define TIM_PWM_PRESC	96
#define TIM3_FREQ_HZ	(HCLK_FREQ_HZ/TIM3_PRESC)	// 1 us (for HCLK=96MHz)
#define TIM3_FREQ_MHZ	(TIM3_FREQ_HZ/MHZ_TO_HZ)
static pwm_struct pwm_L = {
	.gpio_port = PWM_L_PORT,
	.gpio_pin = PWM_L_PIN,
	.gpio_af = PWM_L_GPIO_AF,
	.timer_id = PWM_L_TIMER_ID,
	.tim_pwm_channel = PWM_L_TIMER_CH,
	.period = 1000,
	.config_out_comp = {0},
};
static pwm_struct pwm_R = {
	.gpio_port = PWM_R_PORT,
	.gpio_pin = PWM_R_PIN,
	.gpio_af = PWM_R_GPIO_AF,
	.timer_id = PWM_R_TIMER_ID,
	.tim_pwm_channel = PWM_R_TIMER_CH,
	.period = 1000,
	.config_out_comp = {0},
};

/* Encoders */
#define ENCODER_RESOLUTION	178	// Encoder resolution in points per revolution:
								// each signal (CW and CWW) takes 178*2 edges
								// per full revolution
#define WHEEL_DIAMETER_MM	65
#define PI	3.14159

static encoder_struct encoder_L = {
	.forw_gpio_port = ENCODER_L_FORW_PORT,
	.forw_gpio_pin = ENCODER_L_FORW_PIN,
	.back_gpio_port = ENCODER_L_BACK_PORT,
	.back_gpio_pin = ENCODER_L_BACK_PIN,
	.gpio_af = ENCODER_L_GPIO_AF,
	.timer_id = ENCODER_L_TIMER_ID,
	.last_measurement = 0
};
static encoder_struct encoder_R = {
	.forw_gpio_port = ENCODER_R_FORW_PORT,
	.forw_gpio_pin = ENCODER_R_FORW_PIN,
	.back_gpio_port = ENCODER_R_BACK_PORT,
	.back_gpio_pin = ENCODER_R_BACK_PIN,
	.gpio_af = ENCODER_R_GPIO_AF,
	.timer_id = ENCODER_R_TIMER_ID,
	.last_measurement = 0
};

/* Semaphores and queues definitions */


/* Misc */



/*******************************************************************************
 * FUNCTIONS DEFINITIONS: PRIVATE
 ******************************************************************************/

static void _init_dirx(dir_struct *dir) {
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	// CW
	GPIO_InitStruct.Pin = dir->forw_gpio_pin;
	HAL_GPIO_WritePin(dir->forw_gpio_port, dir->forw_gpio_pin, GPIO_PIN_RESET);
	HAL_GPIO_Init(dir->forw_gpio_port, &GPIO_InitStruct);
	// CWW
	GPIO_InitStruct.Pin = dir->back_gpio_pin;
	HAL_GPIO_WritePin(dir->back_gpio_port, dir->back_gpio_pin, GPIO_PIN_RESET);
	HAL_GPIO_Init(dir->back_gpio_port, &GPIO_InitStruct);
}

static void _init_dir(void) {
	DIR_FORW_L_CLK_EN();
	DIR_BACK_L_CLK_EN();
	DIR_FORW_R_CLK_EN();
	DIR_BACK_R_CLK_EN();
	_init_dirx(&dir_L);
	_init_dirx(&dir_R);

}

static void _init_PWMx(pwm_struct *motor) {
	// Configure GPIO for PWM output
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = motor->gpio_pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = motor->gpio_af;
	HAL_GPIO_Init(motor->gpio_port, &GPIO_InitStruct);

	// Configure timer / channel / pwm
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	motor->timer_config.Instance = motor->timer_id;
	motor->timer_config.Init.Prescaler = TIM_PWM_PRESC;	// 1 us (for HCLK=96MHz)
	motor->timer_config.Init.CounterMode = TIM_COUNTERMODE_UP;
	motor->timer_config.Init.Period = 1000;	// PWM signal with period of 1000 x 1 us = 1000 us
	motor->timer_config.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	motor->timer_config.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_PWM_Init(&(motor->timer_config)) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&(motor->timer_config), &sMasterConfig) != HAL_OK) {
		Error_Handler();
	}
	motor->config_out_comp.OCMode = TIM_OCMODE_PWM1;
	motor->config_out_comp.Pulse = 0;
	motor->config_out_comp.OCPolarity = TIM_OCPOLARITY_HIGH;
	motor->config_out_comp.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&(motor->timer_config), &(motor->config_out_comp), motor->tim_pwm_channel) != HAL_OK) {
		Error_Handler();
	}
}

static void _init_PWM(void) {
	PWM_L_CLK_EN();
	PWM_R_CLK_EN();
	_init_PWMx(&pwm_L);
	_init_PWMx(&pwm_R);
	PWM_L_TIM_EN();
	PWM_R_TIM_EN();
}

static void _deinit_PWM(void) {
	HAL_TIM_PWM_Stop(&(pwm_L.timer_config), pwm_L.tim_pwm_channel);
	HAL_TIM_PWM_Stop(&(pwm_R.timer_config), pwm_R.tim_pwm_channel);
	PWM_L_TIM_DIS();
	PWM_R_TIM_DIS();
}

static void _set_PWM(pwm_struct *motor, unsigned int duty_perc) {
	HAL_TIM_PWM_Stop(&(motor->timer_config), motor->tim_pwm_channel);
	HAL_TIM_PWM_Init(&(motor->timer_config));
	motor->config_out_comp.Pulse = (unsigned int) (duty_perc*motor->period/100);
	HAL_TIM_PWM_ConfigChannel(&(motor->timer_config), &(motor->config_out_comp), motor->tim_pwm_channel);
	HAL_TIM_PWM_Start(&(motor->timer_config), motor->tim_pwm_channel);
}

static void _init_encoderx(encoder_struct *encoder) {

	// Configure GPIO for PWM output
	// General
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = encoder->gpio_af;
	// CW
	GPIO_InitStruct.Pin = encoder->forw_gpio_pin;
	HAL_GPIO_Init(encoder->forw_gpio_port, &GPIO_InitStruct);
	// CWW
	GPIO_InitStruct.Pin = encoder->back_gpio_pin;
	HAL_GPIO_Init(encoder->back_gpio_port, &GPIO_InitStruct);

	// Configure timer / channel / encoder mode
	encoder->timer_config.Instance = encoder->timer_id;
	encoder->timer_config.Init.Prescaler = 0;
	encoder->timer_config.Init.CounterMode = TIM_COUNTERMODE_UP;
	encoder->timer_config.Init.Period = 65535;
	encoder->timer_config.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	encoder->timer_config.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	TIM_Encoder_InitTypeDef sConfig = {0};
	sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
	sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC1Filter = 0;
	sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC2Filter = 0;
	if (HAL_TIM_Encoder_Init(&(encoder->timer_config), &sConfig) != HAL_OK) {
		Error_Handler();
	}
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&(encoder->timer_config), &sMasterConfig) != HAL_OK) {
		Error_Handler();
	}
}

static void _init_encoders(void) {
	ENCODER_L_TIM_EN();
	ENCODER_L_FORW_CLK_EN();
	ENCODER_L_BACK_CLK_EN();
	_init_encoderx(&encoder_L);

	ENCODER_R_TIM_EN();
	ENCODER_R_FORW_CLK_EN();
	ENCODER_R_BACK_CLK_EN();
	_init_encoderx(&encoder_R);

	HAL_TIM_Encoder_Start(&(encoder_L.timer_config), TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&encoder_R.timer_config, TIM_CHANNEL_ALL);

	encoder_L.last_measurement = __HAL_TIM_GetCounter(&(encoder_L.timer_config));
	encoder_R.last_measurement = __HAL_TIM_GetCounter(&(encoder_R.timer_config));
}

static void _deinit_encoder(void) {
	ENCODER_L_TIM_DIS();
    HAL_GPIO_DeInit(encoder_L.forw_gpio_port, encoder_L.forw_gpio_pin);
    HAL_GPIO_DeInit(encoder_L.back_gpio_port, encoder_L.back_gpio_pin);
    ENCODER_R_TIM_DIS();
    HAL_GPIO_DeInit(encoder_R.forw_gpio_port, encoder_R.forw_gpio_pin);
    HAL_GPIO_DeInit(encoder_R.back_gpio_port, encoder_R.back_gpio_pin);
}

/*******************************************************************************
 * FUNCTIONS DEFINITIONS: PUBLIC
 ******************************************************************************/

void motor_ctrl_init(void) {
	_init_PWM();
	_init_encoders();
	_init_dir();
}

void motor_set(unsigned int side, unsigned int dir, unsigned int pwm_perc) {

	// Input check
	if (pwm_perc < 0) {
		pwm_perc = 0;
	} else if (pwm_perc > 100) {
		pwm_perc = 100;
	}

	// Direction
	dir_struct *dir_ptr = NULL;
	if (side == MOTOR_L) {
		dir_ptr = &dir_L;
	} else  if (side == MOTOR_R) {
		dir_ptr = &dir_R;
	} else {
		while(1); // @todo handle error
	}
	GPIO_PinState in1_val = GPIO_PIN_RESET;
	GPIO_PinState in2_val = GPIO_PIN_RESET;
	if (dir == MOTOR_FORWARD) {
		in1_val = GPIO_PIN_SET;
		in2_val = GPIO_PIN_RESET;
	} else if (dir == MOTOR_BACKWARD) {
		in1_val = GPIO_PIN_RESET;
		in2_val = GPIO_PIN_SET;
	} else if (dir == MOTOR_FREE) {
		in1_val = GPIO_PIN_RESET;
		in2_val = GPIO_PIN_RESET;
	} else {
		while(1); // @todo handle error
	}
	HAL_GPIO_WritePin(dir_ptr->forw_gpio_port, dir_ptr->forw_gpio_pin, in1_val);
	HAL_GPIO_WritePin(dir_ptr->back_gpio_port, dir_ptr->back_gpio_pin, in2_val);

	// PWM
	pwm_struct *pwm = NULL;
	if (side == MOTOR_L) {
		pwm = &pwm_L;
	} else  if (side == MOTOR_R) {
		pwm = &pwm_R;
	} else {
		while(1); // @todo handle error
	}
	_set_PWM(pwm, pwm_perc);
}

void read_speed(unsigned int side, unsigned int time_bet_meas_ms, float *speed_mm_s, int16_t *speed_rpm) {

	// Side selection
	encoder_struct *encoder = NULL;
	if (side == MOTOR_L) {
		encoder = &encoder_L;
	} else  if (side == MOTOR_R) {
		encoder = &encoder_R;
	} else {
		while(1); // @todo handle error
	}

	// Read speed in count difference and convert into mm/s
	unsigned int encoder_value_last = encoder->last_measurement;
	unsigned int encoder_value_now = __HAL_TIM_GetCounter(&(encoder->timer_config));
	int encoder_diff = encoder_value_now - encoder_value_last;
	encoder->last_measurement = encoder_value_now;
	// Angular speed in revolutions per second
	// Each point of resolution in the encoder generates 2 edges per each signal
	// (2 signals: CW and CWW), what means an update of 4 cycles in the counter
	float _speed_rev_s = (float)encoder_diff*1000/(2*2*ENCODER_RESOLUTION*time_bet_meas_ms);
	*speed_rpm = _speed_rev_s*60;
	// Lineal speed in mm per second (circ = 2*pi*r = pi*D)
	float _speed_mm_s = _speed_rev_s*PI*WHEEL_DIAMETER_MM;
	*speed_mm_s = _speed_mm_s;
}
