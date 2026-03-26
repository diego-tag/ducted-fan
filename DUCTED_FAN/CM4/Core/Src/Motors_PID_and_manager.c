/**********************************************************************************************************
 * _______________________________________________________________________________________________________
 *| @file Motors_PID_and_manager.c																		  |
 *| @brief This file contains the function definitions used for motors control						      |
 *| @version 0.2																				          |
 *| @date 27-10-2025																					  |
 *| ______________________________________________________________________________________________________|
 *********************************************************************************************************/

/*-------------------------------------------------------------------------------------------------------*/
/*					  		 			 INCLUDE DIRECTIVES 				      						 */
/*-------------------------------------------------------------------------------------------------------*/

#include "Motors_PID_and_manager.h"

/*-------------------------------------------------------------------------------------------------------*/
/*					  		 		EXTERN VARIABLES DECLARATIONS				      					 */
/*-------------------------------------------------------------------------------------------------------*/

extern uint8_t current_number_of_toggles;
extern UART_HandleTypeDef huart3;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim12;
extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim3;
extern int run;
extern int flag_int;
extern int actuate_servo_control;
extern int actuate_motors_control;
extern uint8_t rx_byte;

/*-------------------------------------------------------------------------------------------------------*/
/*					  		 		    FUNCTIONS DEFINITIONS				      					     */
/*-------------------------------------------------------------------------------------------------------*/

void safe_startup(TIM_HandleTypeDef *tim_secure_start) {
	// Reset counter before starting
	current_number_of_toggles = 0;

	HAL_TIM_Base_Start_IT(tim_secure_start);

	// Use  __WFI() in all waiting loops to save power and prevents busy-waiting CPU hogs
	while (current_number_of_toggles <= NUMBER_OF_TOGGLES) {
		__WFI();
	}

	HAL_TIM_Base_Stop_IT(tim_secure_start);

	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_SET); // turn on LD2 (yellow led)
}

uint16_t actual_perpendicular_ranging_data(VL53L1_DEV dev, VL53L1_RangingMeasurementData_t ranging_data, DPDF_zero_axis_rotation axis_zero_rot,
		DPDF_axis_rotation axis_curr_rot) {

	uint16_t perp_ran_data;
	uint16_t rad_roll;
	uint16_t rad_pitch;

	VL53L1_GetRangingMeasurementData(dev, &ranging_data);
	VL53L1_ClearInterruptAndStartMeasurement(dev);

	DPDF_BNO055_firmware_read(axis_zero_rot, axis_curr_rot);

	rad_roll = axis_curr_rot->rot_x * (M_PI / 180);
	rad_pitch = axis_curr_rot->rot_y * (M_PI / 180);

	perp_ran_data = ranging_data.RangeMilliMeter * cos(rad_roll) * cos(rad_pitch);

	return perp_ran_data;

}

void motors_pid_turner_and_turn_on(float kp, float ki, float kd, float t_c, pid_prmts_t *poi) {
	if (!poi || t_c <= 0.0f)
		return; // Prevent null pointer and zero sample time

	poi->prop_coeff = kp;
	poi->int_coeff = ki;
	poi->der_coeff = kd;
	poi->sampl_time = t_c;
	poi->int_term = 0.0f;

}

// Helper function: Median filter of 3 elements
static float apply_median_filter(pid_prmts_t *poi, int16_t new_val) {
    poi->med_buf[poi->med_idx] = new_val;
    poi->med_idx = (poi->med_idx + 1) % 3;

    int16_t a = poi->med_buf[0];
    int16_t b = poi->med_buf[1];
    int16_t c = poi->med_buf[2];

    // Compute median using an hard-coded algorithm for max efficiency
    int16_t median = (a > b) ? ((b > c) ? b : ((a > c) ? c : a))
                             : ((a > c) ? a : ((b > c) ? c : b));
    return (float)median;
}

uint16_t pid_motors(pid_prmts_t *poi, int16_t raw_measurement, uint16_t ref) {
    // Safety control and prevention of division-by-zero
    if (!poi || poi->sampl_time <= 0.001f) {
        return 0;
    }

    // Initialized at first boot (to avoid problems if buffer is at zero)
    if (!poi->initialized) {
        poi->med_buf[0] = poi->med_buf[1] = poi->med_buf[2] = raw_measurement;
        poi->prev_meas = (float)raw_measurement;
        poi->int_term = 0.0f;
        poi->prev_d_term = 0.0f;
        poi->initialized = true;
    }

    // Signal conditioning (median filter to compensate ToF spikes)
    float filtered_meas = apply_median_filter(poi, raw_measurement);

    // Error
    float err = (float)ref - filtered_meas;

    // --- PROPORTIONAL ---
    float p_term = poi->prop_coeff * err;

    // --- INTEGRAL (with anti-windup) ---
    float new_int_term = poi->int_term + (poi->int_coeff * poi->sampl_time * err);

    // --- DERIVATIVE (on measurement + Low pass filter) ---
    // Use-(meas - prev_meas) instead of (err - prev_err) to avoid derivative kick
    float d_raw = - (poi->der_coeff / poi->sampl_time) * (filtered_meas - poi->prev_meas);

    // Exponential low pass filter on derivative
    float d_term = (poi->lpf_alpha * d_raw) + ((1.0f - poi->lpf_alpha) * poi->prev_d_term);

    // Update memory for next loop
    poi->prev_meas = filtered_meas;
    poi->prev_d_term = d_term;

    // Total output
    float output = p_term + new_int_term + d_term;

    // ANTI-WINDUP (clamping method) & SATURATION
    if (output > UPPER_LIMIT_TOP_MOTOR_SATURATION) {
        output = UPPER_LIMIT_TOP_MOTOR_SATURATION;
        // Do not accumulate integral if it's saturated (upper) AND error keep increasing
        if (err < 0.0f) poi->int_term = new_int_term;
    }
    else if (output < LOWER_LIMIT_TOP_MOTOR_SATURATION) {
        output = LOWER_LIMIT_TOP_MOTOR_SATURATION;
        // Do not accumulate integral if it's saturated (bottom) AND error keep decreasing
        if (err > 0.0f) poi->int_term = new_int_term;
    }
    else {
        // If not saturated, accumulate integral
        poi->int_term = new_int_term;
    }

    return (uint16_t)output;
}

void motor_actuation(uint16_t ing_motor) {

	TIM4->CCR2 = ing_motor;
	TIM4->CCR3 = ing_motor;

}

void shutdown(void) {
	// Stop all critical actuators immediately
	motors_secure_turn_off();
	servos_turn_off();

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET); // Turn off LD1 (green led)
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET); // Turn on LD3 (red led)

	// Final system lock-up (Prevents code from wandering after shutdown)
	while (1) {
		// Halt system. Keeps power draw low. System requires a hard reset to restart.
		__WFI();
	}
}

void motors_secure_turn_off(void) {

	TIM4->CCR2 = CCR_VALUE_FOR_MOTOR_ACT;
	TIM4->CCR3 = CCR_VALUE_FOR_MOTOR_ACT;

	HAL_Delay(500); //0.5 s

	HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);
	HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_2);

}

/*-------------------------------------------------------------------------------------------------------*/
/*					  		 		  INTERRUPT FUNCTIONS				      					         */
/*-------------------------------------------------------------------------------------------------------*/

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {

	if (htim->Instance == TIM6) {
		// Tim 6 emits at 2 Hz
		HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_1); // Toggle LD2 (yellow led)
		current_number_of_toggles++;
	}

	if (htim->Instance == TIM7) {
		actuate_servo_control = true;
	}

}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	// When user button is pressed (used for safe startup)
	if (GPIO_Pin == GPIO_PIN_13) {
		run = !run;
	}

	// When VL53L1X data is ready, trigger PID
	if (GPIO_Pin == GPIO_PIN_12) {
		actuate_motors_control = true;
	}

}

// When RX pin reads something
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	// This is the usb cable
    if (huart->Instance == USART3)
    {
    	// At every message (with terminator), toggle run flag. This is useful for remote start/stop
		if (rx_byte == '\n')
		{
			run = !run;
		}
    }
}
