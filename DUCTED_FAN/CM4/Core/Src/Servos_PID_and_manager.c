/**********************************************************************************************************
 * _______________________________________________________________________________________________________
 *| @file Servos_PID_and_manager.c																		  |																	  |
 *| @brief This file contains the function definitions used for servo-motors control					  |
 *| @version 0.2																				          |
 *| @date 27-10-2025																					  |
 *| ______________________________________________________________________________________________________|
 *********************************************************************************************************/

/*-------------------------------------------------------------------------------------------------------*/
/*					  		 			 INCLUDE DIRECTIVES 				      						 */
/*-------------------------------------------------------------------------------------------------------*/

#include "Servos_PID_and_manager.h"

/*-------------------------------------------------------------------------------------------------------*/
/*					  		 		EXTERN VARIABLES DECLARATIONS				      					 */
/*-------------------------------------------------------------------------------------------------------*/

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim5;

/*-------------------------------------------------------------------------------------------------------*/
/*					  		 		    FUNCTIONS DEFINITIONS				      					     */
/*-------------------------------------------------------------------------------------------------------*/

void DPDF_BNO055_firmware_read_init(DPDF_zero_axis_rotation axis_zero_rot) {

	struct bno055_euler_float_t euler_data_init;

	bno055_convert_float_euler_hpr_deg(&euler_data_init);

	axis_zero_rot->zero_rot_x = (int) euler_data_init.r;
	axis_zero_rot->zero_rot_y = (int) euler_data_init.p;
	axis_zero_rot->zero_rot_z = (int) euler_data_init.h;
}

void DPDF_BNO055_firmware_read(DPDF_zero_axis_rotation init_axis_rot, DPDF_axis_rotation ist_axis_rot) {

	struct bno055_euler_float_t euler_data_ist;

	bno055_convert_float_euler_hpr_deg(&euler_data_ist);

	ist_axis_rot->rot_x = (int) euler_data_ist.p - init_axis_rot->zero_rot_x;
	ist_axis_rot->rot_y = (int) euler_data_ist.r - init_axis_rot->zero_rot_y;
	ist_axis_rot->rot_z = (int) euler_data_ist.h - init_axis_rot->zero_rot_z;
}

void pid_servo_roll_turner_and_turn_on(float kp, float ki, float kd, float sample_time, pid_pars pointer_pid_pars_roll) {

	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

	pointer_pid_pars_roll->prop_coeff = kp;
	pointer_pid_pars_roll->int_coeff = ki;
	pointer_pid_pars_roll->der_coeff = kd;
	pointer_pid_pars_roll->sampl_time = sample_time;

}

void pid_servo_pitch_turner_and_turn_on(float kp, float ki, float kd, float sample_time, pid_pars pointer_pid_pars_pitch) {

	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

	pointer_pid_pars_pitch->prop_coeff = kp;
	pointer_pid_pars_pitch->int_coeff = ki;
	pointer_pid_pars_pitch->der_coeff = kd;
	pointer_pid_pars_pitch->sampl_time = sample_time;

}

void pid_servo_init(pid_controller_t *pid, float kp, float ki, float kd, float sample_time) {
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
	pid->sample_time = sample_time;
	pid->int_term = 0.0f;
	pid->e_old = 0.0f;
}

uint16_t pid_servo_compute(pid_controller_t *pid, float ref, float angle_value) {

	// Compute error
	float error = ref - angle_value;

	// Proportional
	float p_term = pid->kp * error;

	// Integral (Euler forward method)
	float i_term_unclamped = pid->int_term + (pid->ki * pid->sample_time * error);

	// Derivative (backward difference)
	float d_term = (pid->kd / pid->sample_time) * (error - pid->e_old);

	// Total effort
	float control_effort = p_term + i_term_unclamped + d_term;

	// Consider offset
	float u_out = CENTER_UPPER_LOWER_SERVO_SATURATION + control_effort;

	// Anti-Windup (Clamping method) and saturation
	uint16_t final_pwm;

	if (u_out > UPPER_LIMIT_SERVO_SATURATION) {
		final_pwm = UPPER_LIMIT_SERVO_SATURATION;
		// Conditional integration: update integral only if error help with saturation
		if (error < 0)
			pid->int_term = i_term_unclamped;

	} else if (u_out < LOWER_LIMIT_SERVO_SATURATION) {
		final_pwm = LOWER_LIMIT_SERVO_SATURATION;
		// Conditional integration: update integral only if error help with saturation
		if (error > 0)
			pid->int_term = i_term_unclamped;

	} else {
		// Linear
		final_pwm = (uint16_t) u_out;
		pid->int_term = i_term_unclamped; // Normal update
	}

	// Save error for the computation in the next loop
	pid->e_old = error;

	return final_pwm;
}

void execution_servo(uint16_t ingresso_roll, uint16_t ingresso_pitch) {

	TIM2->CCR1 = ingresso_roll;
	TIM2->CCR2 = ingresso_pitch;
}

void servos_turn_off(void) {

	TIM2->CCR1 = CENTER_UPPER_LOWER_SERVO_SATURATION;
	TIM2->CCR2 = CENTER_UPPER_LOWER_SERVO_SATURATION;

	HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim5, TIM_CHANNEL_5);

}

