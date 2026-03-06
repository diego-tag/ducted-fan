/**********************************************************************************************************
 * _______________________________________________________________________________________________________
 *| @file Servos_PID_and_manager.c																		  |
 *| @author Robert Laurentiu Mincu																		  |
 *| @brief This file contains the function definitions used for servo-motors control					  |
 *| @version 0.2																				          |
 *| @date 27-10-2025																					  |
 *| ______________________________________________________________________________________________________|
 *********************************************************************************************************/


/*------------------------------------------------------------------*/
/*			             INCLUDE DIRECTIVES 	                    */
/*------------------------------------------------------------------*/

#include "Servos_PID_and_manager.h"

/*------------------------------------------------------------------*/
/*			         EXTERN VARIABLES DECLARATIONS	                */
/*------------------------------------------------------------------*/


extern TIM_HandleTypeDef htim2;
 

/*------------------------------------------------------------------*/
/*			           FUNCTIONS DEFINITIONS	                    */
/*------------------------------------------------------------------*/



void DPDF_BNO055_firmware_read_init( 
							  DPDF_zero_axis_rotation axis_zero_rot){

	struct bno055_euler_float_t euler_data_init;

		bno055_convert_float_euler_hpr_deg(&euler_data_init);

		axis_zero_rot->zero_rot_x = (int)euler_data_init.r;
		axis_zero_rot->zero_rot_y = (int)euler_data_init.p;
		axis_zero_rot->zero_rot_z = (int)euler_data_init.h;
}


void DPDF_BNO055_firmware_read(DPDF_zero_axis_rotation init_axis_rot,
							   DPDF_axis_rotation ist_axis_rot){

struct bno055_euler_float_t euler_data_ist;

bno055_convert_float_euler_hpr_deg(&euler_data_ist);

ist_axis_rot->rot_x = (int)euler_data_ist.p-init_axis_rot->zero_rot_x;
ist_axis_rot->rot_y = (int)euler_data_ist.r-init_axis_rot->zero_rot_y;
ist_axis_rot->rot_z = (int)euler_data_ist.h-init_axis_rot->zero_rot_z;

}


void pid_servo_roll_turner_and_turn_on(float kp,
									   float ki, 
									   float kd, 
									   float sample_time, 
									  pid_pars pointer_pid_pars_roll){

		HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

		pointer_pid_pars_roll->prop_coeff = kp;
		pointer_pid_pars_roll->int_coeff = ki;
		pointer_pid_pars_roll->der_coeff = kd;
		pointer_pid_pars_roll->sampl_time = sample_time;

}


void pid_servo_pitch_turner_and_turn_on(float kp,
									    float ki,
									    float kd,
									    float sample_time,
									 pid_pars pointer_pid_pars_pitch){

		HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

		pointer_pid_pars_pitch->prop_coeff = kp;
		pointer_pid_pars_pitch->int_coeff = ki;
		pointer_pid_pars_pitch->der_coeff = kd;
		pointer_pid_pars_pitch->sampl_time = sample_time;

}


uint16_t pid_servo(pid_pars pid_pars_pointer,
				   int16_t ref,
				   int16_t angle_value){

	static float e_old = 0, int_term = 0;
	uint16_t  u;
	float new_int_term_cl_wi, new_int_term_co_cl_wi;


/*-------------------- COUNTERCLOCKWISE SECTION --------------------*/

	if(angle_value <= 0){

		int16_t e = ref-(-angle_value);

		float prop_term_cl_wi = pid_pars_pointer->prop_coeff*e;

		new_int_term_cl_wi = int_term + 
	   ((pid_pars_pointer->int_coeff*pid_pars_pointer->sampl_time)/2)*
		(e_old+e);

			float der_term_cl_wi = 
		   (pid_pars_pointer->der_coeff/pid_pars_pointer->sampl_time)*
			(e-e_old);

			e_old = e;

			u = CENTER_UPPER_LOWER_SERVO_SATURATION-
			(prop_term_cl_wi + new_int_term_cl_wi + der_term_cl_wi);

			/*ANTI WIND-UP FILTER*/

			if(u < LOWER_LIMIT_SERVO_SATURATION){

				u = LOWER_LIMIT_SERVO_SATURATION;

			}else if(u >CENTER_UPPER_LOWER_SERVO_SATURATION){

				u = CENTER_UPPER_LOWER_SERVO_SATURATION	;

			}else{

				int_term = new_int_term_cl_wi;

			}

			return u;

		}

/*----------------------- CLOCKWISE SECTION -----------------------*/

	if(angle_value > 0){

		int16_t e_2 = ref-angle_value;

		float prop_term_co_cl_wi = 
		pid_pars_pointer->prop_coeff*e_2;

		new_int_term_co_cl_wi = int_term +
	   ((pid_pars_pointer->int_coeff*pid_pars_pointer->sampl_time)/2)*
		(e_old+e_2);

			float der_term_co_cl_wi =
			(pid_pars_pointer->der_coeff/pid_pars_pointer->sampl_time)
			*(e_2-e_old);

			e_old = e_2;

			u = CENTER_UPPER_LOWER_SERVO_SATURATION+
	 (prop_term_co_cl_wi + new_int_term_co_cl_wi + der_term_co_cl_wi);

			/*ANTI WIND-UP FILTER*/

			if(u < CENTER_UPPER_LOWER_SERVO_SATURATION){

				u = CENTER_UPPER_LOWER_SERVO_SATURATION;

			}else if(u > UPPER_LIMIT_SERVO_SATURATION){

				u = UPPER_LIMIT_SERVO_SATURATION;

			}else{

				int_term = new_int_term_co_cl_wi;

			}
				return u;
		}

}


void execution_servo(uint16_t ingresso_roll, uint16_t ingresso_pitch){

	TIM2->CCR1 = ingresso_roll;
	TIM2->CCR2 = ingresso_pitch;
}


void servos_turn_off(void){

	TIM2->CCR1 = CENTER_UPPER_LOWER_SERVO_SATURATION;
	TIM2->CCR2 = CENTER_UPPER_LOWER_SERVO_SATURATION;

	HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim5, TIM_CHANNEL_5);

}






