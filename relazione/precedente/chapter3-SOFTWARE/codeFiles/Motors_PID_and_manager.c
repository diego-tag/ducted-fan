/**********************************************************************************************************
 * _______________________________________________________________________________________________________
 *| @file Motors_PID_and_manager.c																		  |
 *| @author Robert Laurentiu Mincu																		  |
 *| @brief This file contains the function definitions used for motors control						      |
 *| @version 0.2																				          |
 *| @date 27-10-2025																					  |
 *| ______________________________________________________________________________________________________|
 *********************************************************************************************************/


/*------------------------------------------------------------------*/
/*			              INCLUDE DIRECTIVES 	                    */
/*------------------------------------------------------------------*/

#include "Motors_PID_and_manager.h"


/*------------------------------------------------------------------*/
/*			        EXTERN VARIABLES DECLARATIONS	                */
/*------------------------------------------------------------------*/

extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim3;

extern uint8_t ld_st_loop_counter;
extern int flag;
extern flag_int;
extern flag_int_servo_control;
extern flag_int_motor_control;


/*------------------------------------------------------------------*/
/*			           FUNCTIONS DEFINITIONS	                    */
/*------------------------------------------------------------------*/


void safe_startup(TIM_HandleTypeDef* tim_secure_start){

	HAL_TIM_Base_Start_IT(tim_secure_start);

	while(ld_st_loop_counter <= NUMBER_OF_TOGGLES){}

	HAL_TIM_Base_Stop_IT(tim_secure_start);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);

}


float actual_perpendicular_ranging_data(VL53L1_DEV dev,
						 VL53L1_RangingMeasurementData_t ranging_data,
						 DPDF_zero_axis_rotation axis_zero_rot,
						 DPDF_axis_rotation axis_curr_rot){

	float perp_ran_data;
	float rad_roll;
	float rad_pitch;

	 	 VL53L1_GetRangingMeasurementData(dev,&ranging_data);
	 	 VL53L1_ClearInterruptAndStartMeasurement(dev);

	 	 DPDF_BNO055_firmware_read(axis_zero_rot, axis_curr_rot);

	 	 rad_roll = axis_curr_rot->rot_x*(M_PI/180);
	 	 rad_pitch = axis_curr_rot->rot_y*(M_PI/180);

	 	 perp_ran_data = ranging_data.RangeMilliMeter*
		 				 cos(rad_roll)*cos(rad_pitch);

	 return perp_ran_data;

}


void motors_pid_turner_and_turn_on(float kp,
								   float ki,
								   float kd,
								   float t_c,
								   pid_pars poi){

	poi->prop_coeff = kp;
	poi->int_coeff = kd;
	poi->der_coeff = ki;
	poi->sampl_time = t_c;

	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);

}


uint16_t pid_motors(pid_pars poi,
				   float  ranging_measurement,
			       uint16_t ref){

	static float err_old = 0;
	static float int_term = 0;
	float ing;
	float new_int_term;

	float err = ref-ranging_measurement;

	float p_term = poi->prop_coeff*err;

	new_int_term = int_term + (poi->int_coeff)*(poi->sampl_time)
				   *err_old;

	float d_term = (poi->der_coeff/poi->sampl_time)*(err-err_old);

	err_old = err;

	ing = p_term + new_int_term + d_term;

/*-------------- ANTI-WIND-UP FILTER, CLAMPING METHOD -------------*/

	if(ing > UPPER_LIMIT_TOP_MOTOR_SATURATION){

		ing = UPPER_LIMIT_TOP_MOTOR_SATURATION;

	}else if(ing < LOWER_LIMIT_TOP_MOTOR_SATURATION){

		ing = LOWER_LIMIT_TOP_MOTOR_SATURATION;
	}else{

		int_term = new_int_term;

	}

	return ing;

}

void motor_actuation(uint16_t ing_motor){

	TIM4->CCR3 = ing_motor;
	TIM12->CCR2 = ing_motor;

}

void motors_safe_turn_off(void){

	TIM4->CCR3 = CCR_VALUE_FOR_MOTOR_ACT;
	TIM12->CCR2 = CCR_VALUE_FOR_MOTOR_ACT;

	HAL_Delay(500); //0.5 s

	HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);
	HAL_TIM_PWM_Stop(&htim12, TIM_CHANNEL_2);

}


/*------------------------------------------------------------------*/
/*			      	     INTERRUPT FUNCTIONS                        */
/*------------------------------------------------------------------*/


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){

	if(htim==&htim6){
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
		ld_st_loop_counter++;

	}

	if(htim==&htim7){
		flag_int_servo_control = true;
	}

	if(htim==&htim3){
		flag_int_motor_control = true;
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == GPIO_PIN_13){
		if(flag == 0){
		flag = 1;
		}
	}
}











