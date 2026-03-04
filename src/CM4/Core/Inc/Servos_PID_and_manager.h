/**********************************************************************************************************
 * _______________________________________________________________________________________________________
 *| @file Servos_PID_and_manager.h																		  |
 *| @author Robert Laurentiu Mincu																		  |
 *| @brief This file contains the function declaration used for servo-motors control					  |
 *| @version 0.2																				          |
 *| @date 27-10-2025																					  |
 *| ______________________________________________________________________________________________________|
 *********************************************************************************************************/


/*-------------------------------------------------------------------------------------------------------*/
/*					  		 			 INCLUDE DIRECTIVES 				      						 */
/*-------------------------------------------------------------------------------------------------------*/


#include "DPDF_var_def.h"
#include "bno055.h"
#include "bno055_stm32_hal.h"

/*-------------------------------------------------------------------------------------------------------*/
/*					  		 			 FUNCTION DECLARATIONS 				      						 */
/*-------------------------------------------------------------------------------------------------------*/



/**
 * @brief This function extracts the initial rotation data around the axes, which are used to
 * 		  determine the reference frame. The @ref bno055_convert_float_euler_hpr_deg function
 * 		  is declered and defined in the @ref bno055.c file.
 *
 * @param axis_zero_rot 					: data structure containing the initial rotation info.
 */
void DPDF_BNO055_firmware_read_init(DPDF_zero_axis_rotation axis_zero_rot);


/**
 * @brief This function is used to extract the current rotation value around the axes.
 * 		  The @ref bno055_convert_float_euler_hpr_deg function is declared and defined in the
 * 		  @ref bno055.c file.
 *
 * @param axis_zero_rot						: data structure containing the initial rotation info.
 *
 * @param init_axis_rot						: data structure containing the current rotation info.
 */
void DPDF_BNO055_firmware_read(DPDF_zero_axis_rotation init_axis_rot, DPDF_axis_rotation ist_axis_rot);


/**
 * @brief This function is used to set the PID controller parameters for the servo motor
 * 		  responsible for rotation around the x-axis (roll).
 *
 * @param kp								: variable representing the proportional term
 * 											  coefficient.
 *
 * @param ki								: variable representing the integral term
 * 											  coefficient.
 *
 * @param kd								: variable representing the derivative term
 * 											  coefficient.
 *
 * @param sample_time 						: variable representing the sample_time of
 * 											  PID controller.
 *
 * @param pointer_pid_pars_roll				: pointer to the data structure that contains the
 * 											  PID parameters for the pitch servo PID controller.
 */
void pid_servo_roll_turner_and_turn_on(float kp, float ki, float kd, float sample_time, pid_pars pointer_pid_pars_roll);


/**
 * @brief This function is used to set the PID controller parameters for the servo motor
 * 		  responsible for rotation around the y-axis (pitch)
 *
 * @param kp								: variable representing the proportional term
 * 											  coefficient.
 *
 * @param ki								: variable representing the integral term
 * 											  coefficient.
 *
 * @param kd								: variable representing the derivative term
 * 											  coefficient.
 *
 * @param sample_time						: variable representing the sample_time of
 * 											  PID controller.
 *
 * @param pointer_pid_pars_pitch			: pointer to the data structure that contains the
 * 											  PID parameters for the pitch servo PID controller.
 *
 */
void pid_servo_pitch_turner_and_turn_on(float kp, float ki, float kd, float sample_time, pid_pars pointer_pid_pars_pitch);


/**
 * @brief This function applies PID control theory to compute the input value to be written into
 * 		  the timer compare register responsible for generating the PWM signal. The function is
 * 		  divided in two parts: one handling clockwise rotation and the other handling
 * 		  counterclockwise rotation.
 *
 * @param pid_pars_pointer					: pointer to the data structure that contains the
 * 											  PID parameters.
 *
 * @param ref								: variable representing the reference angle (degrees)
 * 											  that the servo motors must follow.
 *
 * @param angle_value						: variable representing the current angle read from
 * 											  the gyroscope.
 *
 * @return uint16_t							: The return value of the function represents the value
 * 											  to be written to the compare register for actuation.
 */
uint16_t pid_servo(pid_pars pid_pars_pointer, int16_t ref, int16_t angle_value);

/**
 * @brief The following function implements the control command computed in the @ref pid_servo
 * 		  function. The General Purpose timer TIM2 CH1 is assigned to the roll servo, whereas
 * 		  the general purpose timer TIM5 CH4 is assigned to the pitch servo.
 *
 * @param ing_roll 							: variable representing the input to the roll servo
 * 											  motor.
 *
 * @param ing_pitch							: variable representing the input to the pitch servo
 * 											  motor.
 */
void execution_servo(uint16_t ing_roll, uint16_t ing_pitch);

/**
 * @brief This function sets the servos to their central position (0 degrees) and then stops the
 * 		  generation of the PWM control signal.
 */
void servos_turn_off(void);

