/**********************************************************************************************************
 * _______________________________________________________________________________________________________
 *| @file Motors_PID_and_manager.h																		  |																	  |
 *| @brief This file contains the function declaration used for motors control						      |
 *| @version 0.2																				          |
 *| @date 27-10-2025																					  |
 *| ______________________________________________________________________________________________________|
 *********************************************************************************************************/

/*-------------------------------------------------------------------------------------------------------*/
/*					  		 			 INCLUDE DIRECTIVES 				      						 */
/*-------------------------------------------------------------------------------------------------------*/

#include "Servos_PID_and_manager.h"
#include "vl53l1_api.h"
#include "math.h"

/*-------------------------------------------------------------------------------------------------------*/
/*					  		 			 FUNCTION DECLARATIONS 				      						 */
/*-------------------------------------------------------------------------------------------------------*/

/**
 * @brief This function is designed to safely start the system once it is placed in the cage.
 * 		  To start the system, the user must press the user button (PC13) on the
 * 		  NUCLEO-H745ZIQ. The delay time can be arbitrarily modified by changing the
 * 		  @ref NUMBER_OF_TOGGLES macro.
 *
 * @param tim_secure_start					: pointer to the HAL entity defining the timer
 * 											  responsible for the safe startup.
 *
 */
void safe_startup(TIM_HandleTypeDef *tim_secure_start);

/**
 * @brief This function computes the actual perpendicular ranging data. The function was created
 * 		  because the sensor must always measure the perpendicular distance to the ground,
 * 		  otherwise, it may provide incorrect data to @ref pid_motors.
 *
 * @param dev								: pointer to the data structure representing the
 * 											  handler and access point for the VL53L1X sensor.
 *
 * @param ranging_data 						: pointer to the data structure that contains the
 * 											  actual ranging data measurement.
 *
 * @param axis_zero_rot						: data structure containing the initial rotation info.
 *
 * @param axis_curr_rot						: data structure containing the current rotation info.
 *
 * @return uint16_t							: the return value of the function is the current
 * 											  perpendicular measured value.
 *
 */
uint16_t actual_perpendicular_ranging_data(VL53L1_DEV dev, VL53L1_RangingMeasurementData_t ranging_data, DPDF_zero_axis_rotation axis_zero_rot,
		DPDF_axis_rotation axis_curr_rot);

/**
 * @brief This function is used to set the PID controller parameters for the motor
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
 * @param sample_time 						: variable representing the sample_time of the motor
 * 											  PID controller.
 *
 * @param poi								: pointer to the data structure that contains the
 * 											  PID parameters for the motor PID controller.
 *
 */
void motors_pid_turner_and_turn_on(float kp, float ki, float kd, float t_c, pid_pars poi);

/**
 * @brief This function stops the motors, returning the to their armed state, and then turns off
 * 		  their PWM control signals.
 */
void motors_secure_turn_off(void);

void shutdown(void);

/**
 * @brief This function applies PID control theory to compute the input value to be written into
 * 		  the timer compare register responsible for generating the PWM signal.
 *
 * @param poi								: pointer to the data structure that contains the PID
 * 											  parameters.
 *
 * @param ranging_measurement				: variable representing the actual perpendicular
 * 											  ranging measurement.
 *
 * @param ref								: variable representing the reference altitude that
 * 											  the system must reach.
 *
 * @return uint16_t							: The return value of the function represents the value
 * 											  to be written to the compare register for actuation.
 *
 */
uint16_t pid_motors(pid_pars poi, int16_t ranging_measurement, uint16_t ref);

/**
 * @brief The following function implements the control command computed in the @ref pid_motors
 * 		  function. The General Purpose timer TIM4 CH3 is assigned to the top motor, whereas
 * 		  the general purpose timer TIM12 CH1 is assigned to the bottom motor.
 *
 * @param ing_motor 						: variable representing the input to the top and bottom
 * 											  motor.
 *
 */
void motor_actuation(uint16_t ing_motor);

/**
 * @brief HAL function used to catch NUCLEO-H745ZIQ GPIO interrupts. This function handles the
 * 		  interrupt generated by the user button: @ref safe_startup, as well as the interrupt sent
 * 		  directly by the altitude sensor (VL53L1X to NUCLEO-H745ZIQ PE7) indicating that new data
 * 		  are ready to be read.
 *
 * @param GPIO_Pin 							: variable representing the GPIO pin that triggered the
 * 											  interrupt
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

/**
 * @brief HAL function used to catch NUCLO-H745ZIQ TIM interrupts. This function handles the
 * 		  interrupt generated by TIM6, which is responsible for the @ref safe_start_up routine.
 * 		  It also manages the periodic interrupt by TIM7, which controls the timing of the servo
 * 		  motor operation.
 *
 * @ htim									: pointer to the data structure that represents the TIM
 * 										      that triggered the timing interrupt.
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

