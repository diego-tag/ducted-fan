#ifndef INC_PWM_H_
#define INC_PWM_H_

#pragma once

#include <stdint.h>

/*-------------------------------------------------------------------------------------------------------*/
/*					  		 			SERVOS-RELATED FUNCTIONS			      					     */
/*-------------------------------------------------------------------------------------------------------*/

uint16_t angle_to_pwm(float target_angle_deg, uint16_t center_servo, float ccr_per_degree, uint16_t upper_limit, uint16_t lower_limit);

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
void servos_turn_off(uint16_t center_servo);

/*-------------------------------------------------------------------------------------------------------*/
/*					  		 			MOTORS-RELATED FUNCTIONS			      					     */
/*-------------------------------------------------------------------------------------------------------*/

/**
 * @brief This function is designed to safely start the system once it is placed in the cage.
 * 		  To start the system, the user must press the user button (PC13) on the
 * 		  NUCLEO-H745ZIQ. The delay time can be arbitrarily modified by changing number of toggles parameter
 *
 */
void safe_startup(uint16_t number_of_toggles);


/**
 * @brief This function stops the motors, returning the to their armed state, and then turns off
 * 		  their PWM control signals.
 */
void motors_secure_turn_off(uint16_t ccr_value_for_motor_act);


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

#endif /* INC_PWM_H_ */
