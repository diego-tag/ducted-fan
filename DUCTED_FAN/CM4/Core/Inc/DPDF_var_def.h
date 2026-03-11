/**********************************************************************************************************
 * _______________________________________________________________________________________________________
 *| @file DPDF_var_def.h																				  |																  |
 *| @brief This file containes the macros, variables and data structure used in the firmware sections     |
 *| 		  responible for motor, servo and PID management.											  |
 *| @version 0.2																				          |
 *| @date 27-10-2025																					  |
 *| ______________________________________________________________________________________________________|
 *********************************************************************************************************/

/*-------------------------------------------------------------------------------------------------------*/
/*					  		 			 INCLUDE DIRECTIVES 				      						 */
/*-------------------------------------------------------------------------------------------------------*/
#include "stdint.h"
#include "stdio.h"
#include "stdbool.h"
#include "stm32h7xx.h"

/*-------------------------------------------------------------------------------------------------------*/
/*					  		 			MACORS / DEFINITIONS 				      						 */
/*-------------------------------------------------------------------------------------------------------*/

/**
 * @def STANDARD_MESSAGE_LENGTH
 *
 * @brief this macro is used to specify the length of the message for the output
 */
#define STANDARD_MESSAGE_LENGTH															50

/*-------------------------------------------------------------------------------------------------------*/
/*					  		 			MOTOR-RELATED MACROS 				      						 */
/*-------------------------------------------------------------------------------------------------------*/

/**
 * @def NUMBER_OF_TOGGLES
 *
 * @brief This macro is used to specify (or set) the number of seconds before the firmware startup.
 * 		  It defines the number of the toggles of the LD3 on NUCLEO-H745ZIQ. The number of seconds
 * 		  is obtained by dividing this value by two.
 */
#define NUMBER_OF_TOGGLES																10

/**
 * @def CCR_VALUE_FOR_MOTOR_ACT
 *
 * @brief This macro specifies the value to be set in the compare register for motor arming. To arm
 * 		  the motor, the PWM signal must have a frequency of 50 Hz and a duty cycle of 4.5%. The
 * 		  value below was calculated considering a precaler of 99 and a period of 14999.
 */
#define CCR_VALUE_FOR_MOTOR_ACT 712

/* MOTOR PID */

/**
 * @def UPPER_LIMIT_TOP_MOTOR_SATURATION
 *
 * @brief This macro defines the upper limit for the compare register value used in PWM generation
 * 		  for the control of the motors.
 * 		  This limit is applied in the anti-windup filter of the motor PID controller. The value is
 * 		  calculated considering a prescaler of 99 and a period of 14999. The value below
 * 		  corresponds to a duty cycle of 12%
 *
 */
#define UPPER_LIMIT_TOP_MOTOR_SATURATION 900

/**
 * @def LOWER_LIMIT_TOP_MOTOR_SATURATION
 *
 * @brief This macro defines the lower limit for the compare register value used in PWM generation
 * 		  for the control of the motors.
 * 		  This limit is applied in the anti-windup filter of the motor PID controller. The value is
 * 		  calculated considering a prescaler of 99 and a period of 14999. The value below
 * 		  corresponds to a duty cycle of 6%.
 * 		  NOTE: the actual minimum value accepted by ESC is 758 which is the corresponds to 1000 µs (in the current configuration of the timer)
 */
#define LOWER_LIMIT_TOP_MOTOR_SATURATION 800

/*-------------------------------------------------------------------------------------------------------*/
/*					  		 			SERVOS-RELATED MACROS 				      					     */
/*-------------------------------------------------------------------------------------------------------*/

/**
 * @def UPPER_LIMIT_SERVO_SATURATION
 *
 * @brief This macro defines the upper limit for the compare register value used in PWM generation
 * 		  for the control of the servos.
 * 		  This limit is applied in the anti-windup filter of the servo PID controller. The value is
 * 		  calculated considering a frequency of 50 Hz, a prescaler of 99 and a period of 14999. The
 * 		  value below corresponds to a duty cycle of 10.3%.
 * 		  This value rotates the servo motor clockwise to its maximum range: 60°.
 *
 */
#define UPPER_LIMIT_SERVO_SATURATION													1545

/**
 * @def CENTER_UPPER_LOWER_SERVO_SATURATION
 *
 * @brief This macro defines the value to write into the compare register to set the PWM duty cycle
 * 		  corresponding to the servo position at 0°
 */
#define CENTER_UPPER_LOWER_SERVO_SATURATION												1125

/**
 * @def LOWER_LIMIT_SERVO_SATURATION
 *
 * @brief This macro defines the lower limit for the compare register value used in PWM generation
 * 		  for the control of the servos.
 * 		  This limit is applied in the anti-windup filter of the servo PID controller. The value is
 * 		  calculated considering a frequency of 50 Hz, a prescaler of 99 and a period of 14999. The
 * 		  value below corresponds to a duty cyle of 4,7 %.
 * 		  This value rotates the servo motor counter clockwise to its maximum range: -60°.
 */
#define LOWER_LIMIT_SERVO_SATURATION													705

/*-------------------------------------------------------------------------------------------------------*/
/*					  		 	     VARIABLES AND DATA STRUCTURES 				      					 */
/*-------------------------------------------------------------------------------------------------------*/

/**
 * @brief This data structure contains the initial angular values read from gyroscope, which are
 * 		  used to determine the origin of the drone's reference frame.
 */
typedef struct {

	uint32_t zero_rot_x;
	uint32_t zero_rot_y;
	uint32_t zero_rot_z;

} DPDF_axis_zero_rot_t;

/**
 * @brief This data structure is used to store the instantaneous rotation values around the
 * 		  Cartesian axes.
 */
typedef struct {

	int32_t rot_x;
	int32_t rot_y;
	int32_t rot_z;

} DPDF_axis_rot_t;

/**
 * @brief this struct contains the proportional, integral and derivative gains, as well as the
 * 		  sample time of the PID. The sampling time is expressed in milliseconds.
 */
typedef struct {
	float prop_coeff;
	float der_coeff;
	float int_coeff;
	float sampl_time;
	float err_old;
	float int_term;
} pid_prmts_t;

/**
 * @brief Pointer to the structure: DPDF_axis_zero_rot_t. It was created to improve code readability.
 */
typedef DPDF_axis_zero_rot_t *DPDF_zero_axis_rotation;

/**
 * @brief Pointer to the structure: DPDF_axis_rot_t. It was created to improve code readability.
 */
typedef DPDF_axis_rot_t *DPDF_axis_rotation;

/**
 * @brief Pointer to the structure: pid_prmts_t. It was created to improve code readability.
 */
typedef pid_prmts_t *pid_pars;

typedef struct {
	// Parameters
	float kp;
	float ki;
	float kd;
	float sample_time;

	// Controller state (memory)
	float int_term;
	float e_old;
} pid_controller_t;

