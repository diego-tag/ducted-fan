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
 * @def UPPER_LIMIT_MOTOR
 *
 * @brief This macro defines the upper limit for the compare register value used in PWM generation
 * 		  for the control of the motors.
 * 		  The value is calculated considering a prescaler of 99 and a period of 14999. Since
 * 		  standard RC maximum throttle is ~2000 μsμs (2.0 ms), it corresponds to 10.0% of duty cycle.
 *
 */
#define UPPER_LIMIT_MOTOR 1500

/**
 * @def LOWER_LIMIT_MOTOR
 *
 * @brief This macro defines the lower limit for the compare register value used in PWM generation
 * 		  for the control of the motors.
 * 		  The value is calculated considering a prescaler of 99 and a period of 14999. Since
 * 		  standard RC minimum throttle: ~1000 μsμs (1.0 ms), it corresponds to 5.0% of duty cycle.
 */
#define LOWER_LIMIT_MOTOR 750

/*-------------------------------------------------------------------------------------------------------*/
/*					  		 			SERVOS-RELATED MACROS 				      					     */
/*-------------------------------------------------------------------------------------------------------*/

/**
 * @def UPPER_LIMIT_SERVO
 *
 * @brief This macro defines the upper limit for the compare register value used in PWM generation
 * 		  for the control of the servos.
 * 		  This limit is applied in the anti-windup filter of the servo PID controller. The value is
 * 		  calculated considering a frequency of 50 Hz, a prescaler of 99 and a period of 14999. The
 * 		  value below corresponds to a duty cycle of 10.3%.
 * 		  This value rotates the servo motor clockwise to its maximum range: 60°.
 *
 */
#define UPPER_LIMIT_SERVO													1545

/**
 * @def CENTER_SERVO
 *
 * @brief This macro defines the value to write into the compare register to set the PWM duty cycle
 * 		  corresponding to the servo position at 0°
 */
#define CENTER_SERVO												1125

/**
 * @def LOWER_LIMIT_SERVO
 *
 * @brief This macro defines the lower limit for the compare register value used in PWM generation
 * 		  for the control of the servos.
 * 		  This limit is applied in the anti-windup filter of the servo PID controller. The value is
 * 		  calculated considering a frequency of 50 Hz, a prescaler of 99 and a period of 14999. The
 * 		  value below corresponds to a duty cyle of 4,7 %.
 * 		  This value rotates the servo motor counter clockwise to its maximum range: -60°.
 */
#define LOWER_LIMIT_SERVO													705


/* Hardware Calibration */
#define CCR_PER_DEGREE      5.0f   /* Based on 75 CCR / 15° measurement */

/**
 * @def MAX_FLAP_ANGLE_DEG
 *
 * @brief This macro defines the ROM of servo motors in degrees for both directions. So total ROM is 120°.
 */
#define MAX_FLAP_ANGLE_DEG  60.0f  /* Maximum safe physical angle for the flap */


