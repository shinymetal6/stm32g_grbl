/*
 * cpu_map_stm32g.h
 *
 *  Created on: Apr 29, 2020
 *      Author: fil
 */

#ifndef GRBL_CPU_MAP_STM32G_H_
#define GRBL_CPU_MAP_STM32G_H_

#define NOEEPROMSUPPORT
extern	UART_HandleTypeDef huart2;

#define	UART_PORT	USART2
#define	UART_PORT_H	&huart2

// Define step pulse output pins. NOTE: All step bit pins must be on the same port.
#define STEP_PORT GPIOA
#define RCC_STEP_PORT RCC_APB2Periph_GPIOA
#define X_STEP_BIT X_STEP_BIT_Pin
#define Y_STEP_BIT Y_STEP_BIT_Pin
#define Z_STEP_BIT Z_STEP_BIT_Pin
#define STEP_MASK ((1 << X_STEP_BIT) | (1 << Y_STEP_BIT) | (1 << Z_STEP_BIT)) // All step bits

// Define step direction output pins. NOTE: All direction pins must be on the same port.
#define DIRECTION_PORT GPIOA
#define RCC_DIRECTION_PORT RCC_APB2Periph_GPIOA
#define X_DIRECTION_BIT X_DIRECTION_BIT_Pin
#define Y_DIRECTION_BIT Y_DIRECTION_BIT_Pin
#define Z_DIRECTION_BIT Z_DIRECTION_BIT_Pin
#define DIRECTION_MASK ((1 << X_DIRECTION_BIT) | (1 << Y_DIRECTION_BIT) | (1 << Z_DIRECTION_BIT)) // All direction bits

// Define stepper driver enable/disable output pin.
#define STEPPERS_DISABLE_PORT STEPPERS_DISABLE_BIT_GPIO_Port
//#define RCC_STEPPERS_DISABLE_PORT RCC_APB2Periph_GPIOA
#define STEPPERS_DISABLE_BIT STEPPERS_DISABLE_BIT_Pin
#define STEPPERS_DISABLE_MASK (1 << STEPPERS_DISABLE_BIT)
#define SetStepperDisableBit() GPIO_SetBits(STEPPERS_DISABLE_PORT, STEPPERS_DISABLE_MASK)
#define ResetStepperDisableBit() GPIO_ResetBits(STEPPERS_DISABLE_PORT, STEPPERS_DISABLE_MASK)

// Define homing/hard limit switch input pins and limit interrupt vectors.
// NOTE: All limit bit pins must be on the same port
#define LIMIT_PIN GPIOB
#define LIMIT_PORT GPIOB
//#define RCC_LIMIT_PORT RCC_APB2Periph_GPIOB
//#define GPIO_LIMIT_PORT GPIO_PortSourceGPIOB
#define X_LIMIT_BIT X_LIMIT_BIT_Pin
#define Y_LIMIT_BIT Y_LIMIT_BIT_Pin
#define Z_LIMIT_BIT Z_LIMIT_BIT_Pin

#define LIMIT_MASK ((1 << X_LIMIT_BIT) | (1 << Y_LIMIT_BIT) | (1 << Z_LIMIT_BIT)) // All limit bits

// Define spindle enable and spindle direction output pins.
#define SPINDLE_ENABLE_PORT GPIOB
//#define RCC_SPINDLE_ENABLE_PORT RCC_APB2Periph_GPIOB
#define SPINDLE_ENABLE_BIT 13 //
#ifndef USE_SPINDLE_DIR_AS_ENABLE_PIN
#define SPINDLE_DIRECTION_DDR GPIOB
#define SPINDLE_DIRECTION_PORT GPIOB
#define SPINDLE_DIRECTION_BIT 14 //
#endif
#define SetSpindleEnablebit() HAL_GPIO_WritePin(SPINDLE_ENABLE_PORT, 1 << SPINDLE_ENABLE_BIT, GPIO_PIN_SET)
#define ResetSpindleEnablebit() HAL_GPIO_WritePin(SPINDLE_ENABLE_PORT, 1 << SPINDLE_ENABLE_BIT, GPIO_PIN_RESET)
#define SetSpindleDirectionBit() HAL_GPIO_WritePin(SPINDLE_DIRECTION_PORT, 1 << SPINDLE_DIRECTION_BIT, GPIO_PIN_SET)
#define ResetSpindleDirectionBit() HAL_GPIO_WritePin(SPINDLE_DIRECTION_PORT, 1 << SPINDLE_DIRECTION_BIT, GPIO_PIN_RESET)

// Define flood and mist coolant enable output pins.
// a later date if flash and memory space allows.
#define COOLANT_FLOOD_PORT GPIOB
//#define RCC_COOLANT_FLOOD_PORT RCC_APB2Periph_GPIOB
#define COOLANT_FLOOD_BIT 3
#define COOLANT_MIST_PORT GPIOB
//#define RCC_COOLANT_MIST_PORT RCC_APB2Periph_GPIOB
#define COOLANT_MIST_BIT 4

// Define user-control controls (cycle start, reset, feed hold) input pins.
// NOTE: All CONTROLs pins must be on the same port and not on a port with other input pins (limits).
#define CONTROL_PIN_PORT GPIOB
#define CONTROL_PORT GPIOB
//#define RCC_CONTROL_PORT RCC_APB2Periph_GPIOB
//#define GPIO_CONTROL_PORT GPIO_PortSourceGPIOB
#define CONTROL_RESET_BIT 5
#define CONTROL_FEED_HOLD_BIT 6
#define CONTROL_CYCLE_START_BIT 7
#define CONTROL_SAFETY_DOOR_BIT 8
#define CONTROL_MASK ((1 << CONTROL_RESET_BIT) | (1 << CONTROL_FEED_HOLD_BIT) | (1 << CONTROL_CYCLE_START_BIT) | (1 << CONTROL_SAFETY_DOOR_BIT))

// Define probe switch input pin.
#define PROBE_PORT PROBE_BIT_GPIO_Port
//#define RCC_PROBE_PORT RCC_APB2Periph_GPIOA
#define PROBE_BIT PROBE_BIT_Pin
#define PROBE_MASK (1 << PROBE_BIT)

// Start of PWM & Stepper Enabled Spindle
#ifdef VARIABLE_SPINDLE
#define	SPINDLE_TIMER	TIM2
#define	SPINDLE_TIMER_H			&htim2
#define	SPINDLE_TIMER_CHANNEL	TIM_CHANNEL_2

// NOTE: On the 328p, these must be the same as the SPINDLE_ENABLE settings.
#define SPINDLE_PWM_FREQUENCY 10000 // KHz
#define SPINDLE_PWM_DDR SPINDLE_PWM_BIT_GPIO_Port
#define SPINDLE_PWM_PORT SPINDLE_PWM_BIT_GPIO_Port
//#define RCC_SPINDLE_PWM_PORT RCC_APB2Periph_GPIOA
#define SPINDLE_PWM_BIT SPINDLE_PWM_BIT_Pin
#endif // End of VARIABLE_SPINDLE
#define SPINDLE_PWM_MAX_VALUE (1000000 / SPINDLE_PWM_FREQUENCY)
#ifndef SPINDLE_PWM_MIN_VALUE
#define SPINDLE_PWM_MIN_VALUE 1 // Must be greater than zero.
#endif
#define SPINDLE_PWM_OFF_VALUE 0
#define SPINDLE_PWM_RANGE (SPINDLE_PWM_MAX_VALUE - SPINDLE_PWM_MIN_VALUE)

//  Port A                                         Port B
//   0      X_STEP_BIT
//   1      Y_STEP_BIT
//   2      Z_STEP_BIT
//   3      X_DIRECTION_BIT                       COOLANT_FLOOD_BIT
//   4      Y_DIRECTION_BIT                       COOLANT_MIST_BIT
//   5      Z_DIRECTION_BIT                       CONTROL_RESET_BIT
//   6      STEPPERS_DISABLE_BIT                  CONTROL_FEED_HOLD_BIT
//   7                                            CONTROL_CYCLE_START_BIT
//   8      SPINDLE_PWM_BIT                       CONTROL_SAFETY_DOOR_BIT
//   9
//   10                                            X_LIMIT_BIT
//   11                                            Y_LIMIT_BIT
//   12                                            Z_LIMIT_BIT
//   13 14 SWD																		SPINDLE_ENABLE_BIT
//     14																						SPINDLE_DIRECTION_BIT
//   15     PROBE_BIT


#endif /* GRBL_CPU_MAP_STM32G_H_ */
