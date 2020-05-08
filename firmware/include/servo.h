/*
 * This file is part of the stm32servo project.
 *
 * Copyright (C) 2020 Christoph Schunk <schunk.christoph@gmail.com>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this library. If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * @file      servo.h
 * @author    Christoph Schunk
 * @copyright Copyright 2020 Christoph Schunk. All rights reserved.
 * @brief     MCU agnostic servo handling routines.
 */

#ifndef SERVO_H
#define SERVO_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

/**
 * @mainpage Servo controlling routines
 * @section pageTOC Modules
 * @ref SERVO
 */

/**
 * @defgroup SERVO Servo routines
 * @brief Servo controlling routines
 *
 * How to use the servo controller routines
 * ========================================
 *
 * Initialization
 * --------------
 * You have to assign a port map and a pin mask for each servo pin:
 * @code
 *  uint8_t      portmap  = {SERVO_PORT_A_INDEX, SERVO_PORT_A_INDEX, ..., SERVO_PORT_B_INDEX, ...};
 *  servo_port_t pinmasks = {SERVO_PIN_INDEX(0), SERVO_PIN_INDEX(1), ..., SERVO_PIN_INDEX(0), ...};
 *  servo_init(portmap, pinmasks);
 * @endcode
 *
 * Update interrupt
 * ----------------
 * @code
 *  servo_update(20); // updates the servo_port_pins array
 * @endcode
 *
 * PWM Generation
 * --------------
 * @code
 * bool is_generating_pwm;
 * is_generating_pwm = servo_generate_pwm();
 * @endcode
 *
 * UART Commands
 * -------------
 * @code
 * char data = read_uart(),
 * servo_put_char_to_ringbuffer(data);
 * @endcode
 * @{
 */

#define SERVO_NUM_SERVOS 24 /**< Number of supported servos. 24 servos is currently enough. */
#define SERVO_NUM_PORTS 2   /**< Number of ports supported. Two 16 bit ports are enough for 24 servos. */

#define SERVO_TICKS_PER_MS 180 /**< Sampling for a PWM servo signal. */

#define SERVO_PORT_A_INDEX 0              /**< Index for port A. */
#define SERVO_PORT_B_INDEX 1              /**< Index for port B. */
#define SERVO_PORT_C_INDEX 2              /**< Index for port C. */
#define SERVO_PORT_D_INDEX 3              /**< Index for port D. */
#define SERVO_PIN_INDEX(IDX) (1 << (IDX)) /**< Pin index for a port */

typedef uint16_t servo_port_t; /**< We use 16 bit ports. Typical for STM32. */

typedef enum servo_led_state_s
{
    servo_led_off = 0,
    servo_led_on = 1,
    servo_led_toggle = 2
} servo_led_state_t;

/**
 * @brief servo_init initializes the servo controller
 * @param port_map An array of indices mapping a servo pin to a port
 * @param pin_masks An array of GPIO pin masks for each servo index
 */
void servo_init(uint8_t *port_map, servo_port_t *pin_masks);

/**
 * @brief servo_enable enables a or disbales a single servo
 * @param servo_index The index of the servo to set.
 * @param state Enable or disable the servo.
 */
void servo_enable(uint8_t servo_index, bool state);

/**
 * @brief servo_enable_all
 * @param state Enable or disable all servos.
 */
void servo_enable_all(bool state);

/**
 * @brief servo_set_calibration
 * @param servo_index The index of the servo to set.
 * @param min_phase_us Minimum phase value for 0°
 * @param max_phase_us Maximum phase value for 180°
 */
void servo_set_calibration(uint8_t servo_index, uint8_t min_angle, uint8_t max_angle, uint16_t min_phase_us, uint16_t max_phase_us);

/**
 * @brief servo_set_angle_timed
 * @param servo_index The index of the servo to set
 * @param angle The angle.
 * @param time The time in ms.
 */
void servo_set_angle_timed(uint8_t servo_index, uint8_t angle, uint16_t time);

/**
 * @brief servo_set_angle_timed_all
 * @param angles Angles to set.
 * @param times The time ein ms.
 */
void servo_set_angle_timed_all(uint8_t *angles, uint16_t *times);

/**
 * @brief servo_set_angle
 * @param servo_index The index of the servo to set.
 * @param angle The angle.
 */
void servo_set_angle(uint8_t servo_index, uint8_t angle);

/**
 * @brief servo_set_angle_all
 * @param angle Angles to set.
 */
void servo_set_angle_all(uint8_t *angles);

/**
 * @brief servo_update linear interpolates all phases and generates a pin mask that can be used
 *        to generate a PWM signal.
 * @param delta_time Delta time, usually 20 ms.
 */
void servo_update(uint16_t delta_time);

/**
 * @brief Generates a PWM signal. Must be called every 1/180 of a millisecond.
 * @return true Is currently generating a PWM signal.
 * @return false Is currenly not generating a PWM signal.
 */
bool servo_generate_pwm();

/**
 * @brief Puts a character in the ring buffer
 * @param ch Character to put
 */
void servo_put_char_to_rx_buffer(char ch);

/**
 * @brief GEts a character in the tx ring buffer
 * @param ch Character to put
 */
bool servo_get_char_from_tx_buffer(char *ch);

/** @}*/

#endif // SERVO_H
