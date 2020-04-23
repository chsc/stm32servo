/*
 * This file is part of the stm32servo project.
 *
 * Copyright (C) 2020 Christoph Schunk <schunk.christoph@gmail.com>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * @file      servo.c
 * @author    Christoph Schunk
 * @copyright Copyright 2020 Christoph Schunk. All rights reserved.
 * @brief     Servo handling routines (mostly MCU agnostic).
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <memory.h>
#include "servo.h"

#define US_TO_TICKS(US) ((uint32_t)(US)*SERVO_TICKS_PER_MS / 1000)							 /**< Convert µs to phase index. */
#define TICKS_TO_US(PH) ((uint32_t)(PH)*1000 / SERVO_TICKS_PER_MS)							 /**< Convert a phase index to phase to µs. */
#define TICKS_MIN_DEFAULT_VALUE US_TO_TICKS(800)											 /**< Minimum value for servo phase. */
#define TICKS_MAX_DEFAULT_VALUE US_TO_TICKS(2600)											 /**< Maximum value for servo phase. */
#define TICKS_MIN_POSSIBLE_VALUE US_TO_TICKS(500)											 /**< Smalles possible phase value. */
#define TICKS_MAX_POSSIBLE_VALUE US_TO_TICKS(3000)											 /**< Bigges possible phase value. */
#define SERVO_TICKS_BUFFER_ENTRIES (TICKS_MAX_POSSIBLE_VALUE - TICKS_MIN_POSSIBLE_VALUE + 1) /**< Size of phase buffer */

#define TIME_LINE_BUFFER_SIZE 128 // not bigger than 256

#define TICKS_MIN_INDEX 0
#define TICKS_MAX_INDEX 1

#define CMD_SET_SERVO_TIMED 0x01
#define CMD_SET_ALL_SERVOS_TIMED 0x11
#define CMD_SET_SERVO 0x02
#define CMD_SET_ALL_SERVOS 0x12
#define CMD_ENABLE_SERVO 0x03
#define CMD_ENABLE_ALL_SERVOS 0x13
#define CMD_SET_CALIBRATION 0x04
#define CMD_SET_LED 0xa0

#define CMD_SET_LED_AUTO 3

#define USART_RING_BUFFER_SIZE 256

#define SET_PIN_IN_USE(I) (servo_port_set_pins[servo_port_map[I]] |= servo_pin_mask_map[I])
#define SET_PIN_NOT_IN_USE(I) (servo_port_set_pins[servo_port_map[I]] &= ~servo_pin_mask_map[I])

typedef enum parser_state
{
	parser_start,
	parser_set_servo,
	parser_set_all_servos,
	parser_set_servo_timed,
	parser_set_all_servos_timed,
	parser_enable_servo,
	parser_enable_all_servos,
	parser_calib_servo,
	parser_set_led
} parser_state_t;

uint8_t servo_port_map[SERVO_NUM_SERVOS];
servo_port_t servo_pin_mask_map[SERVO_NUM_SERVOS];
servo_port_t servo_port_set_pins[SERVO_NUM_PORTS];
servo_port_t servo_port_clear_pins[SERVO_TICKS_BUFFER_ENTRIES][SERVO_NUM_PORTS];
uint16_t time_ticks = 0;

uint16_t calib_servo_phases[SERVO_NUM_SERVOS][2];

uint16_t time_line_phases[SERVO_NUM_SERVOS][TIME_LINE_BUFFER_SIZE];
uint16_t time_line_timings[SERVO_NUM_SERVOS][TIME_LINE_BUFFER_SIZE];
uint8_t time_line_current_index[SERVO_NUM_SERVOS];
uint8_t time_line_append_index[SERVO_NUM_SERVOS];
uint16_t time_line_current_time[SERVO_NUM_SERVOS];
uint16_t time_line_prev_phase[SERVO_NUM_SERVOS];
uint16_t time_line_current_phase[SERVO_NUM_SERVOS];

volatile bool auto_led = true;

char usart_buffer[USART_RING_BUFFER_SIZE];
int usart_buffer_write_index = 0;
int usart_buffer_read_index = 0;

extern void servo_ext_set_pins(servo_port_t *ports);
extern void servo_ext_clear_pins(servo_port_t *ports);
extern void servo_ext_set_led(servo_led_state_t state);
extern void servo_lock();
extern void servo_unlock();

static uint16_t angle_to_phase_length(uint8_t servo_index, uint8_t angle)
{
	const uint32_t minp = calib_servo_phases[servo_index][TICKS_MIN_INDEX];
	const uint32_t maxp = calib_servo_phases[servo_index][TICKS_MAX_INDEX];
	return minp + ((maxp - minp) * angle) / SERVO_MAX_DEG;
}

static void prepare_set_pins()
{
	// reset all set pins
	for (int p = 0; p < SERVO_NUM_PORTS; p++)
	{
		servo_port_set_pins[p] = 0;
	}
}

static void prepare_clear_pins()
{
	// prepare pin state for each phase
	for (int t = 0; t < SERVO_TICKS_BUFFER_ENTRIES; t++)
	{
		const uint16_t tick = t + TICKS_MIN_POSSIBLE_VALUE;
		for (int p = 0; p < SERVO_NUM_PORTS; p++)
		{
			servo_port_clear_pins[t][p] = 0;
		}
		for (size_t s = 0; s < SERVO_NUM_SERVOS; s++)
		{
			if (tick > time_line_current_phase[s])
			{
				servo_port_clear_pins[t][servo_port_map[s]] |= servo_pin_mask_map[s];
			}
		}
	}
}

static void interpolate_pwm(uint8_t servo_index, uint16_t delta_time_ms)
{
	uint8_t k = time_line_append_index[servo_index];
	uint8_t *i = &time_line_current_index[servo_index];
	uint16_t *phases = time_line_phases[servo_index];
	uint16_t *timings = time_line_timings[servo_index];
	uint16_t *cur_phase = &time_line_current_phase[servo_index];
	uint16_t *cur_time = &time_line_current_time[servo_index];
	uint16_t *prev_phase = &time_line_prev_phase[servo_index];

	while (*i != k)
	{
		if (*cur_time < timings[*i])
		{
			*cur_phase = *prev_phase + (phases[*i] - *prev_phase) * *cur_time / timings[*i]; // interpolate phases
			*cur_time += delta_time_ms;
			return;
		}
		*cur_time -= timings[*i];
		*prev_phase = phases[*i];
		*i = (*i + 1) % TIME_LINE_BUFFER_SIZE; // next item
	}
	*cur_time = 0;
	*cur_phase = *prev_phase;
}

static void interpolate_all_pwms(uint16_t delta_time_ms)
{
	for (size_t servo_index = 0; servo_index < SERVO_NUM_SERVOS; servo_index++)
	{
		interpolate_pwm(servo_index, delta_time_ms);
	}
}

static void process_char(char ch)
{
	static parser_state_t state = parser_start;
	static int parser_offset = 0;
	static uint8_t servo_index, servo_angle;
	static uint16_t servo_timing, servo_min_phase, servo_max_phase;
	static uint8_t servo_angles[SERVO_NUM_SERVOS];
	static uint16_t servo_timings[SERVO_NUM_SERVOS];

	switch (state)
	{
	case parser_start:
		if (ch == CMD_SET_SERVO)
		{
			state = parser_set_servo;
		}
		else if (ch == CMD_SET_ALL_SERVOS)
		{
			state = parser_set_all_servos;
		}
		else if (ch == CMD_SET_SERVO_TIMED)
		{
			state = parser_set_servo_timed;
		}
		else if (ch == CMD_SET_ALL_SERVOS_TIMED)
		{
			state = parser_set_all_servos_timed;
		}
		else if (ch == CMD_ENABLE_ALL_SERVOS)
		{
			state = parser_enable_all_servos;
		}
		else if (ch == CMD_ENABLE_SERVO)
		{
			state = parser_enable_servo;
		}
		else if (ch == CMD_SET_CALIBRATION)
		{
			state = parser_calib_servo;
		}
		else if (ch == CMD_SET_LED)
		{
			state = parser_set_led;
		}
		else if (ch == 0xff)
		{
			// NOP
		}
		parser_offset = 0;
		return;
	case parser_set_servo:
		switch (parser_offset)
		{
		case 0:
			servo_index = ch;
			parser_offset++;
			return;
		case 1:
			servo_set_angle(servo_index, ch);
			state = parser_start;
			return;
		}
		return;
	case parser_set_all_servos:
		servo_angles[parser_offset++] = ch;
		if (parser_offset == SERVO_NUM_SERVOS)
		{
			servo_set_angle_all(servo_angles);
			state = parser_start;
		}
		return;
	case parser_set_servo_timed:
		switch (parser_offset)
		{
		case 0:
			servo_index = ch;
			parser_offset++;
			return;
		case 1:
			servo_angle = ch;
			parser_offset++;
			return;
		case 2:
			servo_timing = ch;
			parser_offset++;
			return;
		case 3:
			servo_timing |= (uint16_t)ch << 8;
			servo_set_angle_timed(servo_index, servo_angle, servo_timing);
			state = parser_start;
			return;
		}
		return;
	case parser_set_all_servos_timed:
		if (parser_offset < SERVO_NUM_SERVOS)
		{
			servo_angles[parser_offset++] = ch;
		}
		else
		{
			const int i = parser_offset - SERVO_NUM_SERVOS;
			switch (i % 2)
			{
			case 0:
				servo_timings[i / 2] = ch;
				parser_offset++;
				break;
			case 1:
				servo_timings[i / 2] |= (uint16_t)ch << 8;
				parser_offset++;
				break;
			}
			if (parser_offset == SERVO_NUM_SERVOS * 3)
			{
				servo_set_angle_timed_all(servo_angles, servo_timings);
				state = parser_start;
			}
		}
		return;
	case parser_enable_servo:
		switch (parser_offset)
		{
		case 0:
			servo_index = ch;
			parser_offset++;
			return;
		case 1:
			servo_enable(servo_index, ch);
			state = parser_start;
			return;
		}
		return;
	case parser_enable_all_servos:
		servo_enable_all(ch);
		state = parser_start;
		return;
	case parser_calib_servo:
		switch (parser_offset)
		{
		case 0:
			servo_index = ch;
			parser_offset++;
			return;
		case 1:
			servo_min_phase = ch;
			parser_offset++;
			return;
		case 2:
			servo_min_phase |= (uint16_t)ch << 8;
			parser_offset++;
			return;
		case 3:
			servo_max_phase = ch;
			parser_offset++;
			return;
		case 4:
			servo_max_phase |= (uint16_t)ch << 8;
			servo_set_calibration(servo_index, servo_min_phase, servo_max_phase);
			state = parser_start;
			return;
		}
		return;
	case parser_set_led:
		if (ch == CMD_SET_LED_AUTO)
		{
			auto_led = true;
		}
		else
		{
			servo_ext_set_led(ch);
			auto_led = false;
		}
		state = parser_start;
		return;
	}
}

static bool get_char_from_ringbuffer(char *ch)
{
	servo_lock();
	if (usart_buffer_write_index == usart_buffer_read_index)
	{
		servo_unlock();
		return false;
	}
	*ch = usart_buffer[usart_buffer_read_index++];
	usart_buffer_read_index %= USART_RING_BUFFER_SIZE;
	servo_unlock();
	return true;
}

static void process_usart_ringbuffer()
{
	char ch;
	while (get_char_from_ringbuffer(&ch))
	{
		process_char(ch);
	}
}

void servo_init(uint8_t *pmap, servo_port_t *pmasks)
{
	// copy servo pin mask and port map
	memcpy(servo_port_map, pmap, sizeof(servo_port_map));
	memcpy(servo_pin_mask_map, pmasks, sizeof(servo_pin_mask_map));

	for (size_t i = 0; i < SERVO_NUM_SERVOS; i++)
	{
		calib_servo_phases[i][TICKS_MIN_INDEX] = TICKS_MIN_DEFAULT_VALUE;
		calib_servo_phases[i][TICKS_MAX_INDEX] = TICKS_MAX_DEFAULT_VALUE;

		time_line_current_phase[i] = (TICKS_MIN_DEFAULT_VALUE + TICKS_MAX_DEFAULT_VALUE) / 2;
		time_line_prev_phase[i] = time_line_current_phase[i];
		time_line_current_time[i] = 0;
		time_line_append_index[i] = 0;
		time_line_current_index[i] = 0;
	}

	prepare_set_pins();
	prepare_clear_pins();
}

void servo_enable(uint8_t servo_index, bool state)
{
	if (servo_index >= SERVO_NUM_SERVOS)
		return;

	if (state)
	{
		SET_PIN_IN_USE(servo_index);
	}
	else
	{
		SET_PIN_NOT_IN_USE(servo_index);
	}
}

void servo_enable_all(bool state)
{
	if (state)
	{
		for (size_t i = 0; i < SERVO_NUM_SERVOS; i++)
		{
			SET_PIN_IN_USE(i);
		}
	}
	else
	{
		for (size_t i = 0; i < SERVO_NUM_SERVOS; i++)
		{
			SET_PIN_NOT_IN_USE(i);
		}
	}
}

void servo_set_calibration(uint8_t servo_index, uint16_t min_phase_us, uint16_t max_phase_us)
{
	if (servo_index >= SERVO_NUM_SERVOS)
		return;

	const uint16_t minp = US_TO_TICKS(min_phase_us);
	const uint16_t maxp = US_TO_TICKS(max_phase_us);

	calib_servo_phases[servo_index][TICKS_MIN_INDEX] = minp;
	calib_servo_phases[servo_index][TICKS_MAX_INDEX] = maxp;
}

void servo_set_angle_timed(uint8_t servo_index, uint8_t angle, uint16_t delta_time)
{
	if (servo_index >= SERVO_NUM_SERVOS)
		return;

	const uint8_t idx = time_line_append_index[servo_index];
	const uint16_t ap = angle_to_phase_length(servo_index, angle);
	const uint8_t nidx = (idx + 1) % TIME_LINE_BUFFER_SIZE;

	time_line_phases[servo_index][idx] = ap;
	time_line_timings[servo_index][idx] = delta_time;
	time_line_append_index[servo_index] = nidx;
}

void servo_set_angle_timed_all(uint8_t *angles, uint16_t *times)
{
	for (uint8_t i = 0; i < SERVO_NUM_SERVOS; i++)
	{
		const uint8_t idx = time_line_append_index[i];
		const uint16_t ap = angle_to_phase_length(i, angles[i]);
		const uint8_t nidx = (idx + 1) % TIME_LINE_BUFFER_SIZE;

		time_line_phases[i][idx] = ap;
		time_line_timings[i][idx] = times[i];
		time_line_append_index[i] = nidx;
	}
}

void servo_set_angle(uint8_t servo_index, uint8_t angle)
{
	if (servo_index >= SERVO_NUM_SERVOS)
		return;

	const uint16_t ap = angle_to_phase_length(servo_index, angle);

	time_line_phases[servo_index][0] = ap;
	time_line_timings[servo_index][0] = 0;
	time_line_append_index[servo_index] = 1;
	time_line_current_index[servo_index] = 0;
}

void servo_set_angle_all(uint8_t *angles)
{
	for (uint8_t i = 0; i < SERVO_NUM_SERVOS; i++)
	{
		const uint16_t ap = angle_to_phase_length(i, angles[i]);

		time_line_phases[i][0] = ap;
		time_line_timings[i][0] = 0;
		time_line_append_index[i] = 1;
		time_line_current_index[i] = 0;
	}
}

void servo_update(uint16_t delta_time_ms)
{
	process_usart_ringbuffer();
	interpolate_all_pwms(delta_time_ms);
	prepare_clear_pins();
	if (auto_led)
		servo_ext_set_led(2);
}

bool servo_generate_pwm()
{
	if (time_ticks == 0)
	{
		servo_ext_set_pins(servo_port_set_pins);
		time_ticks++;
		return true;
	}
	else if (time_ticks < TICKS_MIN_POSSIBLE_VALUE)
	{
		time_ticks++;
		return true;
	}
	else if (time_ticks <= TICKS_MAX_POSSIBLE_VALUE)
	{
		const int i = time_ticks - TICKS_MIN_POSSIBLE_VALUE;
		servo_ext_clear_pins(servo_port_clear_pins[i]);
		time_ticks++;
		return true;
	}
	else if (time_ticks < SERVO_TICKS_PER_MS * 20 - 1)
	{
		time_ticks++;
		return false;
	}
	time_ticks = 0;
	return false;
}

void servo_put_char_to_ring_buffer(char ch)
{
	servo_lock();
	usart_buffer[usart_buffer_write_index++] = ch;
	usart_buffer_write_index %= USART_RING_BUFFER_SIZE;
	if (usart_buffer_write_index == usart_buffer_read_index)
	{
		// overflow -> crash: buffer is to small!!!
		for (;;)
		{
			servo_ext_set_led(servo_led_toggle);
		}
	}
	servo_unlock();
}
