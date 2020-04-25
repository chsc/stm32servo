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

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/sync.h>
#include "servo.h"

#define LED_PORT (GPIOC)
#define LED_PIN (GPIO13)

mutex_t mtx;
volatile bool is_generating_pwm = false;

static void clock_setup()
{
	rcc_clock_setup_in_hse_8mhz_out_72mhz();

	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_GPIOC);

	rcc_periph_clock_enable(RCC_AFIO); // alternate function clock
	rcc_periph_clock_enable(RCC_USART1);
}

static void gpio_setup(void)
{
	// disable JTAG on pin PB3, PB4 and PA15 to use them as normal outputs
	AFIO_MAPR |= AFIO_MAPR_SWJ_CFG_JTAG_OFF_SW_ON;

	// servo pins
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO0 | GPIO1 | GPIO2 | GPIO3 | GPIO4 | GPIO5 | GPIO6 | GPIO7 | GPIO8 | GPIO9 | GPIO10 | GPIO11 | GPIO12 | GPIO15);
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO0 | GPIO1 | GPIO3 | GPIO4 | GPIO5 | GPIO8 | GPIO9 | GPIO10 | GPIO11 | GPIO15);

	// led pin
	gpio_set_mode(LED_PORT, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, LED_PIN);
}

static void usart_setup(void)
{
	// Enable USART1 RX Int
	nvic_enable_irq(NVIC_USART1_IRQ);

	// Enable USART1 pin software remapping.
	AFIO_MAPR |= AFIO_MAPR_USART1_REMAP;

	//Configure USART1 RX on PB11
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,
				  GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART1_RE_TX);

	gpio_set_mode(GPIOB, GPIO_MODE_INPUT,
				  GPIO_CNF_INPUT_FLOAT, GPIO_USART1_RE_RX);

	usart_set_baudrate(USART1, 115200);
	usart_set_databits(USART1, 8);
	usart_set_stopbits(USART1, USART_STOPBITS_1);
	usart_set_mode(USART1, USART_MODE_TX_RX);
	usart_set_parity(USART1, USART_PARITY_NONE);
	usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);

	// Enable USART2 RX Int
	USART_CR1(USART1) |= USART_CR1_RXNEIE;

	usart_enable(USART1);
}

void systick_setup()
{
	/* 72 MHz / 8 = 9000000 counts per second */
	systick_set_clocksource(STK_CSR_CLKSOURCE_AHB_DIV8);

	/* 9000000/50 = 180 overflows per millisecond */
	/* SysTick interrupt every 50-1 */
	systick_set_reload(49);

	systick_interrupt_enable();

	systick_counter_enable();
}

static void servo_setup()
{
	uint8_t portMap[SERVO_NUM_SERVOS] = {
		SERVO_PORT_A_INDEX,
		SERVO_PORT_A_INDEX,
		SERVO_PORT_A_INDEX,
		SERVO_PORT_A_INDEX,
		SERVO_PORT_A_INDEX,
		SERVO_PORT_A_INDEX,
		SERVO_PORT_A_INDEX,
		SERVO_PORT_A_INDEX,

		SERVO_PORT_B_INDEX,
		SERVO_PORT_B_INDEX,
		SERVO_PORT_B_INDEX,
		SERVO_PORT_B_INDEX,
		SERVO_PORT_B_INDEX,
		SERVO_PORT_B_INDEX,
		SERVO_PORT_B_INDEX,
		SERVO_PORT_B_INDEX,
		SERVO_PORT_B_INDEX,

		SERVO_PORT_A_INDEX,
		SERVO_PORT_A_INDEX,
		SERVO_PORT_A_INDEX,
		SERVO_PORT_A_INDEX,
		SERVO_PORT_A_INDEX,
		SERVO_PORT_A_INDEX,

		SERVO_PORT_B_INDEX};

	servo_port_t pinMasks[SERVO_NUM_SERVOS] = {
		SERVO_PIN_INDEX(0), // Port A ...
		SERVO_PIN_INDEX(1),
		SERVO_PIN_INDEX(2),
		SERVO_PIN_INDEX(3),
		SERVO_PIN_INDEX(4),
		SERVO_PIN_INDEX(5),
		SERVO_PIN_INDEX(6),
		SERVO_PIN_INDEX(7),
		SERVO_PIN_INDEX(0), // Port B
		SERVO_PIN_INDEX(1),
		SERVO_PIN_INDEX(10),
		SERVO_PIN_INDEX(11),
		SERVO_PIN_INDEX(9),
		SERVO_PIN_INDEX(8),
		SERVO_PIN_INDEX(5),
		SERVO_PIN_INDEX(4),
		SERVO_PIN_INDEX(3),
		SERVO_PIN_INDEX(15), // Port A
		SERVO_PIN_INDEX(12),
		SERVO_PIN_INDEX(11),
		SERVO_PIN_INDEX(10),
		SERVO_PIN_INDEX(9),
		SERVO_PIN_INDEX(8),
		SERVO_PIN_INDEX(15) // Port B
	};

	servo_init(portMap, pinMasks);
}

void servo_ext_set_pins(servo_port_t *set_pins)
{
	gpio_set(GPIOA, set_pins[SERVO_PORT_A_INDEX]);
	gpio_set(GPIOB, set_pins[SERVO_PORT_B_INDEX]);
}

void servo_ext_clear_pins(servo_port_t *set_pins)
{
	gpio_clear(GPIOA, set_pins[SERVO_PORT_A_INDEX]);
	gpio_clear(GPIOB, set_pins[SERVO_PORT_B_INDEX]);
}

void servo_ext_set_led(servo_led_state_t state)
{
	switch (state)
	{
	case servo_led_off:
		gpio_set(LED_PORT, LED_PIN);
		break;
	case servo_led_on:
		gpio_clear(LED_PORT, LED_PIN);
		break;
	case servo_led_toggle:
		gpio_toggle(LED_PORT, LED_PIN);
		break;
	}
}

void servo_lock()
{
	mutex_lock(&mtx);
}

void servo_unlock()
{
	mutex_unlock(&mtx);
}

void sys_tick_handler(void)
{
	is_generating_pwm = servo_generate_pwm();
}

void usart1_isr(void)
{
	static uint8_t data = 0xff;
	if (((USART_CR1(USART1) & USART_CR1_RXNEIE) != 0) && ((USART_SR(USART1) & USART_SR_RXNE) != 0))
	{
		data = usart_recv(USART1);
		servo_put_char_to_ring_buffer(data);
	}
}

int main(void)
{
	clock_setup();
	servo_setup();
	gpio_setup();
	usart_setup();
	systick_setup();

	for (;;)
	{
		while (is_generating_pwm)
			;
		servo_update(20);
		while (!is_generating_pwm)
			;
	}

	return 0;
}