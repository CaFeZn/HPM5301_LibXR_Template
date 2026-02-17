/*
 * Copyright (c) 2024 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

#include "board.h"
#include "hpm_gpio.hpp"
#include "hpm_timebase.hpp"
#include "thread.hpp"

using namespace LibXR;

uint32_t i = 0;
uint32_t button_level = 0;

int main(void) {
	board_init();

	HPMTimebase timebase;
	(void)timebase;

	LibXR::Thread::Sleep(3000);

	HPMGPIO LED(BOARD_LED_GPIO_CTRL, BOARD_LED_GPIO_INDEX, BOARD_LED_GPIO_PIN);
	LED.SetConfig({GPIO::Direction::OUTPUT_PUSH_PULL, GPIO::Pull::NONE});

	HPMGPIO BUTTON(BOARD_APP_GPIO_CTRL, BOARD_APP_GPIO_INDEX, BOARD_APP_GPIO_PIN);
	BUTTON.SetConfig({GPIO::Direction::INPUT,GPIO::Pull::NONE});

	// HPMGPIO	   button(BOARD_APP_GPIO_CTRL, BOARD_APP_GPIO_INDEX, BOARD_APP_GPIO_PIN, BOARD_APP_GPIO_IRQ);
	// const auto trigger =
	// 	(BOARD_BUTTON_PRESSED_VALUE != 0) ? GPIO::Direction::RISING_INTERRUPT : GPIO::Direction::FALL_INTERRUPT;
	// button.SetConfig({trigger, GPIO::Pull::NONE});
	// button.RegisterCallback(
	// 	Callback<>::Create([](bool, HPMGPIO* led_gpio) { led_gpio->Write(!led_gpio->Read()); }, &led));
	// button.EnableInterrupt();
	// intc_m_enable_irq_with_priority(BOARD_APP_GPIO_IRQ, 5);



	while (1) {
		// Delay 1 second with LibXR thread sleep API.

		// LED.Write(true);
		LibXR::Thread::Sleep(1000);
		i++;
		LED.Write(!LED.Read());
		button_level = BUTTON.Read();
		// LED.Write(false);
		// LibXR::Thread::Sleep(1000);
	}

	return 0;
}
