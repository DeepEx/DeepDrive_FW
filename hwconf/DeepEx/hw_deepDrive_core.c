/*
	Copyright 2023 Benjamin Vedder	benjamin@vedder.se

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#include "hw.h"

#include "ch.h"
#include "hal.h"
#include "stm32f4xx_conf.h"
#include "utils_math.h"
#include "terminal.h"
#include "commands.h"
#include "mc_interface.h"
#include "mcpwm_foc.h"
#include "conf_general.h"
#include <stdio.h>
#include <math.h>

// Variables
static volatile bool i2c_running = false;

// Per-board calibration (see hw_deepDrive_core.h). Defaults = nominal scaling.
volatile float hw_dd_cal_i[3] = {1.0f, 1.0f, 1.0f};
volatile float hw_dd_cal_vin_gain = 1.0f;
volatile float hw_dd_cal_vin_ofs = 0.0f;
volatile float hw_dd_cal_vph = 1.0f;
static bool hw_dd_cal_loaded = false;

// EEPROM_VARS_HW layout
#define DD_CAL_EE_MAGIC			0
#define DD_CAL_EE_I0			1	// 1..3: per-phase current gains
#define DD_CAL_EE_VIN_GAIN		4
#define DD_CAL_EE_VIN_OFS		5
#define DD_CAL_EE_VPH			6
#define DD_CAL_MAGIC			0xDDCA1001u

#define DD_CAL_I_MIN			0.5f
#define DD_CAL_I_MAX			1.5f
#define DD_CAL_V_GAIN_MIN		0.8f
#define DD_CAL_V_GAIN_MAX		1.2f
#define DD_CAL_V_OFS_MAX		5.0f

// Private functions
static void terminal_dd_cal(int argc, const char **argv);
static void terminal_dd_cal_shunt(int argc, const char **argv);
static void terminal_dd_cal_gain(int argc, const char **argv);
static void terminal_dd_cal_vin(int argc, const char **argv);
static void terminal_dd_cal_vph(int argc, const char **argv);
static void terminal_dd_cal_reset(int argc, const char **argv);
static void terminal_dd_cal_balance(int argc, const char **argv);

// I2C configuration
static const I2CConfig i2cfg = {
		OPMODE_I2C,
		100000,
		STD_DUTY_CYCLE
};


void hw_init_gpio(void) {
	// GPIO clock enable
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	// LEDs
	palSetPadMode(LED_GREEN_GPIO, LED_GREEN_PIN,
			PAL_MODE_OUTPUT_PUSHPULL |
			PAL_STM32_OSPEED_HIGHEST);
	palSetPadMode(LED_RED_GPIO, LED_RED_PIN,
			PAL_MODE_OUTPUT_PUSHPULL |
			PAL_STM32_OSPEED_HIGHEST);

	// ENABLE_GATE
	// Set the output latch low before enabling the GPIO output to avoid a
	// spurious rising edge. The DRV8353RH clears latched faults with an
	// ENABLE-low pulse between 8 us and 40 us.
	DISABLE_GATE();
	palSetPadMode(GPIOB, 5,
			PAL_MODE_OUTPUT_PUSHPULL |
			PAL_STM32_OSPEED_HIGHEST);

	hw_reset_drv_faults();

	// Current filter
	palSetPadMode(GPIOD, 2,
			PAL_MODE_OUTPUT_PUSHPULL |
			PAL_STM32_OSPEED_HIGHEST);
	CURRENT_FILTER_OFF();

	// Phase filters
	palSetPadMode(PHASE_FILTER_GPIO, PHASE_FILTER_PIN,
			PAL_MODE_OUTPUT_PUSHPULL |
			PAL_STM32_OSPEED_HIGHEST);
	PHASE_FILTER_OFF();

	// GPIOA Configuration: Channel 1 to 3 as alternate function push-pull
	palSetPadMode(GPIOA, 8, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_FLOATING);
	palSetPadMode(GPIOA, 9, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_FLOATING);
	palSetPadMode(GPIOA, 10, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_FLOATING);

	palSetPadMode(GPIOB, 13, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_FLOATING);
	palSetPadMode(GPIOB, 14, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_FLOATING);
	palSetPadMode(GPIOB, 15, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_FLOATING);

	// Hall sensors
	palSetPadMode(HW_HALL_ENC_GPIO1, HW_HALL_ENC_PIN1, PAL_MODE_INPUT_PULLUP);
	palSetPadMode(HW_HALL_ENC_GPIO2, HW_HALL_ENC_PIN2, PAL_MODE_INPUT_PULLUP);
	palSetPadMode(HW_HALL_ENC_GPIO3, HW_HALL_ENC_PIN3, PAL_MODE_INPUT_PULLUP);

	// Fault pin
	palSetPadMode(GPIOB, 7, PAL_MODE_INPUT_PULLUP);

	// ADC Pins
	palSetPadMode(GPIOA, 0, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOA, 1, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOA, 2, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOA, 3, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOA, 5, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOA, 6, PAL_MODE_INPUT_ANALOG);

	palSetPadMode(GPIOC, 0, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOC, 1, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOC, 2, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOC, 3, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOC, 4, PAL_MODE_INPUT_ANALOG);

	// Calibration terminal commands (calibration itself is loaded by
	// HW_POST_CONF_INIT_HOOK, after the emulated EEPROM is initialised)
	terminal_register_command_callback(
			"dd_cal",
			"deepDrive: print current/voltage calibration",
			0, terminal_dd_cal);
	terminal_register_command_callback(
			"dd_cal_shunt",
			"deepDrive: set effective shunt resistance per phase [mOhm] (4-wire, at SP/SN), store",
			"[r1] [r2] [r3]", terminal_dd_cal_shunt);
	terminal_register_command_callback(
			"dd_cal_gain",
			"deepDrive: set per-phase current gain directly (nominal/effective), store",
			"[g1] [g2] [g3]", terminal_dd_cal_gain);
	terminal_register_command_callback(
			"dd_cal_vin",
			"deepDrive: set input voltage calibration V = gain * V_nom + offset, store",
			"[gain] [offset_V]", terminal_dd_cal_vin);
	terminal_register_command_callback(
			"dd_cal_vph",
			"deepDrive: set phase-voltage sense factor, store",
			"[factor]", terminal_dd_cal_vph);
	terminal_register_command_callback(
			"dd_cal_reset",
			"deepDrive: restore nominal calibration and erase stored values",
			0, terminal_dd_cal_reset);
	terminal_register_command_callback(
			"dd_cal_balance",
			"deepDrive: measure per-phase RMS current while running steadily and suggest gains",
			"[seconds]", terminal_dd_cal_balance);
}

void hw_reset_drv_faults(void) {
	DISABLE_GATE();

	// CH_CFG_ST_FREQUENCY is 10 kHz, so a ChibiOS sleep would be at least
	// 100 us and exceed the DRV8353RH reset-pulse maximum of 40 us.
	// The Cortex-M cycle counter gives a deterministic 20 us busy wait.
	const uint32_t start = DWT->CYCCNT;
	const uint32_t cycles = SystemCoreClock / 50000U;
	while ((uint32_t)(DWT->CYCCNT - start) < cycles) {
		__NOP();
	}

	ENABLE_GATE();
}

void hw_setup_adc_channels(void) {
	uint8_t t_samp = ADC_SampleTime_15Cycles;

	// ADC1 regular channels
	ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, t_samp);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 2, t_samp);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 3, t_samp);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_14, 4, t_samp);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_Vrefint, 5, t_samp);

	// ADC2 regular channels
	ADC_RegularChannelConfig(ADC2, ADC_Channel_11, 1, t_samp);
	ADC_RegularChannelConfig(ADC2, ADC_Channel_1, 2, t_samp);
	ADC_RegularChannelConfig(ADC2, ADC_Channel_6, 3, t_samp);
	ADC_RegularChannelConfig(ADC2, ADC_Channel_15, 4, t_samp);
	ADC_RegularChannelConfig(ADC2, ADC_Channel_0, 5, t_samp);

	// ADC3 regular channels
	ADC_RegularChannelConfig(ADC3, ADC_Channel_12, 1, t_samp);
	ADC_RegularChannelConfig(ADC3, ADC_Channel_2, 2, t_samp);
	ADC_RegularChannelConfig(ADC3, ADC_Channel_3, 3, t_samp);
	ADC_RegularChannelConfig(ADC3, ADC_Channel_13, 4, t_samp);
	ADC_RegularChannelConfig(ADC3, ADC_Channel_1, 5, t_samp);

	// Injected channels
	ADC_InjectedChannelConfig(ADC1, ADC_Channel_10, 1, t_samp);
	ADC_InjectedChannelConfig(ADC2, ADC_Channel_11, 1, t_samp);
	ADC_InjectedChannelConfig(ADC3, ADC_Channel_12, 1, t_samp);
	ADC_InjectedChannelConfig(ADC1, ADC_Channel_10, 2, t_samp);
	ADC_InjectedChannelConfig(ADC2, ADC_Channel_11, 2, t_samp);
	ADC_InjectedChannelConfig(ADC3, ADC_Channel_12, 2, t_samp);
	ADC_InjectedChannelConfig(ADC1, ADC_Channel_10, 3, t_samp);
	ADC_InjectedChannelConfig(ADC2, ADC_Channel_11, 3, t_samp);
	ADC_InjectedChannelConfig(ADC3, ADC_Channel_12, 3, t_samp);
}

void hw_start_i2c(void) {
	i2cAcquireBus(&HW_I2C_DEV);

	if (!i2c_running) {
		palSetPadMode(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN,
				PAL_MODE_ALTERNATE(HW_I2C_GPIO_AF) |
				PAL_STM32_OTYPE_OPENDRAIN |
				PAL_STM32_OSPEED_MID1 |
				PAL_STM32_PUDR_PULLUP);
		palSetPadMode(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN,
				PAL_MODE_ALTERNATE(HW_I2C_GPIO_AF) |
				PAL_STM32_OTYPE_OPENDRAIN |
				PAL_STM32_OSPEED_MID1 |
				PAL_STM32_PUDR_PULLUP);

		i2cStart(&HW_I2C_DEV, &i2cfg);
		i2c_running = true;
	}

	i2cReleaseBus(&HW_I2C_DEV);
}

void hw_stop_i2c(void) {
	i2cAcquireBus(&HW_I2C_DEV);

	if (i2c_running) {
		palSetPadMode(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN, PAL_MODE_INPUT);
		palSetPadMode(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN, PAL_MODE_INPUT);

		i2cStop(&HW_I2C_DEV);
		i2c_running = false;

	}

	i2cReleaseBus(&HW_I2C_DEV);
}

/**
 * Try to restore the i2c bus
 */
void hw_try_restore_i2c(void) {
	if (i2c_running) {
		i2cAcquireBus(&HW_I2C_DEV);

		palSetPadMode(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN,
				PAL_STM32_OTYPE_OPENDRAIN |
				PAL_STM32_OSPEED_MID1 |
				PAL_STM32_PUDR_PULLUP);

		palSetPadMode(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN,
				PAL_STM32_OTYPE_OPENDRAIN |
				PAL_STM32_OSPEED_MID1 |
				PAL_STM32_PUDR_PULLUP);

		palSetPad(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN);
		palSetPad(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN);

		chThdSleep(1);

		for(int i = 0;i < 16;i++) {
			palClearPad(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN);
			chThdSleep(1);
			palSetPad(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN);
			chThdSleep(1);
		}

		// Generate start then stop condition
		palClearPad(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN);
		chThdSleep(1);
		palClearPad(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN);
		chThdSleep(1);
		palSetPad(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN);
		chThdSleep(1);
		palSetPad(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN);

		palSetPadMode(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN,
				PAL_MODE_ALTERNATE(HW_I2C_GPIO_AF) |
				PAL_STM32_OTYPE_OPENDRAIN |
				PAL_STM32_OSPEED_MID1 |
				PAL_STM32_PUDR_PULLUP);

		palSetPadMode(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN,
				PAL_MODE_ALTERNATE(HW_I2C_GPIO_AF) |
				PAL_STM32_OTYPE_OPENDRAIN |
				PAL_STM32_OSPEED_MID1 |
				PAL_STM32_PUDR_PULLUP);

		HW_I2C_DEV.state = I2C_STOP;
		i2cStart(&HW_I2C_DEV, &i2cfg);

		i2cReleaseBus(&HW_I2C_DEV);
	}
}

// ---------------------------------------------------------------------------
// Per-board current / voltage calibration
// ---------------------------------------------------------------------------

static bool cal_i_valid(float g) {
	return isfinite(g) && g >= DD_CAL_I_MIN && g <= DD_CAL_I_MAX;
}

static bool cal_vgain_valid(float g) {
	return isfinite(g) && g >= DD_CAL_V_GAIN_MIN && g <= DD_CAL_V_GAIN_MAX;
}

void hw_deepdrive_load_calibration(void) {
	eeprom_var v;

	if (!conf_general_read_eeprom_var_hw(&v, DD_CAL_EE_MAGIC) || v.as_u32 != DD_CAL_MAGIC) {
		hw_dd_cal_loaded = false;
		return; // nothing stored: keep nominal scaling
	}

	float i_cal[3];
	for (int i = 0;i < 3;i++) {
		if (!conf_general_read_eeprom_var_hw(&v, DD_CAL_EE_I0 + i) || !cal_i_valid(v.as_float)) {
			hw_dd_cal_loaded = false;
			return; // corrupt or out of range: keep nominal scaling
		}
		i_cal[i] = v.as_float;
	}

	float vin_gain = 1.0f, vin_ofs = 0.0f, vph = 1.0f;
	if (conf_general_read_eeprom_var_hw(&v, DD_CAL_EE_VIN_GAIN) && cal_vgain_valid(v.as_float)) {
		vin_gain = v.as_float;
	}
	if (conf_general_read_eeprom_var_hw(&v, DD_CAL_EE_VIN_OFS) &&
			isfinite(v.as_float) && fabsf(v.as_float) <= DD_CAL_V_OFS_MAX) {
		vin_ofs = v.as_float;
	}
	if (conf_general_read_eeprom_var_hw(&v, DD_CAL_EE_VPH) && cal_vgain_valid(v.as_float)) {
		vph = v.as_float;
	}

	for (int i = 0;i < 3;i++) {
		hw_dd_cal_i[i] = i_cal[i];
	}
	hw_dd_cal_vin_gain = vin_gain;
	hw_dd_cal_vin_ofs = vin_ofs;
	hw_dd_cal_vph = vph;
	hw_dd_cal_loaded = true;
}

static bool store_calibration(void) {
	eeprom_var v;
	bool ok = true;

	for (int i = 0;i < 3;i++) {
		v.as_float = hw_dd_cal_i[i];
		ok &= conf_general_store_eeprom_var_hw(&v, DD_CAL_EE_I0 + i);
	}
	v.as_float = hw_dd_cal_vin_gain;
	ok &= conf_general_store_eeprom_var_hw(&v, DD_CAL_EE_VIN_GAIN);
	v.as_float = hw_dd_cal_vin_ofs;
	ok &= conf_general_store_eeprom_var_hw(&v, DD_CAL_EE_VIN_OFS);
	v.as_float = hw_dd_cal_vph;
	ok &= conf_general_store_eeprom_var_hw(&v, DD_CAL_EE_VPH);

	// Magic written last, so an interrupted store is never loaded
	v.as_u32 = DD_CAL_MAGIC;
	ok &= conf_general_store_eeprom_var_hw(&v, DD_CAL_EE_MAGIC);

	hw_dd_cal_loaded = ok;
	return ok;
}

static bool motor_is_stopped(void) {
	if (mc_interface_get_state() != MC_STATE_OFF) {
		commands_printf("Refused: stop the motor before changing the calibration.\n");
		return false;
	}
	return true;
}

static void print_after_change(bool stored) {
	commands_printf(stored ? "Stored in EEPROM." : "WARNING: EEPROM store failed, values active until reboot only.");
	commands_printf("Repeat FOC offset calibration and motor detection (R, L, lambda) before use.\n");
}

static void terminal_dd_cal(int argc, const char **argv) {
	(void)argc; (void)argv;

	commands_printf("deepDrive calibration (%s)", hw_dd_cal_loaded ? "loaded from EEPROM" : "nominal, nothing stored");
	commands_printf("  Nominal shunt: %.4f mOhm, amplifier gain: %.1f V/V", (double)(CURRENT_SHUNT_RES * 1e3), (double)CURRENT_AMP_GAIN);
	for (int i = 0;i < 3;i++) {
		commands_printf("  Phase %d: gain %.4f  (effective shunt %.4f mOhm)", i + 1,
				(double)hw_dd_cal_i[i], (double)(CURRENT_SHUNT_RES * 1e3 / hw_dd_cal_i[i]));
	}
	commands_printf("  Vin: gain %.4f, offset %.3f V  (now %.2f V, nominal %.2f V)",
			(double)hw_dd_cal_vin_gain, (double)hw_dd_cal_vin_ofs,
			(double)GET_INPUT_VOLTAGE(), (double)GET_INPUT_VOLTAGE_RAW());
	commands_printf("  Phase-voltage factor: %.4f\n", (double)hw_dd_cal_vph);
}

static void terminal_dd_cal_shunt(int argc, const char **argv) {
	if (argc != 4) {
		commands_printf("Usage: dd_cal_shunt [r1] [r2] [r3]   (mOhm, measured at the SP/SN pins)\n");
		return;
	}
	if (!motor_is_stopped()) {
		return;
	}

	float g[3];
	for (int i = 0;i < 3;i++) {
		float r = -1.0f;
		sscanf(argv[i + 1], "%f", &r);
		g[i] = (r > 0.0f) ? (float)(CURRENT_SHUNT_RES * 1e3) / r : -1.0f;
		if (!cal_i_valid(g[i])) {
			commands_printf("Phase %d: %s mOhm gives gain out of range [%.2f, %.2f]. Nothing changed.\n",
					i + 1, argv[i + 1], (double)DD_CAL_I_MIN, (double)DD_CAL_I_MAX);
			return;
		}
	}
	for (int i = 0;i < 3;i++) {
		hw_dd_cal_i[i] = g[i];
	}
	print_after_change(store_calibration());
	terminal_dd_cal(0, 0);
}

static void terminal_dd_cal_gain(int argc, const char **argv) {
	if (argc != 4) {
		commands_printf("Usage: dd_cal_gain [g1] [g2] [g3]   (nominal shunt / effective shunt)\n");
		return;
	}
	if (!motor_is_stopped()) {
		return;
	}

	float g[3];
	for (int i = 0;i < 3;i++) {
		g[i] = -1.0f;
		sscanf(argv[i + 1], "%f", &g[i]);
		if (!cal_i_valid(g[i])) {
			commands_printf("Phase %d: gain %s out of range [%.2f, %.2f]. Nothing changed.\n",
					i + 1, argv[i + 1], (double)DD_CAL_I_MIN, (double)DD_CAL_I_MAX);
			return;
		}
	}
	for (int i = 0;i < 3;i++) {
		hw_dd_cal_i[i] = g[i];
	}
	print_after_change(store_calibration());
	terminal_dd_cal(0, 0);
}

static void terminal_dd_cal_vin(int argc, const char **argv) {
	if (argc != 3) {
		commands_printf("Usage: dd_cal_vin [gain] [offset_V]   (V_true = gain * V_nominal + offset)\n");
		return;
	}
	if (!motor_is_stopped()) {
		return;
	}

	float gain = -1.0f, ofs = 1e9f;
	sscanf(argv[1], "%f", &gain);
	sscanf(argv[2], "%f", &ofs);
	if (!cal_vgain_valid(gain) || !isfinite(ofs) || fabsf(ofs) > DD_CAL_V_OFS_MAX) {
		commands_printf("Out of range: gain [%.2f, %.2f], |offset| <= %.1f V. Nothing changed.\n",
				(double)DD_CAL_V_GAIN_MIN, (double)DD_CAL_V_GAIN_MAX, (double)DD_CAL_V_OFS_MAX);
		return;
	}
	hw_dd_cal_vin_gain = gain;
	hw_dd_cal_vin_ofs = ofs;
	print_after_change(store_calibration());
	terminal_dd_cal(0, 0);
}

static void terminal_dd_cal_vph(int argc, const char **argv) {
	if (argc != 2) {
		commands_printf("Usage: dd_cal_vph [factor]\n");
		return;
	}
	if (!motor_is_stopped()) {
		return;
	}

	float f = -1.0f;
	sscanf(argv[1], "%f", &f);
	if (!cal_vgain_valid(f)) {
		commands_printf("Factor out of range [%.2f, %.2f]. Nothing changed.\n",
				(double)DD_CAL_V_GAIN_MIN, (double)DD_CAL_V_GAIN_MAX);
		return;
	}
	hw_dd_cal_vph = f;
	print_after_change(store_calibration());
	terminal_dd_cal(0, 0);
}

static void terminal_dd_cal_reset(int argc, const char **argv) {
	(void)argc; (void)argv;
	if (!motor_is_stopped()) {
		return;
	}

	eeprom_var v;
	v.as_u32 = 0;
	bool ok = conf_general_store_eeprom_var_hw(&v, DD_CAL_EE_MAGIC);

	for (int i = 0;i < 3;i++) {
		hw_dd_cal_i[i] = 1.0f;
	}
	hw_dd_cal_vin_gain = 1.0f;
	hw_dd_cal_vin_ofs = 0.0f;
	hw_dd_cal_vph = 1.0f;
	hw_dd_cal_loaded = false;

	commands_printf(ok ? "Nominal calibration restored, stored values invalidated." :
			"WARNING: could not invalidate stored values; nominal values active until reboot.");
	commands_printf("Repeat FOC offset calibration and motor detection before use.\n");
}

/*
 * Relative gain between the three shunt channels. With a healthy motor the
 * three phase currents have the same amplitude, so differences in the measured
 * RMS come from the measurement chain. Run the motor at a steady, moderate
 * current and low duty (< 0.5, so every low-side shunt sample is valid).
 * The suggested gains keep the current average gain (absolute scale is set
 * by dd_cal_shunt) and only remove the phase-to-phase mismatch.
 */
static void terminal_dd_cal_balance(int argc, const char **argv) {
	float seconds = 5.0f;
	if (argc == 2) {
		sscanf(argv[1], "%f", &seconds);
	}
	if (!isfinite(seconds) || seconds < 1.0f || seconds > 20.0f) {
		commands_printf("Duration must be 1-20 s.\n");
		return;
	}
	if (mc_interface_get_state() != MC_STATE_RUNNING || fabsf(mc_interface_get_tot_current_filtered()) < 3.0f) {
		commands_printf("Run the motor steadily (|current| > 3 A, duty < 0.5) before dd_cal_balance.\n");
		return;
	}

	const volatile mc_configuration *conf = mc_interface_get_configuration();
	double sumsq[3] = {0.0, 0.0, 0.0};
	int samples = 0;
	const int n = (int)(seconds * 1000.0f);

	for (int k = 0;k < n;k++) {
		float raw[3];
		mcpwm_foc_get_currents_adc(&raw[0], &raw[1], &raw[2], false);
		for (int i = 0;i < 3;i++) {
			const float amp = (raw[i] - conf->foc_offsets_current[i]) * FAC_CURRENT * hw_dd_cal_i[i];
			sumsq[i] += (double)(amp * amp);
		}
		samples++;
		chThdSleepMilliseconds(1);

		if (mc_interface_get_state() != MC_STATE_RUNNING) {
			commands_printf("Motor stopped during the measurement, aborted.\n");
			return;
		}
	}

	float rms[3];
	float rms_mean = 0.0f, g_mean = 0.0f;
	for (int i = 0;i < 3;i++) {
		rms[i] = sqrtf((float)(sumsq[i] / (double)samples));
		rms_mean += rms[i] / 3.0f;
		g_mean += hw_dd_cal_i[i] / 3.0f;
	}
	if (rms_mean < 1.0f) {
		commands_printf("RMS too low (%.2f A), increase the load.\n", (double)rms_mean);
		return;
	}

	float g_new[3];
	float g_new_mean = 0.0f;
	for (int i = 0;i < 3;i++) {
		g_new[i] = hw_dd_cal_i[i] * rms_mean / rms[i];
		g_new_mean += g_new[i] / 3.0f;
	}
	// Keep the average gain unchanged
	for (int i = 0;i < 3;i++) {
		g_new[i] *= g_mean / g_new_mean;
	}

	commands_printf("Balance over %d samples:", samples);
	for (int i = 0;i < 3;i++) {
		commands_printf("  Phase %d: RMS %.3f A (%+.1f %% vs mean), gain %.4f -> suggested %.4f",
				i + 1, (double)rms[i], (double)((rms[i] / rms_mean - 1.0f) * 100.0f),
				(double)hw_dd_cal_i[i], (double)g_new[i]);
	}
	commands_printf("Nothing stored. To apply, stop the motor and run:");
	commands_printf("  dd_cal_gain %.4f %.4f %.4f\n", (double)g_new[0], (double)g_new[1], (double)g_new[2]);
}
