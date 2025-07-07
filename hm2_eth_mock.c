#include "rtapi.h"
#include "rtapi_app.h"
#include "hal.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "hal_helpers.h"

MODULE_AUTHOR("Peter Ludwig");
MODULE_DESCRIPTION("Mock for Mesa HM2_ETH I/O card driver, enabling testing and simulation without requiring real mesa card hardware.");
MODULE_LICENSE("GPL");

static int comp_id;

static char *version = "0.8";

// simulating configuration of mesa card
static char *config = "";
RTAPI_MP_STRING(config, "Configuration string");
static char *board = "";
static char *daughter_card = "";
RTAPI_MP_STRING(board, "Board Type string");

static int num_stepgens = 5;
static int num_encoders = 1;
static int num_pwmgens = 0;

// Parameters (allocated with hal_malloc)
static hal_u32_t *watchdog_timeout_ns;
static hal_bit_t **watchdog_has_bit;
static hal_s32_t *dpll_01_timer_us;
static hal_u32_t *stepgen_timer_number;

static hal_float_t **field_voltage, **field_voltage_sim;

static char sserial_ports[64] = "";	 // optional capture
static int sserial_first_digit = -1; // Default to invalid

//

typedef struct
{
	hal_bit_t *in;
	hal_bit_t *in_sim;
	hal_bit_t *in_not;
	hal_bit_t *out;
} gpio_t;

// Daughter Card Pins

typedef struct
{
	hal_float_t *in;
	hal_float_t *in_sim;

} analog_in_t;

typedef struct
{
	hal_bit_t *in;
	hal_bit_t *in_sim;
	hal_bit_t *in_not;
} digital_in_t;

typedef struct
{
	hal_bit_t *out;
} digital_out_t;

typedef struct
{
	hal_float_t *pos;
	hal_s32_t *counts;
} enc_t;

typedef struct
{
	hal_bit_t *control_type;
	hal_float_t *pos_cmd;
	hal_float_t *pos_fb;
	hal_bit_t *step;
	hal_bit_t *dir;
	hal_bit_t *enable;
	hal_s32_t *counts;
	hal_float_t *velocity_cmd;
	hal_float_t *velocity_fb;

	hal_u32_t dirSetup;
	hal_u32_t dirHold;
	hal_u32_t stepLen;
	hal_u32_t stepSpace;

	hal_float_t positionScale;
	hal_u32_t stepType;
	hal_float_t maxAcceleration;
	hal_float_t maxVelocity;

} stepgen_t;

typedef struct
{
	hal_bit_t *spindir;
	hal_bit_t spindir_invert;
	hal_bit_t *spinena;
	hal_bit_t spinena_invert;
	hal_float_t *spinout;
	hal_float_t spinout_minlim;
	hal_float_t spinout_maxlim;
	hal_float_t spinout_scalemax;
} spindle_t;

// PWM

typedef struct
{
	hal_float_t *pwm_val;
	hal_float_t *pwm_fb;
	hal_bit_t *pwm_enable;
	hal_float_t pwm_scale;
} pwm_t;
static hal_u32_t *pwm_frequency;

typedef struct
{
	int num_analog_in;
	int num_digital_in;
	int num_digital_out;
	int num_encoders;
	int num_gpios;
	int num_gpios_out; // not used
	int num_stepgens;
	int num_spindle;
	int num_pwm;

} card_config_t;

typedef struct
{
	char identifier[64];
	char *board_type;
	card_config_t config;
	analog_in_t *analog_inputs;
	digital_in_t *digital_inputs;
	digital_out_t *digital_outputs;
	enc_t *enc;
	gpio_t *gpio;

	spindle_t *spindle;
	stepgen_t *step_gen;
	pwm_t *pwm;
} card_t;

static card_t *cards = NULL;
static int num_cards = 0;

static double threat_cycle_time; // cycle time [s] of threat to calculate position made between two calls
static long period_ns = 0;

// Simple helper to extract values like "num_stepgens=3"
static void parse_config_string(const char *config_str)
{
	char *config_copy = strdup(config_str); // make a modifiable copy
	char *token = strtok(config_copy, " ");
	while (token)
	{
		if (strncmp(token, "num_stepgens=", 13) == 0)
		{
			num_stepgens = atoi(token + 13);
		}
		else if (strncmp(token, "num_encoders=", 13) == 0)
		{
			num_encoders = atoi(token + 13);
		}
		else if (strncmp(token, "num_pwmgens=", 12) == 0)
		{
			num_pwmgens = atoi(token + 12);
		}

		else if (strncmp(token, "sserial_port_0=", 15) == 0)
		{
			strncpy(sserial_ports, token + 15, sizeof(sserial_ports) - 1);
			sserial_ports[sizeof(sserial_ports) - 1] = '\0';
			// Extract first digit
			if (isdigit(sserial_ports[0]))
			{
				sserial_first_digit = sserial_ports[0] - '0';
			}
		}

		token = strtok(NULL, " ");
	}

	free(config_copy);
}

// as the physical card has read and write function both are adapted
// but for simulation only one of them does the simulation job
static void write(void *arg, long period_nsec) {}

static void read(void *arg, long period_nsec)
{
	if (period_nsec != period_ns)
	{
		// threat_cycle_time=(double) period_nsec* 1e-10*5.; due to write function was mapped to read during export
		threat_cycle_time = (double)period_nsec * 1e-9;
		period_ns = period_nsec;
	}

	for (int card_index = 0; card_index < num_cards; card_index++)
	{
		// Step Generator
		for (int i = 0; i < cards[card_index].config.num_stepgens; i++)
		{
			stepgen_t *sg = &cards[card_index].step_gen[i];
			*(sg->dir) = (*(sg->pos_cmd) >= 0);
			*(sg->velocity_fb) = *(sg->velocity_cmd);

			if (*(sg->control_type) == 0)
			{
				*(sg->pos_fb) = *(sg->pos_cmd);
				*(sg->step) = ((int)(*(sg->pos_cmd) * 10) % 2);
			}
			else if (*(sg->control_type) == 1)
			{
				*(sg->pos_fb) += (float)((double)(*(sg->velocity_fb)) * threat_cycle_time);
				*(sg->counts) = (hal_s32_t)(*(sg->pos_fb) * sg->positionScale);
			}
		}

		/* TODO: how to simulate

				for (int i = 0; i < cards[card_index].config.num_encoders; i++)
				{
					enc_t *sg = &cards[card_index].enc[i];
					*(sg->enc_pos)+= 0.01;
					*(sg->enc_counts)=(int)(*(sg->enc_pos)* 1000);

				}
		*/

		// GPIO, prepared, but from my perspective not used at all, also not clear how to mock
		/*
		for (int i = 0; i < cards[card_index].config.num_gpios_out; i++)
		{
			gpio_out_t *sgo = &cards[card_index].gpio_out[i];
			gpio_in_t *sgi = &cards[card_index].gpio_in[i];
			*(sgo->out) = *(sgi->in);
		}
		*/

		// Digital Input
		for (int i = 0; i < cards[card_index].config.num_digital_in; i++)
		{
			digital_in_t *sg = &cards[card_index].digital_inputs[i];
			*(sg->in) = *(sg->in_sim);
			*(sg->in_not) = !(*(sg->in));
		}

		// Analog Input
		for (int i = 0; i < cards[card_index].config.num_analog_in; i++)
		{
			analog_in_t *sg = &cards[card_index].analog_inputs[i];
			*(sg->in) = *(sg->in_sim);
		}
		// PWM
		for (int i = 0; i < cards[card_index].config.num_pwm; i++)
		{
			pwm_t *sg = &cards[card_index].pwm[i];
			*(sg->pwm_fb) = *(sg->pwm_val);
			*(sg->pwm_enable) = (*(sg->pwm_val) > 0.1);
		}

		// card specific updates
		if (strcmp(cards[card_index].board_type, "7i76") == 0)
		{
			**field_voltage = **field_voltage_sim;
		}
	}
}

int configure_card(const int index)
{
	char name[64]; // needed for hal_helpers
	cards[index].analog_inputs = hal_malloc(cards[index].config.num_analog_in * sizeof(analog_in_t));
	memset(cards[index].analog_inputs, 0, cards[index].config.num_analog_in * sizeof(analog_in_t));

	cards[index].digital_inputs = hal_malloc(cards[index].config.num_digital_in * sizeof(digital_in_t));
	memset(cards[index].digital_inputs, 0, cards[index].config.num_digital_in * sizeof(digital_in_t));

	cards[index].digital_outputs = hal_malloc(cards[index].config.num_digital_out * sizeof(digital_out_t));
	memset(cards[index].digital_outputs, 0, cards[index].config.num_digital_out * sizeof(digital_out_t));

	cards[index].enc = hal_malloc(cards[index].config.num_encoders * sizeof(enc_t));
	memset(cards[index].enc, 0, cards[index].config.num_encoders * sizeof(enc_t));

	cards[index].gpio = hal_malloc(cards[index].config.num_gpios * sizeof(gpio_t));
	memset(cards[index].gpio, 0, cards[index].config.num_gpios * sizeof(gpio_t));

	cards[index].spindle = hal_malloc(cards[index].config.num_spindle * sizeof(spindle_t));
	memset(cards[index].spindle, 0, cards[index].config.num_spindle * sizeof(spindle_t));

	cards[index].step_gen = hal_malloc(cards[index].config.num_stepgens * sizeof(stepgen_t));
	memset(cards[index].step_gen, 0, cards[index].config.num_stepgens * sizeof(stepgen_t));

	cards[index].pwm = hal_malloc(cards[index].config.num_pwm * sizeof(pwm_t));
	memset(cards[index].pwm, 0, cards[index].config.num_pwm * sizeof(pwm_t));

	// Analog Input

	HAL_PIN_FLOAT_STRUCT_ARRAY(cards[index].analog_inputs, in, cards[index].config.num_analog_in, cards[index].identifier, ".analogin%01d", HAL_OUT, comp_id);
	HAL_PIN_FLOAT_STRUCT_ARRAY(cards[index].analog_inputs, in_sim, cards[index].config.num_analog_in, cards[index].identifier, ".analogin%01d-sim", HAL_IN, comp_id);

	// Encoders
	if (strcmp(cards[index].board_type, "7i76") == 0)
	{
		HAL_PIN_FLOAT_STRUCT_ARRAY(cards[index].enc, pos, cards[index].config.num_encoders, cards[index].identifier, ".enc%01d.position", HAL_OUT, comp_id);
		HAL_PIN_S32_STRUCT_ARRAY(cards[index].enc, counts, cards[index].config.num_encoders, cards[index].identifier, ".enc%01d.count", HAL_OUT, comp_id);
	}
	if (strcmp(cards[index].board_type, "7i76e") == 0)
	{
		HAL_PIN_FLOAT_STRUCT_ARRAY(cards[index].enc, pos, cards[index].config.num_encoders, cards[index].identifier, ".encoder.%02d.position", HAL_OUT, comp_id);
		HAL_PIN_S32_STRUCT_ARRAY(cards[index].enc, counts, cards[index].config.num_encoders, cards[index].identifier, ".encoder.%02d.count", HAL_OUT, comp_id);
	}

	// GPIO
	// HAL_PIN_BIT_STRUCT_ARRAY(cards[index].gpios, in, cards[index].config.num_gpios, cards[index].identifier, ".gpio.%03d.in", HAL_OUT, comp_id);
	// HAL_PIN_BIT_STRUCT_ARRAY(cards[index].gpios, in_sim, cards[index].config.num_gpios, cards[index].identifier, ".gpio.%03d.in-sim", HAL_IN, comp_id);
	// HAL_PIN_BIT_STRUCT_ARRAY(cards[index].gpios, in_not, cards[index].config.num_gpios, cards[index].identifier, "gpio.%03d.in_not", HAL_OUT, comp_id);

	// only prepared but numbering on 7i76e starts at 017
	// HAL_PIN_BIT_STRUCT_ARRAY(cards[index].gpios, out, cards[index].config.num_gpio_out, cards[index].identifier, "gpio.%03d.out", HAL_OUT, comp_id);

	// Digital IO

	HAL_PIN_BIT_STRUCT_ARRAY(cards[index].digital_inputs, in, cards[index].config.num_digital_in, cards[index].identifier, ".input-%02d", HAL_OUT, comp_id);
	HAL_PIN_BIT_STRUCT_ARRAY(cards[index].digital_inputs, in_sim, cards[index].config.num_digital_in, cards[index].identifier, ".input-%02d-sim", HAL_IN, comp_id);
	HAL_PIN_BIT_STRUCT_ARRAY(cards[index].digital_inputs, in_not, cards[index].config.num_digital_in, cards[index].identifier, ".input-%02d-not", HAL_OUT, comp_id);

	HAL_PIN_BIT_STRUCT_ARRAY(cards[index].digital_outputs, out, cards[index].config.num_digital_out, cards[index].identifier, ".output-%02d", HAL_IN, comp_id);

	// Spindle

	HAL_PIN_BIT_STRUCT_ARRAY(cards[index].spindle, spindir, cards[index].config.num_spindle, cards[index].identifier, ".spindir", HAL_IN, comp_id);
	HAL_PARAM_BIT_STRUCT_ARRAY(cards[index].spindle, spindir_invert, cards[index].config.num_spindle, cards[index].identifier, ".spindir-invert", HAL_RW, comp_id);

	HAL_PIN_BIT_STRUCT_ARRAY(cards[index].spindle, spinena, cards[index].config.num_spindle, cards[index].identifier, ".spinena", HAL_IN, comp_id);
	HAL_PARAM_BIT_STRUCT_ARRAY(cards[index].spindle, spinena_invert, cards[index].config.num_spindle, cards[index].identifier, ".spinena-invert", HAL_RW, comp_id);

	HAL_PIN_FLOAT_STRUCT_ARRAY(cards[index].spindle, spinout, cards[index].config.num_spindle, cards[index].identifier, ".spinout", HAL_IN, comp_id);

	HAL_PARAM_FLOAT_STRUCT_ARRAY(cards[index].spindle, spinout_minlim, cards[index].config.num_spindle, cards[index].identifier, ".spinout-minlim", HAL_RW, comp_id);
	HAL_PARAM_FLOAT_STRUCT_ARRAY(cards[index].spindle, spinout_maxlim, cards[index].config.num_spindle, cards[index].identifier, ".spinout-maxlim", HAL_RW, comp_id);
	HAL_PARAM_FLOAT_STRUCT_ARRAY(cards[index].spindle, spinout_scalemax, cards[index].config.num_spindle, cards[index].identifier, ".spinout-scalemax", HAL_RW, comp_id);

	// Stepgen
	HAL_PIN_BIT_STRUCT_ARRAY(cards[index].step_gen, control_type, cards[index].config.num_stepgens, cards[index].identifier, ".stepgen.%02d.control-type", HAL_IN, comp_id);
	HAL_PIN_FLOAT_STRUCT_ARRAY(cards[index].step_gen, pos_cmd, cards[index].config.num_stepgens, cards[index].identifier, ".stepgen.%02d.position-cmd", HAL_IN, comp_id);
	HAL_PIN_FLOAT_STRUCT_ARRAY(cards[index].step_gen, pos_fb, cards[index].config.num_stepgens, cards[index].identifier, ".stepgen.%02d.position-fb", HAL_OUT, comp_id);
	HAL_PIN_BIT_STRUCT_ARRAY(cards[index].step_gen, step, cards[index].config.num_stepgens, cards[index].identifier, ".stepgen.%02d.step", HAL_OUT, comp_id);
	HAL_PIN_BIT_STRUCT_ARRAY(cards[index].step_gen, dir, cards[index].config.num_stepgens, cards[index].identifier, ".stepgen.%02d.dir", HAL_OUT, comp_id);
	HAL_PIN_BIT_STRUCT_ARRAY(cards[index].step_gen, enable, cards[index].config.num_stepgens, cards[index].identifier, ".stepgen.%02d.enable", HAL_IN, comp_id);
	HAL_PIN_S32_STRUCT_ARRAY(cards[index].step_gen, counts, cards[index].config.num_stepgens, cards[index].identifier, ".stepgen.%02d.counts", HAL_OUT, comp_id);

	HAL_PIN_FLOAT_STRUCT_ARRAY(cards[index].step_gen, velocity_cmd, cards[index].config.num_stepgens, cards[index].identifier, ".stepgen.%02d.velocity-cmd", HAL_IN, comp_id);
	HAL_PIN_FLOAT_STRUCT_ARRAY(cards[index].step_gen, velocity_fb, cards[index].config.num_stepgens, cards[index].identifier, ".stepgen.%02d.velocity-fb", HAL_OUT, comp_id);

	HAL_PARAM_U32_STRUCT_ARRAY(cards[index].step_gen, dirSetup, cards[index].config.num_stepgens, cards[index].identifier, ".stepgen.%02d.dirsetup", HAL_RW, comp_id);
	HAL_PARAM_U32_STRUCT_ARRAY(cards[index].step_gen, dirHold, cards[index].config.num_stepgens, cards[index].identifier, ".stepgen.%02d.dirhold", HAL_RW, comp_id);
	HAL_PARAM_U32_STRUCT_ARRAY(cards[index].step_gen, stepLen, cards[index].config.num_stepgens, cards[index].identifier, ".stepgen.%02d.steplen", HAL_RW, comp_id);
	HAL_PARAM_U32_STRUCT_ARRAY(cards[index].step_gen, stepSpace, cards[index].config.num_stepgens, cards[index].identifier, ".stepgen.%02d.stepspace", HAL_RW, comp_id);

	HAL_PARAM_FLOAT_STRUCT_ARRAY(cards[index].step_gen, positionScale, cards[index].config.num_stepgens, cards[index].identifier, ".stepgen.%02d.position-scale", HAL_RW, comp_id);
	HAL_PARAM_U32_STRUCT_ARRAY(cards[index].step_gen, stepType, cards[index].config.num_stepgens, cards[index].identifier, ".stepgen.%02d.step_type", HAL_RW, comp_id);
	HAL_PARAM_FLOAT_STRUCT_ARRAY(cards[index].step_gen, maxAcceleration, cards[index].config.num_stepgens, cards[index].identifier, ".stepgen.%02d.maxaccel", HAL_RW, comp_id);
	HAL_PARAM_FLOAT_STRUCT_ARRAY(cards[index].step_gen, maxVelocity, cards[index].config.num_stepgens, cards[index].identifier, ".stepgen.%02d.maxvel", HAL_RW, comp_id);

	// PWM
	HAL_PIN_FLOAT_STRUCT_ARRAY(cards[index].pwm, pwm_val, cards[index].config.num_pwm, cards[index].identifier, ".pwmgen.%02d.value", HAL_IN, comp_id);
	HAL_PIN_FLOAT_STRUCT_ARRAY(cards[index].pwm, pwm_fb, cards[index].config.num_pwm, cards[index].identifier, ".pwmgen.%02d.feedback", HAL_OUT, comp_id);
	HAL_PIN_BIT_STRUCT_ARRAY(cards[index].pwm, pwm_enable, cards[index].config.num_pwm, cards[index].identifier, ".pwmgen.%02d.enable", HAL_IN, comp_id);
	HAL_PARAM_FLOAT_STRUCT_ARRAY(cards[index].pwm, pwm_scale, cards[index].config.num_pwm, cards[index].identifier, ".pwmgen.%02d.scale", HAL_RW, comp_id);
	if (cards[index].config.num_pwm > 0)
	{
		HAL_PARAM_U32(pwm_frequency, cards[index].identifier, ".pwmgen.pwm_frequency", HAL_RW, comp_id);
	}

	// board type individual pins and parameters
	if (strcmp(cards[index].board_type, "7i76e") == 0)
	{
		// Parameters (hal_malloc + hal_param_*_new)
		watchdog_timeout_ns = hal_malloc(sizeof(hal_u32_t));
		dpll_01_timer_us = hal_malloc(sizeof(hal_s32_t));
		stepgen_timer_number = hal_malloc(sizeof(hal_u32_t));
		watchdog_has_bit = hal_malloc(sizeof(hal_bit_t *));

		if (!watchdog_timeout_ns || !dpll_01_timer_us || !stepgen_timer_number)
			return -ENOMEM;

		HAL_PARAM_U32(watchdog_timeout_ns, cards[index].identifier, ".watchdog.timeout_ns", HAL_RW, comp_id);
		HAL_PIN_BIT(watchdog_has_bit, cards[index].identifier, ".watchdog.has_bit", HAL_OUT, comp_id);
		HAL_PARAM_S32(dpll_01_timer_us, cards[index].identifier, ".dpll.01.timer-us", HAL_RW, comp_id);
		HAL_PARAM_U32(stepgen_timer_number, cards[index].identifier, ".stepgen.timer-number", HAL_RW, comp_id);
	}

	if (strcmp(cards[index].board_type, "7i76") == 0)
	{
		HAL_PIN_FLOAT(field_voltage, cards[index].identifier, ".fieldvoltage", HAL_OUT, comp_id);
		HAL_PIN_FLOAT(field_voltage_sim, cards[index].identifier, ".fieldvoltage-sim", HAL_IN, comp_id);
	}
}

int rtapi_app_main(void)
{
	int p_return;
	char name[64]; // needed for hal_helpers

	int counter = 0;
	to_lowercase(board);
	rtapi_print("Board string: %s\n", board);
	rtapi_print("hm2_eth_mock Version: %s\n", version);
	// TODO: The supported boards are: 7I76E, 7I80DB, 7I80HD, 7I92, 7I93, 7I94, 7I95, 7I96, 7I96S, 7I97, 7I98
	// currently only supported: 7I76E

	int daughter_card_connector = 0;
	int daughter_card_instance = 0;

	parse_config_string(config);

	comp_id = hal_init("hm2_eth_mock");
	if (comp_id < 0)
		return comp_id;

	num_cards = 2; // or from config/environment
	cards = malloc(sizeof(card_t) * num_cards);
	if (!cards)
	{
		fprintf(stderr, "Failed to allocate memory for cards\n");
		exit(1);
	}

	snprintf(cards[0].identifier, sizeof(cards[0].identifier), "hm2_%s.%01d", board, counter);
	cards[0].board_type = board;
	cards[0].config.num_gpios = 16;
	cards[0].config.num_gpios_out = 16;
	cards[0].config.num_stepgens = num_stepgens;
	cards[0].config.num_spindle = 0;
	cards[0].config.num_analog_in = 0;
	cards[0].config.num_digital_in = 0;
	cards[0].config.num_digital_out = 0;
	cards[0].config.num_encoders = num_encoders;
	cards[0].config.num_pwm = num_pwmgens;

	rtapi_print("cards[0].config.num_encoders: %i\n", cards[0].config.num_encoders);

	// board type individual pins and parameters
	if (strcmp(cards[0].board_type, "7i76e") == 0)
	{
		cards[1].board_type = "7i76";
	}

	// Export the function
	HAL_EXPORT_FUNCT(cards[0].identifier, ".read", read);
	HAL_EXPORT_FUNCT(cards[0].identifier, ".write", write);

	// Daughter Card 7i76 Inputs/Outputs
	// Segment		Meaning
	// hm2_7i76e	The driver/module name, indicating it's a HostMot2 interface using the 7i76e card as the host interface.
	// .0			The first instance of the card (index 0) — useful if multiple cards are used.
	// .7i76		This indicates the daughter card type — in this case, a 7i76.
	// .0.0			Two things:
	// 1. 			The first daughtercard connector (index 0) on the 7i76e (DB25 or internal expansion).
	// 2. 			The first instance of a daughtercard plugged into that connector.
	// .input-01	This is the actual I/O pin, in this case, input #1.

	// base on the first digit of the sserial_port_0 value the card can be configured in different ways
	// -> see manual 7I76E/7I76ED ETHERNET STEP/DIR PLUS I/O DAUGHTERCARD

	snprintf(cards[1].identifier, sizeof(cards[1].identifier), "hm2_%s.%01d.%s.%01d.%01d", board, counter, cards[1].board_type, daughter_card_connector, daughter_card_instance);

	cards[1].config.num_gpios = 0;
	cards[1].config.num_gpios_out = 0;
	cards[1].config.num_stepgens = 0;
	cards[1].config.num_spindle = 1;
	cards[1].config.num_pwm = 0;
	if (sserial_first_digit == 0)
	{
		cards[1].config.num_analog_in = 0;
		cards[1].config.num_digital_in = 32;
		cards[1].config.num_digital_out = 16;
		cards[1].config.num_encoders = 0;
	}
	else if (sserial_first_digit == 1)
	{
		cards[1].config.num_analog_in = 4;
		cards[1].config.num_digital_in = 32;
		cards[1].config.num_digital_out = 16;
		cards[1].config.num_encoders = 0;
	}
	else if (sserial_first_digit == 2)
	{
		cards[1].config.num_analog_in = 4;
		cards[1].config.num_digital_in = 32;
		cards[1].config.num_digital_out = 16;
		cards[1].config.num_encoders = 2;
	}

	for (int i = 0; i < num_cards; i++)
	{
		p_return = configure_card(i);
	}
	// Allocate arrays

	return hal_ready(comp_id);
}

void rtapi_app_exit(void)
{
	hal_exit(comp_id);
}
