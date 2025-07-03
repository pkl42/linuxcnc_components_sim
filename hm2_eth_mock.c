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

static char *version="0.8";

// simulating configuration of mesa card
static char *config = "";
RTAPI_MP_STRING(config, "Configuration string");
static char *board = "";
static char *daughter_card = "";
RTAPI_MP_STRING(board, "Board Type string");

static int num_stepgens = 5;
static int num_encoders = 1;
static int num_pwmgens = 0;
static char sserial_ports[64] = "";  // optional capture
static int sserial_first_digit = -1;  // Default to invalid

// Parameters (allocated with hal_malloc)
static hal_u32_t *watchdog_timeout_ns;
static hal_bit_t **watchdog_has_bit;
static hal_s32_t *dpll_01_timer_us;
static hal_u32_t *stepgen_timer_number;

// Stepgen
static hal_float_t **pos_cmd, **pos_fb,**velocity_cmd,**velocity_fb;
static hal_bit_t **step, **dir, **control_type, **enable;
static hal_s32_t **counts;

static hal_u32_t *dirSetup, *dirHold,*stepLen,*stepSpace,*stepType;
static hal_float_t *positionScale, *maxAcceleration,*maxVelocity;

static double threat_cycle_time; // cycle time [s] of threat to calculate position made between two calls
static long period_ns=0;

// Encoders
static hal_float_t **enc_pos;
static hal_s32_t **enc_counts;

// GPIO
#define NUM_GPIO 16
static hal_bit_t **gpio_in, **gpio_in_sim, **gpio_out;

// Daughter Card Pins

static int  dcard_num_io_in=32;
static int  dcard_num_io_out=16;
static int  dcard_num_analog_in=4;
static int  dcard_num_encoders=2;

static hal_float_t **ss_analogin, **ss_analogin_sim;

static hal_bit_t **ss_input, **ss_input_sim, **ss_input_not, **ss_output;
static hal_bit_t **spinena,**spindir;
static hal_float_t **field_voltage, **field_voltage_sim;
static hal_float_t **spinout;

// Daughter Card Encodres
static hal_float_t **dcard_enc_pos;
static hal_s32_t **dcard_enc_counts;

// Smart Serial Parameters
static hal_float_t *spinout_minlim,*spinout_maxlim,*spinout_scalemax;


// PWM
static hal_float_t **pwm_val, **pwm_fb;
static hal_bit_t **pwm_enable;

static hal_float_t *pwm_scale;
static hal_u32_t *pwm_frequency;

// Simple helper to extract values like "num_stepgens=3"
static void parse_config_string(const char *config_str) {
    char *config_copy = strdup(config_str);  // make a modifiable copy
    char *token = strtok(config_copy, " ");
    while (token) {
        if (strncmp(token, "num_stepgens=", 13) == 0) {
            num_stepgens = atoi(token + 13);
        } else if (strncmp(token, "num_encoders=", 13) == 0) {
            num_encoders = atoi(token + 13);
        } else if (strncmp(token, "num_pwmgens=", 12) == 0) {
            num_encoders = atoi(token + 13);
        } else if (strncmp(token, "sserial_port_0=", 15) == 0) {
            strncpy(sserial_ports, token + 15, sizeof(sserial_ports) - 1);
            sserial_ports[sizeof(sserial_ports) - 1] = '\0';
			// Extract first digit
            if (isdigit(sserial_ports[0])) {
                sserial_first_digit = sserial_ports[0] - '0';
            }
        }

        token = strtok(NULL, " ");
    }

    free(config_copy);
}



// as the physical card has read and write function both are adapted
// but for simulation only one of them does the simulation job
static void write(void *arg, long period_nsec) { }

static void read(void *arg, long period_nsec)
{
	if (period_nsec != period_ns){
		//threat_cycle_time=(double) period_nsec* 1e-10*5.; due to write function was mapped to read during export
		threat_cycle_time=(double) period_nsec* 1e-9;
		period_ns=period_nsec;
	}


    for (int i = 0; i < num_stepgens; i++) {

        *dir[i] = (*pos_cmd[i] >= 0);
		*velocity_fb[i] = *velocity_cmd[i];		
		/*
		if (maxVelocity[i]< *velocity_fb[i] && *velocity_fb[i]>0)
		{ 
			*velocity_fb[i] =maxVelocity[i];
		}
		else if (-maxVelocity[i]> *velocity_fb[i] && *velocity_fb[i]<0)
		{
			*velocity_fb[i] =-maxVelocity[i];
		}
		*/
		

		if (*control_type[i]== 0)
		{
        	*pos_fb[i] = *pos_cmd[i];
			*step[i] = ((int)(*pos_cmd[i] * 10) % 2);
		}
		else if (*control_type[i]== 1)
		{
			*pos_fb[i] +=(float)((double)*velocity_fb[i]* threat_cycle_time);
			*counts[i] = (hal_s32_t)(*pos_fb[i] *positionScale[i]);
		}

    }

	/* TODO: how to simulate
    for (int i = 0; i < num_encoders; i++) {
        *enc_pos[i] += 0.01;
        *enc_counts[i] = (int)(*enc_pos[i] * 1000);
    }
	*/

    for (int i = 0; i < NUM_GPIO; i++) {
        *gpio_out[i] = *gpio_in[i];
		//*gpio_in[i] = *gpio_in_sim[i];
    }

	// 7i76e Daughter Card 7i76

    for (int i = 0; i < dcard_num_io_in; i++) {
        *ss_input[i] = *ss_input_sim[i];
    }

    for (int i = 0; i < dcard_num_io_in; i++) {
        *ss_input_not[i] = !*ss_input[i];
    }

    for (int i = 0; i < dcard_num_analog_in; i++) {
        *ss_analogin[i] = *ss_analogin_sim[i];
    }

	/* TODO: how to simulate
    for (int i = 0; i < dcard_num_encoders; i++) {
        *dcard_enc_pos[i] += 0.01;
        *dcard_enc_counts[i] = (int)(*enc_pos[i] * 1000);
    }
	*/


	**field_voltage= **field_voltage_sim;

    **pwm_fb = **pwm_val;
    **pwm_enable = (**pwm_val > 0.1);

}

int rtapi_app_main(void)
{
    char name[64]; // needed for hal_helpers
	char card_identifier[64];
	int counter=0;
	to_lowercase(board);
	rtapi_print("Board string: %s\n", board);
	rtapi_print("hm2_eth_mock Version: %s\n", version);
	// TODO: The supported boards are: 7I76E, 7I80DB, 7I80HD, 7I92, 7I93, 7I94, 7I95, 7I96, 7I96S, 7I97, 7I98
	// currently only supported: 7I76E

	daughter_card = "7i76";
	int daughter_card_connector = 0;
	int daughter_card_instance = 0;

	parse_config_string(config);

    comp_id = hal_init("hm2_eth_mock");
    if (comp_id < 0) return comp_id;

    // Parameters (hal_malloc + hal_param_*_new)
    watchdog_timeout_ns = hal_malloc(sizeof(hal_u32_t));
    dpll_01_timer_us    = hal_malloc(sizeof(hal_s32_t));
    stepgen_timer_number = hal_malloc(sizeof(hal_u32_t));
    watchdog_has_bit    = hal_malloc(sizeof(hal_bit_t *));

    if (!watchdog_timeout_ns || !dpll_01_timer_us || !stepgen_timer_number )
        return -ENOMEM;

	snprintf(card_identifier, sizeof(card_identifier), "hm2_%s.%01d", board, counter);
	HAL_PARAM_U32(watchdog_timeout_ns,card_identifier,".watchdog.timeout_ns") ;
	HAL_PIN_BIT(watchdog_has_bit, card_identifier,".watchdog.has_bit", HAL_OUT)  ; 
	HAL_PARAM_S32( dpll_01_timer_us,card_identifier,".dpll.01.timer-us") ;
	HAL_PARAM_U32( stepgen_timer_number,card_identifier,".stepgen.timer-number") ;


    // Stepgen
	HAL_PIN_BIT_ARRAY(control_type, num_stepgens, card_identifier,".stepgen.%02d.control-type", HAL_IN);
	HAL_PIN_FLOAT_ARRAY(pos_cmd, num_stepgens, card_identifier,".stepgen.%02d.position-cmd", HAL_IN);
	HAL_PIN_FLOAT_ARRAY(pos_fb, num_stepgens, card_identifier,".stepgen.%02d.position-fb", HAL_OUT);
	HAL_PIN_BIT_ARRAY(step, num_stepgens, card_identifier,".stepgen.%02d.step", HAL_OUT);
	HAL_PIN_BIT_ARRAY(dir, num_stepgens, card_identifier,".stepgen.%02d.dir", HAL_OUT);
	HAL_PIN_BIT_ARRAY(enable, num_stepgens, card_identifier,".stepgen.%02d.enable", HAL_IN);
	HAL_PIN_S32_ARRAY(counts, num_stepgens, card_identifier,".stepgen.%02d.counts", HAL_OUT);

	HAL_PIN_FLOAT_ARRAY(velocity_cmd, num_stepgens, card_identifier,".stepgen.%02d.velocity-cmd", HAL_IN);
	HAL_PIN_FLOAT_ARRAY(velocity_fb, num_stepgens, card_identifier,".stepgen.%02d.velocity-fb", HAL_OUT);

	HAL_PARAM_U32_ARRAY(dirSetup, num_stepgens, card_identifier,".stepgen.%02d.dirsetup");
	HAL_PARAM_U32_ARRAY(dirHold, num_stepgens, card_identifier,".stepgen.%02d.dirhold");
	HAL_PARAM_U32_ARRAY(stepLen, num_stepgens, card_identifier,".stepgen.%02d.steplen");
	HAL_PARAM_U32_ARRAY(stepSpace, num_stepgens, card_identifier,".stepgen.%02d.stepspace");

	HAL_PARAM_FLOAT_ARRAY(positionScale, num_stepgens, card_identifier,".stepgen.%02d.position-scale");
	HAL_PARAM_U32_ARRAY(stepType, num_stepgens, card_identifier,".stepgen.%02d.step_type");
	HAL_PARAM_FLOAT_ARRAY(maxAcceleration, num_stepgens, card_identifier,".stepgen.%02d.maxaccel");
	HAL_PARAM_FLOAT_ARRAY(maxVelocity, num_stepgens, card_identifier,".stepgen.%02d.maxvel");

    // Encoders
	HAL_PIN_FLOAT_ARRAY(enc_pos, num_encoders, card_identifier,".encoder.%02d.position", HAL_OUT);
	HAL_PIN_S32_ARRAY(enc_counts, num_encoders, card_identifier,".encoder.%02d.counts", HAL_OUT);
 
    // GPIO
	HAL_PIN_BIT_ARRAY(gpio_in, NUM_GPIO, card_identifier,".gpio.%03d.in", HAL_IN);
	HAL_PIN_BIT_ARRAY(gpio_out, NUM_GPIO, card_identifier,".gpio.%03d.out", HAL_OUT);


    // PWM
	HAL_PIN_FLOAT(pwm_val,card_identifier, ".pwmgen.00.value", HAL_IN);
	HAL_PIN_FLOAT(pwm_fb, card_identifier,".pwmgen.00.feedback", HAL_OUT);
	HAL_PIN_BIT(pwm_enable,card_identifier, ".pwmgen.00.enable", HAL_IN);

	HAL_PARAM_FLOAT(pwm_scale, card_identifier,".pwmgen.00.scale");
	HAL_PARAM_U32(pwm_frequency, card_identifier,".pwmgen.pwm_frequency");

    // Export the function
	HAL_EXPORT_FUNCT(card_identifier,".read", read);
	HAL_EXPORT_FUNCT(card_identifier,".write", write);

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
	if (sserial_first_digit == 0){
		dcard_num_io_out=16;
		dcard_num_analog_in=0;
		dcard_num_encoders=0;
	}
	else if (sserial_first_digit == 1)
	{
		dcard_num_io_out=16;
		dcard_num_analog_in=4;
		dcard_num_encoders=0;

	}
	else if (sserial_first_digit == 2)
	{
		dcard_num_io_out=16;
		dcard_num_analog_in=4;
		dcard_num_encoders=2;

	}

	snprintf(card_identifier, sizeof(card_identifier), "hm2_%s.%01d.%s.%01d.%01d", board, counter, daughter_card,daughter_card_connector,daughter_card_instance);
	HAL_PIN_FLOAT_ARRAY(ss_analogin, dcard_num_analog_in,card_identifier, ".analogin%01d", HAL_OUT);
	HAL_PIN_FLOAT_ARRAY(ss_analogin_sim, dcard_num_analog_in,card_identifier, ".analogin%01d-sim", HAL_IN);

	// Encoders
	HAL_PIN_FLOAT_ARRAY(dcard_enc_pos, dcard_num_encoders, card_identifier,".enc%01d.position", HAL_OUT);
	HAL_PIN_S32_ARRAY(dcard_enc_counts, dcard_num_encoders, card_identifier,".enc%01d.counts", HAL_OUT);

	HAL_PIN_FLOAT(field_voltage, card_identifier,".fieldvoltage", HAL_OUT);
    HAL_PIN_FLOAT(field_voltage_sim,card_identifier, ".fieldvoltage-sim", HAL_IN);
	
	HAL_PIN_BIT_ARRAY(ss_input, dcard_num_io_in, card_identifier, ".input-%02d", HAL_OUT);
	HAL_PIN_BIT_ARRAY(ss_input_sim, dcard_num_io_in, card_identifier, ".input-%02d-sim", HAL_IN);
	HAL_PIN_BIT_ARRAY(ss_input_not, dcard_num_io_in, card_identifier, ".input-%02d-not", HAL_OUT);
	HAL_PIN_BIT_ARRAY(ss_output, dcard_num_io_out, card_identifier, ".output-%02d", HAL_IN);

	HAL_PIN_BIT(spinena, card_identifier,".spinena", HAL_IN);
	HAL_PIN_BIT(spindir, card_identifier,".spindir", HAL_IN);
	HAL_PIN_FLOAT(spinout, card_identifier,".spinout", HAL_IN);

	HAL_PARAM_FLOAT(spinout_minlim,card_identifier, ".spinout-minlim");
	HAL_PARAM_FLOAT(spinout_maxlim, card_identifier,".spinout-maxlim");
	HAL_PARAM_FLOAT(spinout_scalemax, card_identifier,".spinout-scalemax");

    return hal_ready(comp_id);
}


void rtapi_app_exit(void)
{
    hal_exit(comp_id);
}
