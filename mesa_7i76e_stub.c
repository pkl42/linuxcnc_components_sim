#include "rtapi.h"
#include "rtapi_app.h"
#include "hal.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "hal_helpers.h"

MODULE_AUTHOR("Peter Ludwig");
MODULE_DESCRIPTION("Software stub for Mesa 7I76E I/O card, enabling off-target simulation in LinuxCNC");
MODULE_LICENSE("GPL");

static int comp_id;

// simulating configuration of mesa card
static char *config = "";
RTAPI_MP_STRING(config, "Configuration string");
static int num_stepgens = 5;
static int num_encoders = 3;
static int num_pwmgens = 0;
static char sserial_ports[64] = "";  // optional capture

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

// Smart Serial Pins
#define NUM_SSERIAL_IO_IN 32
#define NUM_SSERIAL_IO_OUT 16
#define NUM_SSERIAL_ANALOG_IN 4
static hal_float_t **ss_analogin, **ss_analogin_sim;

static hal_bit_t **ss_input, **ss_input_sim, **ss_input_not, **ss_output;
static hal_bit_t **spinena,**spindir;
static hal_float_t **field_voltage, **field_voltage_sim;
static hal_float_t **spinout;

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
		threat_cycle_time=(double) period_nsec* 1e-10*5.;
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

    for (int i = 0; i < num_encoders; i++) {
        *enc_pos[i] += 0.01;
        *enc_counts[i] = (int)(*enc_pos[i] * 1000);
    }

    for (int i = 0; i < NUM_GPIO; i++) {
        *gpio_out[i] = *gpio_in[i];
		//*gpio_in[i] = *gpio_in_sim[i];
    }

    for (int i = 0; i < NUM_SSERIAL_IO_IN; i++) {
        *ss_input[i] = *ss_input_sim[i];
    }

    for (int i = 0; i < NUM_SSERIAL_IO_IN; i++) {
        *ss_input_not[i] = !*ss_input[i];
    }

    for (int i = 0; i < NUM_SSERIAL_ANALOG_IN; i++) {
        *ss_analogin[i] = *ss_analogin_sim[i];
    }

	**field_voltage= **field_voltage_sim;

    **pwm_fb = **pwm_val;
    **pwm_enable = (**pwm_val > 0.1);

}

int rtapi_app_main(void)
{
    char name[64]; // needed for hal_helpers

	parse_config_string(config);

    comp_id = hal_init("mesa_7i76e_stub");
    if (comp_id < 0) return comp_id;

    // Parameters (hal_malloc + hal_param_*_new)
    watchdog_timeout_ns = hal_malloc(sizeof(hal_u32_t));
    dpll_01_timer_us    = hal_malloc(sizeof(hal_s32_t));
    stepgen_timer_number = hal_malloc(sizeof(hal_u32_t));
    watchdog_has_bit    = hal_malloc(sizeof(hal_bit_t *));

    if (!watchdog_timeout_ns || !dpll_01_timer_us || !stepgen_timer_number )
        return -ENOMEM;

    hal_param_u32_new("hm2_7i76e.0.watchdog.timeout_ns", HAL_RW, watchdog_timeout_ns, comp_id);
    hal_pin_bit_new("hm2_7i76e.0.watchdog.has_bit", HAL_OUT, watchdog_has_bit, comp_id);
    hal_param_s32_new("hm2_7i76e.0.dpll.01.timer-us", HAL_RW, dpll_01_timer_us, comp_id);
    hal_param_u32_new("hm2_7i76e.0.stepgen.timer-number", HAL_RW, stepgen_timer_number, comp_id);

    // Stepgen

	HAL_PIN_BIT_ARRAY(control_type, num_stepgens, "hm2_7i76e.0.stepgen.%02d.control-type", HAL_IN);
	HAL_PIN_FLOAT_ARRAY(pos_cmd, num_stepgens, "hm2_7i76e.0.stepgen.%02d.position-cmd", HAL_IN);
	HAL_PIN_FLOAT_ARRAY(pos_fb, num_stepgens, "hm2_7i76e.0.stepgen.%02d.position-fb", HAL_OUT);
	HAL_PIN_BIT_ARRAY(step, num_stepgens, "hm2_7i76e.0.stepgen.%02d.step", HAL_OUT);
	HAL_PIN_BIT_ARRAY(dir, num_stepgens, "hm2_7i76e.0.stepgen.%02d.dir", HAL_OUT);
	HAL_PIN_BIT_ARRAY(enable, num_stepgens, "hm2_7i76e.0.stepgen.%02d.enable", HAL_IN);
	HAL_PIN_S32_ARRAY(counts, num_stepgens, "hm2_7i76e.0.stepgen.%02d.counts", HAL_OUT);

	HAL_PIN_FLOAT_ARRAY(velocity_cmd, num_stepgens, "hm2_7i76e.0.stepgen.%02d.velocity-cmd", HAL_IN);
	HAL_PIN_FLOAT_ARRAY(velocity_fb, num_stepgens, "hm2_7i76e.0.stepgen.%02d.velocity-fb", HAL_OUT);

	HAL_PARAM_U32_ARRAY(dirSetup, num_stepgens, "hm2_7i76e.0.stepgen.%02d.dirsetup");
	HAL_PARAM_U32_ARRAY(dirHold, num_stepgens, "hm2_7i76e.0.stepgen.%02d.dirhold");
	HAL_PARAM_U32_ARRAY(stepLen, num_stepgens, "hm2_7i76e.0.stepgen.%02d.steplen");
	HAL_PARAM_U32_ARRAY(stepSpace, num_stepgens, "hm2_7i76e.0.stepgen.%02d.stepspace");

	HAL_PARAM_FLOAT_ARRAY(positionScale, num_stepgens, "hm2_7i76e.0.stepgen.%02d.position-scale");
	HAL_PARAM_U32_ARRAY(stepType, num_stepgens, "hm2_7i76e.0.stepgen.%02d.step_type");
	HAL_PARAM_FLOAT_ARRAY(maxAcceleration, num_stepgens, "hm2_7i76e.0.stepgen.%02d.maxaccel");
	HAL_PARAM_FLOAT_ARRAY(maxVelocity, num_stepgens, "hm2_7i76e.0.stepgen.%02d.maxvel");



    // Encoders

	HAL_PIN_FLOAT_ARRAY(enc_pos, num_encoders, "hm2_7i76e.0.encoder.%02d.position", HAL_OUT);
	HAL_PIN_S32_ARRAY(enc_counts, num_encoders, "hm2_7i76e.0.encoder.%02d.counts", HAL_OUT);
 
    // GPIO

	HAL_PIN_BIT_ARRAY(gpio_in, NUM_GPIO, "hm2_7i76e.0.gpio.%03d.in", HAL_IN);
	HAL_PIN_BIT_ARRAY(gpio_out, NUM_GPIO, "hm2_7i76e.0.gpio.%03d.out", HAL_OUT);

    // 7i76 Inputs/Outputs


	HAL_PIN_FLOAT_ARRAY(ss_analogin, NUM_SSERIAL_ANALOG_IN,"hm2_7i76e.0.7i76.0.0.analogin%01d", HAL_OUT);
	HAL_PIN_FLOAT_ARRAY(ss_analogin_sim, NUM_SSERIAL_ANALOG_IN,"hm2_7i76e.0.7i76.0.0.analogin%01d-sim", HAL_IN);


	HAL_PIN_FLOAT(field_voltage, "hm2_7i76e.0.7i76.0.0.fieldvoltage", HAL_OUT);
    HAL_PIN_FLOAT(field_voltage_sim, "hm2_7i76e.0.7i76.0.0.fieldvoltage-sim", HAL_IN);

	HAL_PIN_BIT_ARRAY(ss_input, NUM_SSERIAL_IO_IN, "hm2_7i76e.0.7i76.0.0.input-%02d", HAL_OUT);
	HAL_PIN_BIT_ARRAY(ss_input_sim, NUM_SSERIAL_IO_IN, "hm2_7i76e.0.7i76.0.0.input-%02d-sim", HAL_IN);
	HAL_PIN_BIT_ARRAY(ss_input_not, NUM_SSERIAL_IO_IN, "hm2_7i76e.0.7i76.0.0.input-%02d-not", HAL_OUT);
	HAL_PIN_BIT_ARRAY(ss_output, NUM_SSERIAL_IO_OUT, "hm2_7i76e.0.7i76.0.0.output-%02d", HAL_IN);

	HAL_PIN_BIT(spinena, "hm2_7i76e.0.7i76.0.0.spinena", HAL_IN);
	HAL_PIN_BIT(spindir, "hm2_7i76e.0.7i76.0.0.spindir", HAL_IN);
	HAL_PIN_FLOAT(spinout, "hm2_7i76e.0.7i76.0.0.spinout", HAL_IN);

	HAL_PARAM_FLOAT(spinout_minlim, "hm2_7i76e.0.7i76.0.0.spinout-minlim");
	HAL_PARAM_FLOAT(spinout_maxlim, "hm2_7i76e.0.7i76.0.0.spinout-maxlim");
	HAL_PARAM_FLOAT(spinout_scalemax, "hm2_7i76e.0.7i76.0.0.spinout-scalemax");



    // PWM
	HAL_PIN_FLOAT(pwm_val, "hm2_7i76e.0.pwmgen.00.value", HAL_IN);
	HAL_PIN_FLOAT(pwm_fb, "hm2_7i76e.0.pwmgen.00.feedback", HAL_OUT);
	HAL_PIN_BIT(pwm_enable, "hm2_7i76e.0.pwmgen.00.enable", HAL_IN);

	HAL_PARAM_FLOAT(pwm_scale, "hm2_7i76e.0.pwmgen.00.scale");
	HAL_PARAM_U32(pwm_frequency, "hm2_7i76e.0.pwmgen.pwm_frequency");

    // Export the function
	hal_export_funct("hm2_7i76e.0.read", read, 0, 0, 0, comp_id);
	hal_export_funct("hm2_7i76e.0.write", read, 0, 0, 0, comp_id);


    return hal_ready(comp_id);
}


void rtapi_app_exit(void)
{
    hal_exit(comp_id);
}
