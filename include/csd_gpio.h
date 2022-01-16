#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/adc.h>



static void gpio_setup(void)
{
	/* Enable GPIO clocks. */
	//rcc_periph_clock_enable(RCC_GPIOA);  // already enabled
	//rcc_periph_clock_enable(RCC_GPIOC);  // already enabled

    // Initialize LED
    gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_10_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);
    //gpio_set(GPIOC, GPIO13);
	// turn off LED until in active mode
	gpio_clear(GPIOC, GPIO13);

    // Configure B6 & B7 as INPUT_PULLUP
    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO6);
    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO7);
    gpio_set_mode(GPIOB, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, GPIO6);
    gpio_set_mode(GPIOB, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, GPIO7);
    gpio_set(GPIOB, GPIO6);
    gpio_set(GPIOB, GPIO7);

    // Configure A1 & A2 as analog inputs
    gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_ANALOG, GPIO1);
    gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_ANALOG, GPIO2);
}

static void adc_setup(void)
{
	int i;

	/*****    ADC1   *********/
	rcc_periph_clock_enable(RCC_ADC1);

	/* Make sure the ADC doesn't run during config. */
	adc_power_off(ADC1);

	adc_disable_scan_mode(ADC1);
	//adc_enable_scan_mode(ADC1);
	//adc_set_continuous_conversion_mode(ADC1);
	adc_enable_discontinuous_mode_regular(ADC1, 2);
	//adc_set_single_conversion_mode(ADC1);
	adc_enable_external_trigger_regular(ADC1, ADC_CR2_EXTSEL_SWSTART);
	adc_set_right_aligned(ADC1);
	
	adc_disable_temperature_sensor();

	adc_set_sample_time_on_all_channels(ADC1, ADC_SMPR_SMP_71DOT5CYC);

	adc_power_on(ADC1);

	/* Wait for ADC starting up. */
	for (i = 0; i < 800000; i++)    /* Wait a bit. */
		__asm__("nop");

	adc_reset_calibration(ADC1);
	adc_calibrate(ADC1);
}