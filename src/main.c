
#include <stdlib.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/hid.h>
#include <string.h>

#include "csd_usb.h"
#include "csd_gpio.h"


bool csdbg[8];
float csflt[8];
int csint[8];

// Output buffer (must match HID report descriptor)
int8_t outbuf[5];

int8_t swbnk[24] = {0};

//int32_t btns;

int main(void)
{
	// Initialized explicitly for clarity and debugging
	outbuf[0] = 0; // slider - thrust
	outbuf[1] = 0; // wheel
	outbuf[2] = 0; // switch bank 1
	outbuf[3] = 0; // switch bank 2
	outbuf[4] = 0; // switch bank 2

	edtc_report.slider1 = 0;
	edtc_report.slider2 = 0;
	edtc_report.buttons = 0;

	rcc_clock_setup_in_hse_8mhz_out_72mhz();

	gpio_setup();
	adc_setup();


    /* ADC channels for conversion*/
	uint8_t channel_array[2] = {0};
	/*
	 * This is a somewhat common cheap hack to trigger device re-enumeration
	 * on startup.  Assuming a fixed external pullup on D+, (For USB-FS)
	 * setting the pin to output, and driving it explicitly low effectively
	 * "removes" the pullup.  The subsequent USB init will "take over" the
	 * pin, and it will appear as a proper pullup to the host.
	 * The magic delay is somewhat arbitrary, no guarantees on USBIF
	 * compliance here, but "it works" in most places.
	 */
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ,
		GPIO_CNF_OUTPUT_PUSHPULL, GPIO12);
	gpio_clear(GPIOA, GPIO12);
	for (unsigned i = 0; i < 800000; i++) {
		__asm__("nop");
	}

	usbd_dev = usbd_init(&st_usbfs_v1_usb_driver, &dev_descr, &config, usb_strings, 3, usbd_control_buffer, sizeof(usbd_control_buffer));
	usbd_register_set_config_callback(usbd_dev, hid_set_config);

	while (true)
	{
		channel_array[0] = 3;
		adc_set_regular_sequence(ADC1, 1, channel_array);
		//ADC1
		adc_start_conversion_direct(ADC1);
		/* Wait for end of conversion. */
		while (!(adc_eoc(ADC1)))
			continue;
		
		int16_t tk1 = adc_read_regular(ADC1);  // range (135 - 3800)

		csint[0] = tk1;
		tk1 -= 2010;
		if(tk1 > 1790) tk1 = 1790;
		if(tk1 < 10)   tk1 = 0;
		float tf1 = ((float)tk1 / 1790.0 * 255.0) - 127.0;
		if(tf1 > 127.0) tf1 = 127.0;
		if(tf1 < -127.0) tf1 = -127.0;
		tk1 = (int8_t)tf1;
		edtc_report.slider1 = tk1;
		
		//ADC2
		channel_array[0] = 0;
		adc_set_regular_sequence(ADC1, 1, channel_array);
		adc_start_conversion_direct(ADC1);
		while (!(adc_eoc(ADC1)))
			continue;
		int16_t tk2 = adc_read_regular(ADC1);  // range (1400)
		csint[1] = tk2;
		if(tk2 > 1400) tk2 = 1400;
		if(tk2 < 0)    tk2 = 0;
		float tf2 = ((float)tk2 / 1400.0 * 255.0) - 127.0;
		if(tf2 > 127.0) tf2 = 127.0;
		if(tf2 < -127.0) tf2 = -127.0;
		tk2 = (int8_t)tf2;
		edtc_report.slider2 = tk2;

		usbd_poll(usbd_dev);

	}
}

void sys_tick_handler(void)
{
	// index directional switch
	gpio_clear(GPIOB, GPIO14);
	gpio_set(GPIOB, GPIO12 | GPIO13);
	swbnk[0] = !gpio_get(GPIOB, GPIO4); // idx push
	swbnk[1] = !gpio_get(GPIOB, GPIO5); // idx up 
	swbnk[2] = !gpio_get(GPIOB, GPIO7); // idx down
	swbnk[3] = !gpio_get(GPIOB, GPIO9); // idx left
	swbnk[4] = !gpio_get(GPIOB, GPIO6); // idx right
	//swbk1[4] = !gpio_get(GPIOB, GPIO8); // spare
	

    // thumb directional switch
	gpio_clear(GPIOB, GPIO12);
	gpio_set(GPIOB, GPIO13 | GPIO14);
	//gpio_set(GPIOB, GPIO14);
	swbnk[5] = !gpio_get(GPIOB, GPIO4);  // thumb push
	swbnk[6] = !gpio_get(GPIOB, GPIO9); // thumb up
	swbnk[7] = !gpio_get(GPIOB, GPIO6);  // thumb down
	swbnk[8] = !gpio_get(GPIOB, GPIO5);  // thumb left
	swbnk[9] = !gpio_get(GPIOB, GPIO7);  // thumb right
	//swbnk1[11] = !gpio_get(GPIOB, GPIO8); // spare
	
	// chromatic buttons
	gpio_clear(GPIOB, GPIO13);
	gpio_set(GPIOB, GPIO12 | GPIO14);
	swbnk[10] = !gpio_get(GPIOB, GPIO9); // BLU
	swbnk[11] = !gpio_get(GPIOB, GPIO5); // ORG
	swbnk[12] = !gpio_get(GPIOB, GPIO7); // BRN
	swbnk[13] = !gpio_get(GPIOB, GPIO8); // YEL
	swbnk[14] = !gpio_get(GPIOB, GPIO6); // LEFT BLK
	swbnk[15] = !gpio_get(GPIOB, GPIO4); // RIGHT BLK

	// misc buttons and switches
	swbnk[16] = !gpio_get(GPIOB, GPIO15); // Heatsink
	swbnk[17] = 0;
	swbnk[18] = 0;
	swbnk[19] = 0;
	swbnk[20] = 0;
	swbnk[21] = 0;
	swbnk[22] = 0;
	swbnk[23] = 0;

	edtc_report.buttons = 0;
	for(int i=0;i<24;i++) {
		edtc_report.buttons |= (swbnk[i] << i);
	}

	memcpy(&outbuf[0], &edtc_report.slider1, 1);
	memcpy(&outbuf[1], &edtc_report.slider2, 1);
	memcpy(&outbuf[2], &edtc_report.buttons, 3);

	usbd_ep_write_packet(usbd_dev, 0x81, &outbuf, 5);

}