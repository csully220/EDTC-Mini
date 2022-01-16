
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

//#define BUFSZ 3

bool csdbg[8];
float csflt[8];
int csint[8];

// Output buffer (must match HID report descriptor)
int8_t outbuf[ 4 ];

int main(void)
{
	// Initialized explicitly for clarity and debugging
	outbuf[0] = 0; // slider - thrust
	outbuf[1] = 0; // wheel
	outbuf[2] = 0; // switch bank 1
	outbuf[3] = 0; // switch bank 2

	edtc_report.slider1 = 0;
	edtc_report.slider2 = 0;
	edtc_report.buttons1 = 4;
	edtc_report.buttons2 = 3;

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
		tk1 -= 135;
		if(tk1 > 3665) tk1 = 3665;
		if(tk1 < 10)   tk1 = 0;
		float tf1 = ((float)tk1 / 3665.0 * 255.0) - 127.0;
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

    int8_t swbnk1[16] = {0};
	
	// index directional switch
	gpio_clear(GPIOB, GPIO14);
	gpio_set(GPIOB, GPIO12 | GPIO13);
	swbnk1[0] = !gpio_get(GPIOB, GPIO4); // push
	swbnk1[1] = !gpio_get(GPIOB, GPIO5); // up 
	swbnk1[2] = !gpio_get(GPIOB, GPIO6); // left
	swbnk1[3] = !gpio_get(GPIOB, GPIO7); // down
	//swbnk1[4] = !gpio_get(GPIOB, GPIO8); 
	swbnk1[4] = !gpio_get(GPIOB, GPIO9); // right

    // thumb directional switch
	gpio_clear(GPIOB, GPIO12);
	gpio_set(GPIOB, GPIO13 | GPIO14);
	//gpio_set(GPIOB, GPIO14);
	swbnk1[5] = !gpio_get(GPIOB, GPIO4);  // push
	swbnk1[6] = !gpio_get(GPIOB, GPIO5);  // left
	swbnk1[7] = !gpio_get(GPIOB, GPIO6);  // down
	swbnk1[8] = !gpio_get(GPIOB, GPIO7);  // right
	swbnk1[9] = !gpio_get(GPIOB, GPIO9); // up (inop)
	//swbnk1[11] = !gpio_get(GPIOB, GPIO8);

	// chromatic buttons
	gpio_clear(GPIOB, GPIO13);
	gpio_set(GPIOB, GPIO12 | GPIO14);
	swbnk1[10] = !gpio_get(GPIOB, GPIO4); // BLK, right-side
	swbnk1[11] = !gpio_get(GPIOB, GPIO5); // ORG
	swbnk1[12] = !gpio_get(GPIOB, GPIO6); // BLK, left-side
	swbnk1[13] = !gpio_get(GPIOB, GPIO7); // BRN
	swbnk1[14] = !gpio_get(GPIOB, GPIO8); // YEL
	swbnk1[15] = !gpio_get(GPIOB, GPIO9); // BLU

	int8_t btns1 = swbnk1[0]; // 0;

	for(int i=1;i<8;i++) {
		//(swbnk1[5] << 5) | (swbnk1[4] << 4) | (swbnk1[3] << 3) | (swbnk1[2] << 2) | (swbnk1[1] << 1) | (swbnk1[0]);
		btns1 |= (swbnk1[i] << i);
	}


	int8_t btns2 = swbnk1[8];
	for(int i=9;i<16;i++) {
		//(swbnk1[5] << 5) | (swbnk1[4] << 4) | (swbnk1[3] << 3) | (swbnk1[2] << 2) | (swbnk1[1] << 1) | (swbnk1[0]);
		btns2 |= (swbnk1[i] << (i-8));
	}

	memcpy(&outbuf[0], &edtc_report.slider1, 1);
	memcpy(&outbuf[1], &edtc_report.slider2, 1);
	memcpy(&outbuf[2], &btns1, 1);
	memcpy(&outbuf[3], &btns2, 1);

	usbd_ep_write_packet(usbd_dev, 0x81, &outbuf, sizeof(outbuf));
}