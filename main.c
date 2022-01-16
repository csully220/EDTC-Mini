/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2010 Gareth McMullin <gareth@blacksphere.co.nz>
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
#include "wcid.h"
#include <stdlib.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/hid.h>

#include "csd_globals.h"
#include "csd_gpio.h"
#include <string.h>

/* Define this to include the DFU APP interface. */
//#define INCLUDE_DFU_INTERFACE

#ifdef INCLUDE_DFU_INTERFACE
#include <libopencm3/cm3/scb.h>
#include <libopencm3/usb/dfu.h>
#endif

static usbd_device *usbd_dev;



const struct usb_device_descriptor dev_descr = {
	.bLength = USB_DT_DEVICE_SIZE,
	.bDescriptorType = USB_DT_DEVICE,
	.bcdUSB = 0x0200,
	.bDeviceClass = 0,
	.bDeviceSubClass = 0,
	.bDeviceProtocol = 0,
	.bMaxPacketSize0 = 64,
	.idVendor = 0x0C5D,
	.idProduct = 0x0030,
	.bcdDevice = 0x0200,
	.iManufacturer = 1,
	.iProduct = 2,
	.iSerialNumber = 3,
	.bNumConfigurations = 1,
};

static const uint8_t hid_report_descriptor[] = {
	0x05, 0x01, 			// USAGE_PAGE (Generic Desktop)
	0x09, 0x04, 			// USAGE (Joystick)
	0xA1, 0x00, 			// COLLECTION (Application)
		0x09, 0x36, 		//     USAGE (Slider)
		0x15, 0x81, 		//     LOGICAL_MINIMUM (-127)
		0x25, 0x7F, 		//     LOGICAL_MAXIMUM (127)
		0x75, 0x08, 		//     REPORT_SIZE (8)
		0x95, 0x01, 		//     REPORT_COUNT (1)
		0x81, 0x02,			//     INPUT (Data,Var,Abs)

		0x05, 0x01, 		// USAGE_PAGE (Generic Desktop)
		0x09, 0x01, 		// USAGE (Pointer)
		0xA1, 0x00, 		// COLLECTION (Physical)
			0x09, 0x30, 	// 		USAGE (X)
			0x09, 0x31, 	// 		USAGE (Y)
			0x15, 0x81, 	//      LOGICAL_MINIMUM (-127)
			0x25, 0x7F, 	//      LOGICAL_MAXIMUM (127)
			0x95, 0x02,     //  	REPORT_COUNT(2)
			0x81, 0x02,		//  	INPUT (Data,Var,Abs)
		0xc0,       		// END_COLLECTION

		0x05, 0x09, 		// USAGE_PAGE (Buttons)
		0x19, 0x01,  		// USAGE_MINUMUM (Button 1)
		0x29, 0x08,         // USAGE_MAXIMUM (Button 8)
		0x15, 0x00, 		// LOGICAL_MINIMUM (0)
		0x25, 0x01, 		// LOGICAL_MAXIMUM (1)
		0x35, 0x00, 		// PHYSICAL_MINIMUM (0)
		0x45, 0x01, 		// PHYSICAL_MAXIMUM (1)
		0x95, 0x08,     	// REPORT_COUNT(8)
		0x75, 0x01, 		// REPORT_SIZE (1)
		0x65, 0x00,         // UNIT (None)
		0x81, 0x02,			// INPUT (Data,Var,Abs)
	0xc0,       		//   END_COLLECTION	
};

static const struct {
	struct usb_hid_descriptor hid_descriptor;
	struct {
		uint8_t bReportDescriptorType;
		uint16_t wDescriptorLength;
	} __attribute__((packed)) hid_report;
} __attribute__((packed)) hid_function = {
	.hid_descriptor = {
		.bLength = sizeof(hid_function),
		.bDescriptorType = USB_DT_HID,
		.bcdHID = 0x0100,
		.bCountryCode = 0,
		.bNumDescriptors = 1,
	},
	.hid_report = {
		.bReportDescriptorType = USB_DT_REPORT,
		.wDescriptorLength = sizeof(hid_report_descriptor),
	}
};

const struct usb_endpoint_descriptor hid_endpoint = {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = 0x81,
	.bmAttributes = USB_ENDPOINT_ATTR_INTERRUPT,
	.wMaxPacketSize = 4,
	.bInterval = 0x20,
};

const struct usb_interface_descriptor hid_iface = {
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = 0,
	.bAlternateSetting = 0,
	.bNumEndpoints = 1,
	.bInterfaceClass = USB_CLASS_HID,
	.bInterfaceSubClass = 1, /* boot */
	.bInterfaceProtocol = 0, /* 2 for mouse */
	.iInterface = 0,

	.endpoint = &hid_endpoint,

	.extra = &hid_function,
	.extralen = sizeof(hid_function),
};

#ifdef INCLUDE_DFU_INTERFACE
const struct usb_dfu_descriptor dfu_function = {
	.bLength = sizeof(struct usb_dfu_descriptor),
	.bDescriptorType = DFU_FUNCTIONAL,
	.bmAttributes = USB_DFU_CAN_DOWNLOAD | USB_DFU_WILL_DETACH,
	.wDetachTimeout = 255,
	.wTransferSize = 1024,
	.bcdDFUVersion = 0x011A,
};

const struct usb_interface_descriptor dfu_iface = {
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = 1,
	.bAlternateSetting = 0,
	.bNumEndpoints = 0,
	.bInterfaceClass = 0xFE,
	.bInterfaceSubClass = 1,
	.bInterfaceProtocol = 1,
	.iInterface = 0,

	.extra = &dfu_function,
	.extralen = sizeof(dfu_function),
};
#endif

const struct usb_interface ifaces[] = {{
	.num_altsetting = 1,
	.altsetting = &hid_iface,
#ifdef INCLUDE_DFU_INTERFACE
}, {
	.num_altsetting = 1,
	.altsetting = &dfu_iface,
#endif
}};

const struct usb_config_descriptor config = {
	.bLength = USB_DT_CONFIGURATION_SIZE,
	.bDescriptorType = USB_DT_CONFIGURATION,
	.wTotalLength = 0,
#ifdef INCLUDE_DFU_INTERFACE
	.bNumInterfaces = 2,
#else
	.bNumInterfaces = 1,
#endif
	.bConfigurationValue = 1,
	.iConfiguration = 0,
	.bmAttributes = 0xC0,
	.bMaxPower = 0x32,

	.interface = ifaces,
};

static const char *usb_strings[] = {
	"Corvus Sim Devices",
	"CSD Elite Dangerous Thrust Control",
	"CSD EDTC",
};

/* Buffer to be used for control requests. */
uint8_t usbd_control_buffer[128];

static enum usbd_request_return_codes hid_control_request(usbd_device *dev, struct usb_setup_data *req, uint8_t **buf, uint16_t *len,
			void (**complete)(usbd_device *, struct usb_setup_data *))
{
	(void)complete;
	(void)dev;

	if((req->bmRequestType != 0x81) ||
	   (req->bRequest != USB_REQ_GET_DESCRIPTOR) ||
	   (req->wValue != 0x2200))
		return USBD_REQ_NOTSUPP;

	/* Handle the HID report descriptor. */
	*buf = (uint8_t *)hid_report_descriptor;
	*len = sizeof(hid_report_descriptor);

	return USBD_REQ_HANDLED;
}

#ifdef INCLUDE_DFU_INTERFACE
static void dfu_detach_complete(usbd_device *dev, struct usb_setup_data *req)
{
	(void)req;
	(void)dev;

	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ,
		      GPIO_CNF_OUTPUT_PUSHPULL, GPIO10);
	gpio_set(GPIOA, GPIO10);
	scb_reset_core();
}

static enum usbd_request_return_codes dfu_control_request(usbd_device *dev, struct usb_setup_data *req, uint8_t **buf, uint16_t *len,
			void (**complete)(usbd_device *, struct usb_setup_data *))
{
	(void)buf;
	(void)len;
	(void)dev;

	if ((req->bmRequestType != 0x21) || (req->bRequest != DFU_DETACH))
		return USBD_REQ_NOTSUPP; /* Only accept class request. */

	*complete = dfu_detach_complete;

	return USBD_REQ_HANDLED;
}
#endif

static void hid_set_config(usbd_device *dev, uint16_t wValue)
{

	(void)wValue;
	(void)dev;

	usbd_ep_setup(dev, 0x81, USB_ENDPOINT_ATTR_INTERRUPT, 4, NULL);

	usbd_register_control_callback(
				dev,
				USB_REQ_TYPE_STANDARD | USB_REQ_TYPE_INTERFACE,
				USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT,
				hid_control_request);
#ifdef INCLUDE_DFU_INTERFACE
	usbd_register_control_callback(
				dev,
				USB_REQ_TYPE_CLASS | USB_REQ_TYPE_INTERFACE,
				USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT,
				dfu_control_request);
#endif

	systick_set_clocksource(STK_CSR_CLKSOURCE_AHB_DIV8);
	/* SysTick interrupt every N clock pulses: set reload to N-1 */
	systick_set_reload(99999);
	systick_interrupt_enable();
	systick_counter_enable();
}

/*static void csd_clock_setup(void)
{
    rcc_clock_setup_in_hse_8mhz_out_72mhz();
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);
}*/



int main(void)
{
    runMode = STANDBY;

	edtc_report.slider = 0;
	edtc_report.thumb_x = 0;
	edtc_report.thumb_y = 0;
	edtc_report.buttons = 0;

    // Enable required clocks
    //csd_clock_setup();
	rcc_clock_setup_in_hse_8mhz_out_72mhz();
    gpio_setup();
    adc_setup();

    /* ADC channels for conversion*/
	uint8_t channel_array[16] = {0};
	channel_array[0] = 0;
    //channel_array[1] = 1;
	//channel_array[2] = 2;
	
	adc_set_regular_sequence(ADC1, 1, channel_array);

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

    uint32_t frame = 0;

    bool led_on = false;

    gpio_clear(GPIOB, GPIO12);
	gpio_set(GPIOB, GPIO13);
    gpio_set(GPIOB, GPIO14);

	while (1)
	{

		if(led_on)
			gpio_set(GPIOC, GPIO13);
		else
			gpio_clear(GPIOC, GPIO13);

		//ADC1
		adc_start_conversion_regular(ADC1);
		/* Wait for end of conversion. */
        while (!(adc_eoc(ADC1)))
			continue;
		uint16_t tk1 = (adc_read_regular(ADC1) / 16) - 128;
		memcpy(&edtc_report.slider, &tk1, 1);

		//ADC2
		/*adc_start_conversion_regular(ADC1);
        while (!(adc_eoc(ADC1)))
			continue;
		tk1 = (adc_read_regular(ADC1) / 16) - 128;
		*/
		//tk1 = 0;
		//memcpy(&edtc_report.thumb_y, &tk1, 1);

		//ADC3
		/*adc_start_conversion_regular(ADC1);
        while (!(adc_eoc(ADC1)))
			continue;
		tk1 = (adc_read_regular(ADC1) / 16) - 128;
		*/
		//tk1 = 0;
		//memcpy(&edtc_report.thumb_x, &tk1, 1);

		//uint8_t tb[5] = {0};
		/*tb[0] = !gpio_get(GPIOB, GPIO5);
		tb[1] = !gpio_get(GPIOB, GPIO6);
		tb[2] = !gpio_get(GPIOB, GPIO7);
		tb[3] = !gpio_get(GPIOB, GPIO8);
		tb[4] = !gpio_get(GPIOB, GPIO9);
        */
	    // Sink the DOG for switch bank 1

	    //gpio_set(GPIOB, GPIO12);
		//gpio_set(GPIOB, GPIO12 | GPIO13 | GPIO14); // allow reading of column

	    /*uint8_t swbnk1[5] = {0};
	   	swbnk1[0] = !gpio_get(GPIOB, GPIO4);
		swbnk1[1] = !gpio_get(GPIOB, GPIO5);
		swbnk1[2] = !gpio_get(GPIOB, GPIO6);
		swbnk1[3] = !gpio_get(GPIOB, GPIO7);
		swbnk1[4] = !gpio_get(GPIOB, GPIO8);*/

        uint8_t swbnk1[7] = {0};
		swbnk1[0] = !gpio_get(GPIOB, GPIO3);
		swbnk1[1] = !gpio_get(GPIOB, GPIO4);
		swbnk1[2] = !gpio_get(GPIOB, GPIO5);
		swbnk1[3] = !gpio_get(GPIOB, GPIO6);
		swbnk1[4] = !gpio_get(GPIOB, GPIO7);
		swbnk1[5] = !gpio_get(GPIOB, GPIO8);
		swbnk1[6] = !gpio_get(GPIOB, GPIO9);

		uint8_t btns = (swbnk1[6] << 6) | (swbnk1[5] << 5) | (swbnk1[4] << 4) | (swbnk1[3] << 3) | (swbnk1[2] << 2) | (swbnk1[1] << 1) | (swbnk1[0]);
		memcpy(&edtc_report.buttons, &btns, 1);

		//uint8_t btns = (tb[4] << 4) | (tb[3] << 3) | (tb[2] << 2) | (tb[1] << 1) | (tb[0]);
		//memcpy(&edtc_report.buttons, &btns, 1);

		usbd_ep_write_packet(usbd_dev, 0x81, &edtc_report, 4);
		usbd_poll(usbd_dev);

		frame++;

		if(frame > 10000000)
			frame = 0;
	}
}

void sys_tick_handler(void)
{
	
}
