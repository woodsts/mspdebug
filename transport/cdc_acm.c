/* MSPDebug - debugging tool for the eZ430
 * Copyright (C) 2009-2012 Daniel Beer
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef __Windows__
#include <usb.h>
#else
#include <lusb0_usb.h>
#endif

#include "cdc_acm.h"
#include "util.h"
#include "usbutil.h"
#include "output.h"

#define READ_BUFFER_SIZE	1024

struct cdc_acm_transport {
	struct transport            base;

	int                         int_number;
	struct libusb_device_handle *handle;

	int                         in_ep;
	int                         out_ep;

	/* We have to implement an intermediate read buffer, because
	 * some interfaces are buggy and don't like single-byte reads.
	 */
	int                         rbuf_len;
	int                         rbuf_ptr;
	unsigned char               rbuf[READ_BUFFER_SIZE];
};

#define CDC_INTERFACE_CLASS		10

#define TIMEOUT		                30000

/* CDC requests */
#define CDC_REQTYPE_HOST_TO_DEVICE	0x21
#define CDC_SET_CONTROL			0x22
#define CDC_SET_LINE_CODING             0x20

/* Modem control line bitmask */
#define CDC_CTRL_DTR			0x01
#define CDC_CTRL_RTS			0x02

static int usbtr_send(transport_t tr_base, const uint8_t *data, int len)
{
	struct cdc_acm_transport *tr = (struct cdc_acm_transport *)tr_base;
	int sent;
	int rc;

#ifdef DEBUG_CDC_ACM
	debug_hexdump(__FILE__": USB transfer out", data, len);
#endif
	while (len > 0) {
		rc = libusb_bulk_transfer(tr->handle, tr->out_ep,
				      (unsigned char *)data, len, &sent,
				      TIMEOUT);
		if ((rc != 0) && (rc != LIBUSB_ERROR_TIMEOUT)) {
			pr_error(__FILE__": can't send data");
			return -1;
		}

		data += sent;
		len -= sent;
	}

	return 0;
}

static int usbtr_recv(transport_t tr_base, uint8_t *databuf, int len)
{
	struct cdc_acm_transport *tr = (struct cdc_acm_transport *)tr_base;
	int rc;
	int received;

	if (tr->rbuf_ptr >= tr->rbuf_len) {
		tr->rbuf_ptr = 0;
		rc = libusb_bulk_transfer(tr->handle, tr->in_ep,
					  tr->rbuf, sizeof(tr->rbuf),
					  &received, TIMEOUT);

		if ((rc != 0) && (rc != LIBUSB_ERROR_TIMEOUT)) {
			pr_error(__FILE__": can't receive data");
			return -1;
		}

		tr->rbuf_len = received;

#ifdef DEBUG_CDC_ACM
		debug_hexdump(__FILE__": USB transfer in",
			      (uint8_t *)tr->rbuf, tr->rbuf_len);
#endif
	}

	if (tr->rbuf_ptr + len > tr->rbuf_len)
		len = tr->rbuf_len - tr->rbuf_ptr;

	memcpy(databuf, tr->rbuf + tr->rbuf_ptr, len);
	tr->rbuf_ptr += len;

	return len;
}

static void usbtr_destroy(transport_t tr_base)
{
	struct cdc_acm_transport *tr = (struct cdc_acm_transport *)tr_base;

	libusb_release_interface(tr->handle, tr->int_number);
	libusb_unref_device(libusb_get_device(tr->handle));
	libusb_close(tr->handle);
	free(tr);
}

static int usbtr_flush(transport_t tr_base)
{
	struct cdc_acm_transport *tr = (struct cdc_acm_transport *)tr_base;
	unsigned char buf[64];
	int received;

	/* Flush out lingering data */
	while (libusb_bulk_transfer(tr->handle, tr->in_ep, buf, sizeof(buf),
				    &received, 100) != 0);

	tr->rbuf_len = 0;
	tr->rbuf_ptr = 0;
	return 0;
}

static int usbtr_set_modem(transport_t tr_base, transport_modem_t state)
{
	struct cdc_acm_transport *tr = (struct cdc_acm_transport *)tr_base;
	int value = 0;

	if (state & TRANSPORT_MODEM_DTR)
		value |= CDC_CTRL_DTR;
	if (state & TRANSPORT_MODEM_RTS)
		value |= CDC_CTRL_RTS;

#ifdef DEBUG_CDC_ACM
	printc(__FILE__": modem ctrl = 0x%x\n", value);
#endif

	if (libusb_control_transfer(tr->handle, CDC_REQTYPE_HOST_TO_DEVICE,
			    CDC_SET_CONTROL, value, 0,
			    NULL, 0, 300) != 0) {
		pr_error(__FILE__": failed to set modem control lines\n");
		return -1;
	}

	return 0;
}

static const struct transport_class cdc_acm_class = {
	.destroy	= usbtr_destroy,
	.send		= usbtr_send,
	.recv		= usbtr_recv,
	.flush		= usbtr_flush,
	.set_modem	= usbtr_set_modem
};

static int find_interface(struct cdc_acm_transport *tr,
			  struct libusb_device *dev)
{
	int i;
	struct libusb_config_descriptor *c;
	int rc = libusb_get_config_descriptor(dev, 0, &c);

	if (rc != 0) {
		printc_err(__FILE__": can't get configuration: %s\n",
			   libusb_strerror(rc));
		return -1;
	}

	for (i = 0; i < c->bNumInterfaces; i++) {
		const struct libusb_interface *intf = &c->interface[i];
		const struct libusb_interface_descriptor *desc = &intf->altsetting[0];
		int j;

		if (desc->bInterfaceClass != CDC_INTERFACE_CLASS)
			continue;

		/* Look for bulk in/out endpoints */
		tr->in_ep = -1;
		tr->out_ep = -1;

		for (j = 0; j < desc->bNumEndpoints; j++) {
			const struct libusb_endpoint_descriptor *ep =
				&desc->endpoint[j];
			const int type =
				ep->bmAttributes & USB_ENDPOINT_TYPE_MASK;
			const int addr = ep->bEndpointAddress;

			if (type != USB_ENDPOINT_TYPE_BULK)
				continue;

			if (addr & USB_ENDPOINT_DIR_MASK)
				tr->in_ep = addr;
			else
				tr->out_ep = addr;
		}

		if (tr->in_ep >= 0 && tr->out_ep >= 0) {
			tr->int_number = i;
			return 0;
		}
	}

	return -1;
}

static int open_interface(struct cdc_acm_transport *tr,
			  struct libusb_device *dev)
{
	int rc;

	printc_dbg(__FILE__": Trying to open interface %d on %03d:%03d\n",
		   tr->int_number,
		   libusb_get_bus_number(dev), libusb_get_device_address(dev));

	rc = libusb_open(dev, &tr->handle);
	if (!tr->handle) {
		printc_err(__FILE__": can't open device: %s\n",
			   libusb_strerror(rc));
		return -1;
	}

#if defined(__linux__)
	if (libusb_kernel_driver_active(tr->handle, tr->int_number) == 1) {
		printc_dbg(__FILE__": Detaching kernel driver for %03d:%03d\n",
			   libusb_get_bus_number(dev),
			   libusb_get_device_address(dev));
		rc = libusb_detach_kernel_driver(tr->handle, tr->int_number);
		if (rc != 0)
			printc_err(__FILE__": warning: can't detach kernel "
				   "driver: %s", libusb_strerror(rc));
	}
#endif

	rc = libusb_claim_interface(tr->handle, tr->int_number);
	if (rc != 0) {
		printc_err(__FILE__": can't claim interface: %s\n",
			   libusb_strerror(rc));
		libusb_close(tr->handle);
		return -1;
	}

	return 0;
}

static int configure_port(struct cdc_acm_transport *tr, int baud_rate)
{
	uint8_t line_coding[7];
	int rc;

	line_coding[0] = baud_rate & 0xff;
	line_coding[1] = (baud_rate >> 8) & 0xff;
	line_coding[2] = (baud_rate >> 16) & 0xff;
	line_coding[3] = (baud_rate >> 24) & 0xff;
	line_coding[4] = 0; /* 1 stop bit */
	line_coding[5] = 0; /* no parity */
	line_coding[6] = 8; /* 8 data bits */

	rc = libusb_control_transfer(tr->handle, CDC_REQTYPE_HOST_TO_DEVICE,
			             CDC_SET_LINE_CODING, 0, 0,
				     line_coding, 7, 300);
	if (rc != 0) {
		printc_err(__FILE__": failed to set line coding: %s\n",
			   libusb_strerror(rc));
		return -1;
	}

	rc = libusb_control_transfer(tr->handle, CDC_REQTYPE_HOST_TO_DEVICE,
			             CDC_SET_CONTROL, 0, 0, NULL, 0, 300);
	if (rc != 0) {
		printc_err(__FILE__": failed to set modem control lines: %s\n",
			   libusb_strerror(rc));
		return -1;
	}

	return 0;
}

transport_t cdc_acm_open(const char *devpath, const char *requested_serial,
			 int baud_rate, uint16_t vendor, uint16_t product)
{
	struct cdc_acm_transport *tr = malloc(sizeof(*tr));
	struct libusb_device *dev;

	if (!tr) {
		pr_error(__FILE__": can't allocate memory");
		return NULL;
	}

	memset(tr, 0, sizeof(*tr));
	tr->base.ops = &cdc_acm_class;

	libusb_init(NULL);

	if (devpath)
		dev = usbutil_find_by_loc(devpath);
	else
		dev = usbutil_find_by_id(vendor, product, requested_serial);

	if (!dev) {
		free(tr);
		return NULL;
	}

	if (find_interface(tr, dev) < 0) {
		printc_err(__FILE__": failed to locate CDC-ACM interface\n");
		free(tr);
		return NULL;
	}

	if (open_interface(tr, dev) < 0) {
		printc_err(__FILE__": failed to open interface\n");
		free(tr);
		return NULL;
	}

	if (configure_port(tr, baud_rate) < 0) {
		usbtr_destroy((transport_t)tr);
		return NULL;
	}

	usbtr_flush((transport_t)tr);

	return (transport_t)tr;
}
