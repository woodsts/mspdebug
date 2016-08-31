/* MSPDebug - debugging tool for MSP430 MCUs
 * Copyright (C) 2009-2013 Daniel Beer
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

#ifndef __Windows__
#include <usb.h>
#else
#include <lusb0_usb.h>
#endif
#include <stdlib.h>
#include <string.h>
#include <limits.h>
#include "usbutil.h"
#include "output.h"
#include "output_util.h"
#include "bslhid.h"

#define BSLHID_VID		0x2047
#define BSLHID_PID		0x0200

#define BSLHID_CLASS		USB_CLASS_HID

#define BSLHID_XFER_SIZE	64
#define BSLHID_MTU		(BSLHID_XFER_SIZE - 2)
#define BSLHID_HEADER		0x3F
#define BSLHID_TIMEOUT		5000

struct bslhid_transport {
	struct transport		base;

	int				cfg_number;
	int				int_number;

	struct libusb_device_handle	*handle;

	int				in_ep;
	int				out_ep;

	char				path[8];
	char				serial[128];
};

static int find_interface(struct bslhid_transport *tr,
			  const struct libusb_device *dev)
{
	int i;
	struct libusb_config_descriptor *c;
	int rc = libusb_get_config_descriptor((struct libusb_device *)dev, 0, &c);

	if (rc != 0) {
		printc_err("bslhid: can't get configuration: %s\n",
			   libusb_strerror(rc));
		return -1;
	}

	for (i = 0; i < c->bNumInterfaces; i++) {
		const struct libusb_interface *intf = &c->interface[i];
		const struct libusb_interface_descriptor *desc = &intf->altsetting[0];
		int j;

		if (desc->bInterfaceClass != BSLHID_CLASS)
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
			tr->cfg_number = c->bConfigurationValue;
			tr->int_number = i;
			printc_dbg("Opening interface %d (config %d)...\n",
				   tr->int_number, tr->cfg_number);
			printc_dbg("Found endpoints: IN: 0x%02x, OUT: 0x%02x\n",
				   tr->in_ep, tr->out_ep);
			return 0;
		}

		printc_err("bslhid: can't find suitable endpoints\n");
	}

	printc_err("bslhid: can't find a matching interface\n");

	return -1;
}

static int open_device(struct bslhid_transport *tr, struct libusb_device *dev)
{
	struct libusb_device_descriptor desc;
	int rc = libusb_get_device_descriptor(dev, &desc);

	if (rc != 0) {
		printc_err("bslhid: can't get device descriptor: %s\n",
			   libusb_strerror(rc));
		return -1;
	}

	if (find_interface(tr, dev) != 0) {
		return -1;
	}

	printc_dbg("bslhid: Trying to open interface %d on %03d:%03d %04x:%04x\n",
		   tr->int_number,
		   libusb_get_bus_number(dev), libusb_get_device_address(dev),
		   desc.idVendor, desc.idProduct);

	rc = libusb_open(dev, &tr->handle);
	if (!tr->handle) {
		printc_err("bslhid: can't open device: %s\n",
			   libusb_strerror(rc));
		return -1;
	}

#ifdef __linux__
	if (libusb_kernel_driver_active(tr->handle, tr->int_number) == 1) {
		printc_dbg("bslhid: Detaching kernel driver for %03d:%03d %04x:%04x\n",
			   libusb_get_bus_number(dev), libusb_get_device_address(dev),
			   desc.idVendor, desc.idProduct);
		rc = libusb_detach_kernel_driver(tr->handle, tr->int_number);
		if (rc != 0)
			printc_err("bslhid: warning: can't detach kernel driver: %s\n",
				   libusb_strerror(rc));
	}
#endif

#ifdef __Windows__
	rc = libusb_set_configuration(tr->handle, tr->cfg_number);
	if (rc != 0) {
		printc_err("bslhid: can't set configuration: %s\n",
			   libusb_strerror(rc));
		libusb_close(tr->handle);
		return -1;
	}
#endif

	rc = libusb_claim_interface(tr->handle, tr->int_number);
	if (rc != 0) {
		printc_err("ftdi: can't claim interface: %s\n",
			   libusb_strerror(rc));
		libusb_close(tr->handle);
		return -1;
	}

	return 0;
}

static void bslhid_destroy(transport_t base)
{
	struct bslhid_transport *tr = (struct bslhid_transport *)base;

	if (tr) {
		if (tr->handle) {
			libusb_release_interface(tr->handle, tr->int_number);
			libusb_unref_device(libusb_get_device(tr->handle));
			libusb_close(tr->handle);
		}
		free(tr);
	}
}

static int bslhid_flush(transport_t base)
{
#ifndef __APPLE__
	struct bslhid_transport *tr = (struct bslhid_transport *)base;
	uint8_t inbuf[BSLHID_XFER_SIZE];
	int received;

	if (!tr->handle)
		return 0;

	/* Flush out lingering data */
	while (libusb_bulk_transfer(tr->handle, tr->in_ep,
			     inbuf, sizeof(inbuf),
			     &received, 100) != 0);
#endif

	return 0;
}

static int bslhid_send(transport_t base, const uint8_t *data, int len)
{
	struct bslhid_transport *tr = (struct bslhid_transport *)base;
	uint8_t outbuf[BSLHID_XFER_SIZE];
	int rc;
	int sent;

	if (!tr->handle) {
		printc_err("bslhid: send on suspended device\n");
		return -1;
	}

	memset(outbuf, 0xac, sizeof(outbuf));

	if (len > BSLHID_MTU) {
		printc_err("bslhid: send in excess of MTU: %d\n", len);
		return -1;
	}

	outbuf[0] = BSLHID_HEADER;
	outbuf[1] = len;
	memcpy(outbuf + 2, data, len);

#ifdef DEBUG_BSLHID
	debug_hexdump("bslhid_send", outbuf, sizeof(outbuf));
#endif

	while (len) {
		rc = libusb_bulk_transfer(tr->handle, tr->out_ep,
				      (unsigned char *)data, len, &sent,
				      BSLHID_TIMEOUT);
		if ((rc != 0) && (rc != LIBUSB_ERROR_TIMEOUT)) {
			pr_error("bslhid: can't send data");
			return -1;
		}

		data += sent;
		len -= sent;
	}

	return 0;
}

static int bslhid_recv(transport_t base, uint8_t *data, int max_len)
{
	struct bslhid_transport *tr = (struct bslhid_transport *)base;
	uint8_t inbuf[BSLHID_XFER_SIZE];
	int rc;
	int r;
	int len;

	if (!tr->handle) {
		printc_err("bslhid: recv on suspended device\n");
		return -1;
	}

	rc = libusb_bulk_transfer(tr->handle, tr->in_ep,
			  (unsigned char *)inbuf, sizeof(inbuf), &r, BSLHID_TIMEOUT);
	if ((rc != 0) && (rc != LIBUSB_ERROR_TIMEOUT)) {
		printc_err("bslhid_recv: usb_bulk_read: %s\n", libusb_strerror(rc));
		return -1;
	}

#ifdef DEBUG_BSLHID
	debug_hexdump("bslhid_recv", inbuf, r);
#endif

	if (r < 2) {
		printc_err("bslhid_recv: short transfer\n");
		return -1;
	}

	if (inbuf[0] != BSLHID_HEADER) {
		printc_err("bslhid_recv: missing transfer header\n");
		return -1;
	}

	len = inbuf[1];
	if ((len > max_len) || (len + 2 > r)) {
		printc_err("bslhid_recv: bad length: %d (%d byte transfer)\n",
			   len, r);
		return -1;
	}

	memcpy(data, inbuf + 2, len);

	return len;
}

static int bslhid_set_modem(transport_t base, transport_modem_t state)
{
	printc_err("bslhid: unsupported operation: set_modem\n");

	return -1;
}

static int bslhid_suspend(transport_t base)
{
	struct bslhid_transport *tr = (struct bslhid_transport *)base;

	if (tr->handle) {
		libusb_release_interface(tr->handle, tr->int_number);
		libusb_close(tr->handle);
		tr->handle = NULL;
	}

	return 0;
}

static int bslhid_resume(transport_t base)
{
	struct bslhid_transport *tr = (struct bslhid_transport *)base;
	struct libusb_device *dev;

	if (tr->handle)
		return 0;

	libusb_init(NULL);

	dev = usbutil_find_by_loc((const char *)tr->path);
	if (!dev) {
		printc_err("bslhid: failed to find BSL HID device on resume\n");
		return -1;
	}

	if (open_device(tr, dev) < 0) {
		printc_err("bslhid: failed to resume BSL HID device\n");
		return -1;
	}

	return 0;
}

static const struct transport_class bslhid_transport_class = {
	.destroy	= bslhid_destroy,
	.send		= bslhid_send,
	.recv		= bslhid_recv,
	.flush		= bslhid_flush,
	.set_modem	= bslhid_set_modem,
	.suspend	= bslhid_suspend,
	.resume		= bslhid_resume
};

transport_t bslhid_open(const char *dev_path, const char *requested_serial)
{
	struct bslhid_transport *tr = malloc(sizeof(*tr));
	struct libusb_device *dev;
	unsigned int len;

	if (!tr) {
		pr_error("bslhid: can't allocate memory");
		return NULL;
	}

	memset(tr, 0, sizeof(*tr));

	tr->base.ops = &bslhid_transport_class;

	libusb_init(NULL);

	if (dev_path) {
		len = strlen(dev_path);
		memset(tr->serial, 0, sizeof(tr->serial));
		memcpy(tr->path, dev_path,
		       len < sizeof(tr->path) ?
		       len : (sizeof(tr->path) - 1));
		dev = usbutil_find_by_loc(dev_path);
	} else {
		len = strlen(requested_serial);
		memset(tr->path, 0, sizeof(tr->path));
		memcpy(tr->serial, requested_serial,
		       len < sizeof(tr->serial) ?
		       len : (sizeof(tr->serial) - 1));
		dev = usbutil_find_by_id(BSLHID_VID, BSLHID_PID,
					 requested_serial);
	}

	if (!dev) {
		free(tr);
		return NULL;
	}

	if (open_device(tr, dev) < 0) {
		printc_err("bslhid: failed to open BSL HID device\n");
		free(tr);
		return NULL;
	}

	bslhid_flush(&tr->base);

	return &tr->base;
}
