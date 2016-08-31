/* MSPDebug - debugging tool for MSP430 MCUs
 * Copyright (C) 2012 Daniel Beer
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

#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "ftdi.h"
#include "util.h"
#include "usbutil.h"
#include "output.h"

struct ftdi_transport {
	struct transport base;
	struct libusb_device_handle *handle;
};

#define USB_INTERFACE           0
#define USB_CONFIG              1

#define EP_IN                   0x81
#define EP_OUT                  0x02

#define TIMEOUT_S               30
#define REQ_TIMEOUT_MS          100

#define REQTYPE_HOST_TO_DEVICE  0x40

#define FTDI_SIO_RESET			0 /* Reset the port */
#define FTDI_SIO_MODEM_CTRL		1 /* Set the modem control register */
#define FTDI_SIO_SET_FLOW_CTRL		2 /* Set flow control register */
#define FTDI_SIO_SET_BAUD_RATE		3 /* Set baud rate */
#define FTDI_SIO_SET_DATA		4 /* Set the data characteristics of
					     the port */
#define FTDI_SIO_GET_MODEM_STATUS	5 /* Retrieve current value of modem
					     status register */
#define FTDI_SIO_SET_EVENT_CHAR		6 /* Set the event character */
#define FTDI_SIO_SET_ERROR_CHAR		7 /* Set the error character */
#define FTDI_SIO_SET_LATENCY_TIMER	9 /* Set the latency timer */
#define FTDI_SIO_GET_LATENCY_TIMER	10 /* Get the latency timer */

#define FTDI_SIO_RESET_SIO              0
#define FTDI_SIO_RESET_PURGE_RX         1
#define FTDI_SIO_RESET_PURGE_TX         2

#define FTDI_PACKET_SIZE                64

#define FTDI_CLOCK			3000000

#define FTDI_DTR			0x0001
#define FTDI_RTS			0x0002
#define FTDI_WRITE_DTR			0x0100
#define FTDI_WRITE_RTS			0x0200

static int do_cfg(struct libusb_device_handle *handle, const char *what,
		  int request, int value)
{
	int rc = libusb_control_transfer(handle, REQTYPE_HOST_TO_DEVICE,
			    request, value, 0,
			    NULL, 0, REQ_TIMEOUT_MS);
	if (rc != 0) {
		printc_err("ftdi: %s failed: %s\n", what, libusb_strerror(rc));
		return -1;
	}

	return 0;
}

int configure_ftdi(struct libusb_device_handle *h, int baud_rate)
{
	if (do_cfg(h, "reset FTDI",
		   FTDI_SIO_RESET, FTDI_SIO_RESET_SIO) < 0 ||
	    do_cfg(h, "set data characteristics",
		   FTDI_SIO_SET_DATA, 8) < 0 ||
	    do_cfg(h, "disable flow control",
		   FTDI_SIO_SET_FLOW_CTRL, 0) < 0 ||
	    do_cfg(h, "set modem control lines",
		   FTDI_SIO_MODEM_CTRL, 0x303) < 0 ||
	    do_cfg(h, "set baud rate",
		   FTDI_SIO_SET_BAUD_RATE, FTDI_CLOCK / baud_rate) < 0 ||
	    do_cfg(h, "set latency timer",
		   FTDI_SIO_SET_LATENCY_TIMER, 50) < 0 ||
	    do_cfg(h, "purge TX",
		   FTDI_SIO_RESET, FTDI_SIO_RESET_PURGE_TX) < 0 ||
	    do_cfg(h, "purge RX",
		   FTDI_SIO_RESET, FTDI_SIO_RESET_PURGE_RX) < 0)
		return -1;

	return 0;
}

static int open_device(struct ftdi_transport *tr, struct libusb_device *dev,
		       int baud_rate)
{
	struct libusb_device_descriptor desc;
	int rc = libusb_get_device_descriptor(dev, &desc);

	if (rc != 0) {
		printc_err("ftdi: can't get device descriptor: %s\n",
			   libusb_strerror(rc));
		return -1;
	}

	printc_dbg("ftdi: trying to open interface %d on %03d:%03d %04x:%04x\n",
		   USB_INTERFACE,
		   libusb_get_bus_number(dev), libusb_get_device_address(dev),
		   desc.idVendor, desc.idProduct);

	rc = libusb_open(dev, &tr->handle);
	if (!tr->handle) {
		printc_err("ftdi: can't open device: %s\n",
			   libusb_strerror(rc));
		return -1;
	}

#ifdef __linux__
	if (libusb_kernel_driver_active(tr->handle, USB_INTERFACE) == 1) {
		printc_dbg("ftdi: Detaching kernel driver for %03d:%03d %04x:%04x\n",
			   libusb_get_bus_number(dev), libusb_get_device_address(dev),
			   desc.idVendor, desc.idProduct);
		rc = libusb_detach_kernel_driver(tr->handle, USB_INTERFACE);
		if (rc != 0)
			printc_err("ftdi: warning: can't detach kernel driver: %s\n",
				   libusb_strerror(rc));
	}
#endif

#ifdef __Windows__
	rc = libusb_set_configuration(tr->handle, USB_CONFIG);
	if (rc != 0) {
		printc_err("ftdi: can't set configuration: %s\n",
			   libusb_strerror(rc));
		libusb_close(tr->handle);
		return -1;
	}
#endif

	rc = libusb_claim_interface(tr->handle, USB_INTERFACE);
	if (rc != 0) {
		printc_err("ftdi: can't claim interface: %s\n",
			   libusb_strerror(rc));
		libusb_close(tr->handle);
		return -1;
	}

	if (configure_ftdi(tr->handle, baud_rate) < 0) {
		libusb_close(tr->handle);
		return -1;
	}

	return 0;
}

static void tr_destroy(transport_t tr_base)
{
	struct ftdi_transport *tr = (struct ftdi_transport *)tr_base;

	libusb_close(tr->handle);

	free(tr);
}

static int tr_recv(transport_t tr_base, uint8_t *databuf, int max_len)
{
	struct ftdi_transport *tr = (struct ftdi_transport *)tr_base;
	int rc;
	int received;
	unsigned char tmpbuf[FTDI_PACKET_SIZE];
	time_t deadline = time(NULL) + TIMEOUT_S;

	if (max_len > FTDI_PACKET_SIZE - 2)
		max_len = FTDI_PACKET_SIZE - 2;

	while(time(NULL) < deadline) {
		rc = libusb_bulk_transfer(tr->handle, EP_IN,
				      tmpbuf, max_len + 2, &received,
				      TIMEOUT_S * 1000);

		if ((rc != 0) && (rc != LIBUSB_ERROR_TIMEOUT)) {
			printc_err("ftdi: usb_bulk_read: %s\n",
				   libusb_strerror(rc));
			return -1;
		}

		if ((rc == 0) && (received > 2)) {
			memcpy(databuf, tmpbuf + 2, received - 2);
#ifdef DEBUG_OLIMEX_ISO
			printc_dbg("ftdi: tr_recv: flags = %02x %02x\n",
				   tmpbuf[0], tmpbuf[1]);
			debug_hexdump("ftdi: tr_recv", databuf, received - 2);
#endif
			return received - 2;
		}
	}

	printc_err("ftdi: timed out while receiving data\n");
	return -1;
}

static int tr_send(transport_t tr_base, const uint8_t *databuf, int len)
{
	struct ftdi_transport *tr = (struct ftdi_transport *)tr_base;
	int rc;
	int sent;

#ifdef DEBUG_OLIMEX_ISO
	debug_hexdump("ftdi: tr_send", databuf, len);
#endif
	while (len) {
		rc = libusb_bulk_transfer(tr->handle, EP_OUT,
					  (unsigned char *)databuf, len, &sent,
					  TIMEOUT_S * 1000);

		if ((rc != 0) && (rc != LIBUSB_ERROR_TIMEOUT)) {
			printc_err("ftdi: libusb_bulk_transfer: %s\n",
				   libusb_strerror(rc));
			return -1;
		}

		databuf += sent;
		len -= sent;
	}

	return 0;
}

static int tr_flush(transport_t tr_base)
{
	struct ftdi_transport *tr = malloc(sizeof(*tr));

	return do_cfg(tr->handle, "purge RX",
		      FTDI_SIO_RESET, FTDI_SIO_RESET_PURGE_RX);
}

static int tr_set_modem(transport_t tr_base, transport_modem_t state)
{
	struct ftdi_transport *tr = malloc(sizeof(*tr));
	int value = FTDI_WRITE_DTR | FTDI_WRITE_RTS;

	/* DTR and RTS bits are active-low for this device */
	if (!(state & TRANSPORT_MODEM_DTR))
		value |= FTDI_DTR;
	if (!(state & TRANSPORT_MODEM_RTS))
		value |= FTDI_RTS;

	return do_cfg(tr->handle, "set modem control lines",
		      FTDI_SIO_MODEM_CTRL, value);
}

static const struct transport_class ftdi_class = {
	.destroy	= tr_destroy,
	.send		= tr_send,
	.recv		= tr_recv,
	.flush		= tr_flush,
	.set_modem	= tr_set_modem
};

transport_t ftdi_open(const char *devpath,
		      const char *requested_serial,
		      uint16_t vendor, uint16_t product,
		      int baud_rate)
{
	struct ftdi_transport *tr = malloc(sizeof(*tr));
	struct libusb_device *dev;

	if (!tr) {
		pr_error("ftdi: can't allocate memory");
		return NULL;
	}

	memset(tr, 0, sizeof(*tr));

	tr->base.ops = &ftdi_class;

	libusb_init(NULL);

	if (devpath)
		dev = usbutil_find_by_loc(devpath);
	else
		dev = usbutil_find_by_id(vendor, product, requested_serial);

	if (!dev) {
		free(tr);
		return NULL;
	}

	if (open_device(tr, dev, baud_rate) < 0) {
		printc_err("ftdi: failed to open device\n");
		free(tr);
		return NULL;
	}

	return &tr->base;
}
