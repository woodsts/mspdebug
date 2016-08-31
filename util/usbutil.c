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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "usbutil.h"
#include "util.h"
#include "output.h"

static const char *device_help(const struct libusb_device_descriptor *desc)
{
	static const struct {
		int vendor;
		int product;
		const char *help;
	} info[] = {
		{0x0451, 0xf432, "eZ430-RF2500"},
		{0x0451, 0xf430, "FET430UIF"},
		{0x2047, 0x0010, "FET430UIF (V3 firmware)"},
		{0x15ba, 0x0002, "Olimex MSP430-JTAG-TINY (v1)"},
		{0x15ba, 0x0008, "Olimex MSP430-JTAG-ISO"},
		{0x15ba, 0x0031, "Olimex MSP430-JTAG-TINY (v2)"},
		{0x15ba, 0x0100, "Olimex MSP430-JTAG-ISO-MK2 (v2)"},
		{0x2047, 0x0200, "USB bootstrap loader"}
	};
	int i;

	for (i = 0; i < ARRAY_LEN(info); i++)
		if (desc->idProduct == info[i].product &&
		    desc->idVendor == info[i].vendor)
			return info[i].help;

	return "";
}

static int read_serial(struct libusb_device *dev, unsigned char *buf,
		       int max_len)
{
	struct libusb_device_handle *dh;
	struct libusb_device_descriptor desc;

	if (libusb_open(dev, &dh) != 0)
		return -1;

	if (libusb_get_device_descriptor(dev, &desc) != 0) {
		libusb_close(dh);
		return -1;
	}

	if (libusb_get_string_descriptor_ascii(dh, desc.iSerialNumber,
				               buf, max_len) < 0) {
		libusb_close(dh);
		return -1;
	}

	libusb_close(dh);

	return 0;
}

void usbutil_list(void)
{
	libusb_device **devs;
	libusb_device *dev;
	int i = 0;
	int bus = -1;

	libusb_get_device_list(NULL, &devs);

	while ((dev = devs[i++]) != NULL) {
		unsigned char serial[128];
		struct libusb_device_descriptor desc;

		if (libusb_get_device_descriptor(dev, &desc) != 0) {
			continue;
		}

		if (bus != libusb_get_bus_number(dev)) {
			bus = libusb_get_bus_number(dev);
			printc("Devices on bus %03d:\n", bus);
		}

		printc("    %03d:%03d %04x:%04x %s",
		       bus, libusb_get_device_address(dev),
		       desc.idVendor, desc.idProduct,
		       device_help(&desc));

		if (!read_serial(dev, serial, sizeof(serial)))
			printc(" [serial: %s]\n", serial);
		else
			printc("\n");
	}

	libusb_free_device_list(devs, 1);
}

struct libusb_device *usbutil_find_by_id(int vendor, int product,
				      const char *requested_serial)
{
	libusb_device **devs;
	libusb_device *dev;
	libusb_device *found = NULL;
	int i = 0;

	libusb_get_device_list(NULL, &devs);

	while ((dev = devs[i++]) != NULL) {
		struct libusb_device_descriptor desc;

		if (libusb_get_device_descriptor(dev, &desc) != 0) {
			continue;
		}

		if (desc.idVendor == vendor && desc.idProduct == product) {
			unsigned char buf[128];
			if (!requested_serial ||
					(!read_serial(dev, buf, sizeof(buf)) &&
					 !strcasecmp(requested_serial, (char *)buf))) {
				found = dev;
			} else {
				libusb_unref_device(dev);
			}
		}
	}

	libusb_free_device_list(devs, 0);

	if (!found)
		printc_err("usbutil: unable to find vendor=%d, product=%d, "
			   "serial=%s\n", vendor, product,
			   requested_serial ? requested_serial : "");

	return found;
}

struct libusb_device *usbutil_find_by_loc(const char *loc)
{
	int i = 0;
	char buf[64];
	char *bus_text;
	char *dev_text;
	int target_bus;
	int target_dev;
	libusb_device **devs;
	libusb_device *dev;
	libusb_device *found = NULL;

	if (!loc)
		return NULL;

	strncpy(buf, loc, sizeof(buf));
	buf[sizeof(buf) - 1] = 0;

	bus_text = strtok(buf, ":\t\r\n");
	dev_text = strtok(NULL, ":\t\r\n");

	if (!(bus_text && dev_text)) {
		printc_err("usbutil: location must be specified as "
			"<bus>:<device>\n");
		return NULL;
	}

	target_bus = atoi(bus_text);
	target_dev = atoi(dev_text);

	libusb_get_device_list(NULL, &devs);

	while ((dev = devs[i++]) != NULL) {
		if ((libusb_get_bus_number(dev) == target_bus) &&
				(libusb_get_device_address(dev) == target_dev)) {
			found = dev;
		} else {
			libusb_unref_device(dev);
		}
	}

	libusb_free_device_list(devs, 0);

	if (!found)
		printc_err("usbutil: unable to find %03d:%03d\n",
			   target_bus, target_dev);

	return found;
}
