// USB descriptor capture — sequential blocking enumeration
// Reads device, configuration, HID report, and string descriptors
// from a device connected to the USB2 host port.
// Supports composite devices with multiple HID interfaces.

#include <string.h>
#include "imxrt.h"
#include "usb_host.h"
#include "desc_capture.h"
#include "uart.h"

extern void delay(uint32_t msec);

// Build a setup packet for GET_DESCRIPTOR
static usb_setup_t make_get_descriptor(uint8_t type, uint8_t index,
	uint16_t langid, uint16_t length)
{
	usb_setup_t s;
	s.bmRequestType = 0x80; // Device-to-Host, Standard, Device
	s.bRequest = USB_REQ_GET_DESCRIPTOR;
	s.wValue = (type << 8) | index;
	s.wIndex = langid;
	s.wLength = length;
	return s;
}

// Build GET_DESCRIPTOR request for an interface-level request (e.g. HID report)
static usb_setup_t make_get_iface_descriptor(uint8_t type, uint8_t index,
	uint16_t iface, uint16_t length)
{
	usb_setup_t s;
	s.bmRequestType = 0x81; // Device-to-Host, Standard, Interface
	s.bRequest = USB_REQ_GET_DESCRIPTOR;
	s.wValue = (type << 8) | index;
	s.wIndex = iface;
	s.wLength = length;
	return s;
}

// Parse config descriptor to find all interfaces and their interrupt IN endpoints
static bool parse_config_descriptor(captured_descriptors_t *desc)
{
	const uint8_t *p = desc->config_desc;
	const uint8_t *end = p + desc->config_desc_len;
	captured_iface_t *cur_iface = NULL;
	desc->num_ifaces = 0;

	while (p < end) {
		uint8_t dlen = p[0];
		uint8_t dtype = p[1];

		if (dlen < 2 || p + dlen > end) break;

		if (dtype == USB_DESC_INTERFACE && dlen >= 9) {
			// Skip alternate settings
			uint8_t alt_setting = p[3];
			if (alt_setting != 0) {
				cur_iface = NULL;
				p += dlen;
				continue;
			}

			if (desc->num_ifaces < MAX_INTERFACES) {
				cur_iface = &desc->ifaces[desc->num_ifaces++];
				memset(cur_iface, 0, sizeof(*cur_iface));
				cur_iface->iface_num      = p[2];
				cur_iface->iface_class    = p[5];
				cur_iface->iface_subclass = p[6];
				cur_iface->iface_protocol = p[7];

				uart_puts("  Interface ");
				uart_putdec(cur_iface->iface_num);
				uart_puts(": class=");
				uart_putdec(cur_iface->iface_class);
				uart_puts(", subclass=");
				uart_putdec(cur_iface->iface_subclass);
				uart_puts(", protocol=");
				uart_putdec(cur_iface->iface_protocol);
				uart_puts("\r\n");
			} else {
				cur_iface = NULL;
				uart_puts("  WARNING: too many interfaces, skipping\r\n");
			}
		} else if (dtype == USB_DESC_HID && dlen >= 9 && cur_iface != NULL) {
			if (cur_iface->iface_class == 3) {
				uint8_t num_descs = p[5];
				for (uint8_t i = 0; i < num_descs; i++) {
					if (6 + i * 3 + 2 < dlen) {
						uint8_t rtype = p[6 + i * 3];
						uint16_t rlen = p[7 + i * 3] | (p[8 + i * 3] << 8);
						if (rtype == USB_DESC_HID_REPORT) {
							cur_iface->hid_report_desc_len = rlen;
							uart_puts("    HID report desc size = ");
							uart_putdec(rlen);
							uart_puts("\r\n");
						}
					}
				}
			}
		} else if (dtype == USB_DESC_ENDPOINT && dlen >= 7 && cur_iface != NULL) {
			uint8_t ep_addr = p[2];
			uint8_t ep_attr = p[3];
			uint16_t ep_maxpkt = p[4] | (p[5] << 8);
			uint8_t ep_interval = p[6];

			// Interrupt IN endpoint?
			if ((ep_attr & 3) == 3 && (ep_addr & 0x80)) {
				cur_iface->interrupt_ep       = ep_addr;
				cur_iface->interrupt_maxpkt   = ep_maxpkt;
				cur_iface->interrupt_interval = ep_interval;
				uart_puts("    Interrupt IN EP 0x");
				uart_puthex8(ep_addr);
				uart_puts(", maxpkt=");
				uart_putdec(ep_maxpkt);
				uart_puts(", interval=");
				uart_putdec(ep_interval);
				uart_puts("ms\r\n");
			}
		}

		p += dlen;
	}

	return desc->num_ifaces > 0;
}

bool capture_descriptors(captured_descriptors_t *desc)
{
	memset(desc, 0, sizeof(*desc));
	int ret;
	usb_setup_t setup;

	uart_puts("\r\n--- Step 1: GET_DESCRIPTOR(Device, 8) ---\r\n");

	// Step 1: Read first 8 bytes of device descriptor to get bMaxPacketSize0
	setup = make_get_descriptor(USB_DESC_DEVICE, 0, 0, 8);
	ret = usb_host_control_transfer(0, 8, &setup, desc->device_desc);
	if (ret < 0 || ret < 8 || desc->device_desc[0] != 18 ||
	    desc->device_desc[1] != USB_DESC_DEVICE) {
		uart_puts("FAIL: could not read device descriptor (8 bytes)\r\n");
		return false;
	}
	desc->ep0_maxpkt = desc->device_desc[7]; // bMaxPacketSize0
	uart_puts("  bMaxPacketSize0 = ");
	uart_putdec(desc->ep0_maxpkt);
	uart_puts("\r\n");

	// Step 2: SET_ADDRESS
	uart_puts("\r\n--- Step 2: SET_ADDRESS(1) ---\r\n");
	desc->dev_addr = 1;
	setup.bmRequestType = 0x00; // Host-to-Device, Standard, Device
	setup.bRequest = USB_REQ_SET_ADDRESS;
	setup.wValue = desc->dev_addr;
	setup.wIndex = 0;
	setup.wLength = 0;
	ret = usb_host_control_transfer(0, desc->ep0_maxpkt, &setup, NULL);
	if (ret < 0) {
		uart_puts("FAIL: SET_ADDRESS\r\n");
		return false;
	}
	delay(10); // Device needs time to process new address

	// Step 3: Read full device descriptor
	uart_puts("\r\n--- Step 3: GET_DESCRIPTOR(Device, 18) ---\r\n");
	setup = make_get_descriptor(USB_DESC_DEVICE, 0, 0, 18);
	ret = usb_host_control_transfer(desc->dev_addr, desc->ep0_maxpkt,
		&setup, desc->device_desc);
	if (ret < 0 || ret < 18 || desc->device_desc[0] != 18 ||
	    desc->device_desc[1] != USB_DESC_DEVICE) {
		uart_puts("FAIL: full device descriptor\r\n");
		return false;
	}
	desc->device_desc_len = 18;
	uart_puts("  idVendor=0x");
	uart_puthex16(desc->device_desc[8] | (desc->device_desc[9] << 8));
	uart_puts(" idProduct=0x");
	uart_puthex16(desc->device_desc[10] | (desc->device_desc[11] << 8));
	uart_puts("\r\n");

	// Step 4: Read config descriptor header (9 bytes) to get wTotalLength
	uart_puts("\r\n--- Step 4: GET_DESCRIPTOR(Config, 9) ---\r\n");
	setup = make_get_descriptor(USB_DESC_CONFIGURATION, 0, 0, 9);
	ret = usb_host_control_transfer(desc->dev_addr, desc->ep0_maxpkt,
		&setup, desc->config_desc);
	if (ret < 0 || ret < 9 || desc->config_desc[1] != USB_DESC_CONFIGURATION) {
		uart_puts("FAIL: config descriptor header\r\n");
		return false;
	}
	uint16_t total_len = desc->config_desc[2] | (desc->config_desc[3] << 8);
	uart_puts("  wTotalLength = ");
	uart_putdec(total_len);
	uart_puts("\r\n");

	if (total_len > MAX_CONFIG_DESC_SIZE) {
		uart_puts("  WARNING: truncating to ");
		uart_putdec(MAX_CONFIG_DESC_SIZE);
		uart_puts("\r\n");
		total_len = MAX_CONFIG_DESC_SIZE;
	}

	// Step 5: Read full config descriptor
	uart_puts("\r\n--- Step 5: GET_DESCRIPTOR(Config, full) ---\r\n");
	setup = make_get_descriptor(USB_DESC_CONFIGURATION, 0, 0, total_len);
	ret = usb_host_control_transfer(desc->dev_addr, desc->ep0_maxpkt,
		&setup, desc->config_desc);
	if (ret < 0 || ret < 9 || desc->config_desc[1] != USB_DESC_CONFIGURATION) {
		uart_puts("FAIL: full config descriptor\r\n");
		return false;
	}
	desc->config_desc_len = (uint16_t)ret;

	// Step 6: Parse config to find all interfaces and endpoints
	uart_puts("\r\n--- Step 6: Parse config descriptor ---\r\n");
	if (!parse_config_descriptor(desc)) {
		uart_puts("  No interfaces found\r\n");
	}

	// Step 7: Read HID report descriptors for each HID interface
	uart_puts("\r\n--- Step 7: HID Report Descriptors ---\r\n");
	for (uint8_t i = 0; i < desc->num_ifaces; i++) {
		captured_iface_t *iface = &desc->ifaces[i];
		if (iface->iface_class != 3) continue;
		if (iface->hid_report_desc_len == 0) continue;

		uint16_t rdlen = iface->hid_report_desc_len;
		if (rdlen > MAX_HID_REPORT_DESC_SIZE)
			rdlen = MAX_HID_REPORT_DESC_SIZE;

		uart_puts("  Iface ");
		uart_putdec(iface->iface_num);
		uart_puts(": fetching ");
		uart_putdec(rdlen);
		uart_puts(" bytes...\r\n");

		setup = make_get_iface_descriptor(USB_DESC_HID_REPORT, 0,
			iface->iface_num, rdlen);
		ret = usb_host_control_transfer(desc->dev_addr, desc->ep0_maxpkt,
			&setup, iface->hid_report_desc);
		if (ret < 0) {
			uart_puts("    FAIL\r\n");
			iface->hid_report_desc_len = 0;
		} else {
			iface->hid_report_desc_len = ret;
			uart_puts("    Read ");
			uart_putdec(ret);
			uart_puts(" bytes\r\n");
		}
	}

	// Step 8: Read string descriptors
	// First get language ID
	uart_puts("\r\n--- Step 8: String descriptors ---\r\n");
	uint8_t str_buf[MAX_STRING_DESC_SIZE];
	setup = make_get_descriptor(USB_DESC_STRING, 0, 0, 4);
	ret = usb_host_control_transfer(desc->dev_addr, desc->ep0_maxpkt,
		&setup, str_buf);
	uint16_t langid = 0x0409; // Default to English
	if (ret >= 4 && str_buf[1] == USB_DESC_STRING) {
		langid = str_buf[2] | (str_buf[3] << 8);
	}
	uart_puts("  Language ID = 0x");
	uart_puthex16(langid);
	uart_puts("\r\n");

	// Read manufacturer, product, serial strings
	uint8_t string_indices[3] = {
		desc->device_desc[14], // iManufacturer
		desc->device_desc[15], // iProduct
		desc->device_desc[16], // iSerialNumber
	};
	const char *string_names[] = {"Manufacturer", "Product", "Serial"};

	desc->num_strings = 0;
	for (int i = 0; i < 3; i++) {
		if (string_indices[i] == 0) continue;
		if (desc->num_strings >= MAX_STRINGS) break;

		setup = make_get_descriptor(USB_DESC_STRING, string_indices[i],
			langid, MAX_STRING_DESC_SIZE);
		ret = usb_host_control_transfer(desc->dev_addr, desc->ep0_maxpkt,
			&setup, desc->string_desc[desc->num_strings]);
		if (ret > 0) {
			desc->string_desc_len[desc->num_strings] = ret;
			desc->string_index[desc->num_strings] = string_indices[i];

			// Print as ASCII (extract from UTF-16LE)
			uart_puts("  ");
			uart_puts(string_names[i]);
			uart_puts(": ");
			uint8_t *sd = desc->string_desc[desc->num_strings];
			uint8_t slen = sd[0];
			for (int j = 2; j < slen && j < ret; j += 2) {
				uart_putc(sd[j]); // Low byte of UTF-16
			}
			uart_puts("\r\n");
			desc->num_strings++;
		}
	}

	desc->valid = true;
	return true;
}

void dump_descriptors(const captured_descriptors_t *desc)
{
	uart_puts("\r\n========== DEVICE DESCRIPTOR (");
	uart_putdec(desc->device_desc_len);
	uart_puts(" bytes) ==========\r\n");
	uart_hexdump(desc->device_desc, desc->device_desc_len);

	uart_puts("\r\n========== CONFIGURATION DESCRIPTOR (");
	uart_putdec(desc->config_desc_len);
	uart_puts(" bytes) ==========\r\n");
	uart_hexdump(desc->config_desc, desc->config_desc_len);

	for (uint8_t i = 0; i < desc->num_ifaces; i++) {
		const captured_iface_t *iface = &desc->ifaces[i];
		uart_puts("\r\n========== INTERFACE ");
		uart_putdec(iface->iface_num);
		uart_puts(" (class=");
		uart_putdec(iface->iface_class);
		uart_puts(") ==========\r\n");

		if (iface->interrupt_ep) {
			uart_puts("  Interrupt IN EP: 0x");
			uart_puthex8(iface->interrupt_ep);
			uart_puts(", maxpkt=");
			uart_putdec(iface->interrupt_maxpkt);
			uart_puts(", interval=");
			uart_putdec(iface->interrupt_interval);
			uart_puts("ms\r\n");
		}

		if (iface->hid_report_desc_len > 0) {
			uart_puts("  HID Report Descriptor (");
			uart_putdec(iface->hid_report_desc_len);
			uart_puts(" bytes):\r\n");
			uart_hexdump(iface->hid_report_desc, iface->hid_report_desc_len);
		}
	}

	for (uint8_t i = 0; i < desc->num_strings; i++) {
		uart_puts("\r\n========== STRING DESCRIPTOR ");
		uart_putdec(i);
		uart_puts(" (");
		uart_putdec(desc->string_desc_len[i]);
		uart_puts(" bytes) ==========\r\n");
		uart_hexdump(desc->string_desc[i], desc->string_desc_len[i]);
	}

	uart_puts("\r\n========== SUMMARY ==========\r\n");
	uart_puts("EP0 max packet: ");
	uart_putdec(desc->ep0_maxpkt);
	uart_puts("\r\nInterfaces: ");
	uart_putdec(desc->num_ifaces);
	uart_puts("\r\n");
	for (uint8_t i = 0; i < desc->num_ifaces; i++) {
		uart_puts("  [");
		uart_putdec(i);
		uart_puts("] iface=");
		uart_putdec(desc->ifaces[i].iface_num);
		uart_puts(" class=");
		uart_putdec(desc->ifaces[i].iface_class);
		if (desc->ifaces[i].interrupt_ep) {
			uart_puts(" EP=0x");
			uart_puthex8(desc->ifaces[i].interrupt_ep);
		}
		uart_puts("\r\n");
	}
}
