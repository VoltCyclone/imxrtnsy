#pragma once
#include <stdint.h>
#include <stdbool.h>

#define MAX_CONFIG_DESC_SIZE    512
#define MAX_HID_REPORT_DESC_SIZE 512
#define MAX_STRING_DESC_SIZE    128
#define MAX_STRINGS             8
#define MAX_INTERFACES          4

// Per-interface captured data
typedef struct {
	uint8_t  iface_num;            // bInterfaceNumber
	uint8_t  iface_class;          // bInterfaceClass (3 = HID)
	uint8_t  iface_subclass;       // bInterfaceSubClass
	uint8_t  iface_protocol;       // bInterfaceProtocol
	uint8_t  interrupt_ep;         // IN EP addr (0x81 etc), 0 if none
	uint16_t interrupt_maxpkt;     // max packet size for interrupt EP
	uint8_t  interrupt_interval;   // polling interval
	uint8_t  hid_report_desc[MAX_HID_REPORT_DESC_SIZE];
	uint16_t hid_report_desc_len;  // 0 if not HID or not fetched
} captured_iface_t;

typedef struct {
	// Raw device descriptor (18 bytes)
	uint8_t  device_desc[18];
	uint8_t  device_desc_len;

	// Raw configuration descriptor (variable length, includes all
	// interface, endpoint, HID, and class-specific descriptors)
	uint8_t  config_desc[MAX_CONFIG_DESC_SIZE];
	uint16_t config_desc_len;

	// Per-interface data (replaces single-interface fields)
	captured_iface_t ifaces[MAX_INTERFACES];
	uint8_t  num_ifaces;

	// String descriptors (raw USB string descriptor format)
	uint8_t  string_desc[MAX_STRINGS][MAX_STRING_DESC_SIZE];
	uint8_t  string_desc_len[MAX_STRINGS];
	uint8_t  string_index[MAX_STRINGS]; // Original USB string index for each
	uint8_t  num_strings;

	// Max packet size from device descriptor (for EP0)
	uint8_t  ep0_maxpkt;

	// Device address assigned during enumeration
	uint8_t  dev_addr;

	bool valid;
} captured_descriptors_t;

// Perform full descriptor capture from a freshly-reset device.
// Returns true on success.
bool capture_descriptors(captured_descriptors_t *desc);

// Print all captured descriptors over UART.
void dump_descriptors(const captured_descriptors_t *desc);
