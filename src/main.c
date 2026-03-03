// imxrtnsy — USB proxy for Teensy 4.1
// Phase 1: Capture descriptors from device on USB2 (host)
// Phase 2: Replay as device on USB1, forward HID reports
// Supports composite devices with multiple HID interfaces.

#include <stdint.h>
#include <stddef.h>
#include "imxrt.h"
#include "uart.h"
#include "usb_host.h"
#include "usb_device.h"
#include "desc_capture.h"
#include "kmbox.h"
#include "smooth.h"

extern uint32_t millis(void);
extern void delay(uint32_t msec);

// LED on pin 13 (GPIO7 bit 3, configured in startup)
static void led_on(void)  { GPIO7_DR_SET = (1 << 3); }
static void led_off(void) { GPIO7_DR_CLEAR = (1 << 3); }
static void led_toggle(void) __attribute__((unused));
static void led_toggle(void) { GPIO7_DR_TOGGLE = (1 << 3); }
static void led_blink_forever(uint8_t code, uint32_t on_ms, uint32_t off_ms)
{
	while (1) {
		for (uint8_t i = 0; i < code; i++) {
			led_on();
			delay(on_ms);
			led_off();
			delay(off_ms);
		}
		delay(600);
	}
}

static void led_stage(uint8_t n) __attribute__((unused));
static void led_stage(uint8_t n)
{
	for (uint8_t i = 0; i < n; i++) {
		led_on();
		delay(80);
		led_off();
		delay(120);
	}
	delay(250);
}

static void led_wait_once(uint8_t pulses, uint32_t on_ms, uint32_t off_ms, uint32_t gap_ms)
{
	for (uint8_t i = 0; i < pulses; i++) {
		led_on();
		delay(on_ms);
		led_off();
		delay(off_ms);
	}
	delay(gap_ms);
}

// Endpoint mapping: host poll slot -> device EP number
typedef struct {
	uint8_t  host_slot;
	uint8_t  dev_ep_num;
	uint16_t maxpkt;
	uint8_t  iface_protocol;  // 1=keyboard, 2=mouse (for kmbox merge)
} ep_mapping_t;

// ---- PIT0: 1kHz tick for smooth injection timing ----
static volatile bool pit_tick_pending;
static void pit0_isr(void)
{
	PIT_TFLG0 = PIT_TFLG_TIF; // clear interrupt flag
	pit_tick_pending = true;
}

// Static to keep off the stack (~3.6KB struct)
static captured_descriptors_t desc;

int main(void)
{
	// Enable SEVONPEND so WFE wakes on any pending interrupt
	SCB_SCR |= SCB_SCR_SEVONPEND;

	uart_init();
	kmbox_init();

	// PIT0: smooth injection timer — clock/ISR now, rate set after enumeration
	CCM_CCGR1 |= CCM_CCGR1_PIT(CCM_CCGR_ON);
	PIT_MCR = 0; // enable module, timers run in debug
	PIT_TCTRL0 = 0; // stopped until we know the mouse poll interval
	PIT_TFLG0 = PIT_TFLG_TIF;
	attachInterruptVector(IRQ_PIT, pit0_isr);
	NVIC_SET_PRIORITY(IRQ_PIT, 64);
	NVIC_ENABLE_IRQ(IRQ_PIT);

	uart_puts("\r\n\r\nimxrtnsy: USB proxy\r\n");
	uart_puts("Teensy 4.1 — i.MX RT1062\r\n\r\n");
	led_on();

	// ---- Phase 1: Host init + descriptor capture ----

	uart_puts("Initializing USB2 host controller...\r\n");
	usb_host_init();

	uart_puts("Waiting for device on USB host port...\r\n");
	led_off();
	usb_host_power_on();
	uint32_t host_wait_loops = 0;
	while (!usb_host_device_connected()) {
		usb_host_power_on();
		// State: waiting for device on USB2 host port — idle between blinks
		__asm volatile("wfe");
		led_wait_once(1, 70, 120, 650);
		host_wait_loops++;

		// Periodic diagnostic: print PORTSC to help debug connect detection
		if ((host_wait_loops % 5) == 1) {
			uart_puts("  wait PORTSC=0x");
			uart_puthex32(USB2_PORTSC1);
			uart_puts("\r\n");
		}

		// Timeout after ~45s of connect attempts.
		if (host_wait_loops > 60u) {
			// No host-side USB device detected within timeout window
			led_blink_forever(7, 80, 120);
		}
	}

	led_on();
	uart_puts("Device connected!\r\n");
	delay(10);

	uart_puts("Resetting port...\r\n");
	usb_host_port_reset();

	uint8_t speed = usb_host_device_speed();
	uart_puts("Device speed: ");
	if (speed == 0) uart_puts("Full (12 Mbps)");
	else if (speed == 1) uart_puts("Low (1.5 Mbps)");
	else if (speed == 2) uart_puts("High (480 Mbps)");
	uart_puts("\r\n\r\n");

	if (!capture_descriptors(&desc)) {
		uart_puts("\r\n=== CAPTURE FAILED ===\r\n");
		led_blink_forever(5, 100, 100);
	}

	uart_puts("\r\n=== CAPTURE COMPLETE ===\r\n");
	dump_descriptors(&desc);

	// ---- Phase 2: Device stack + proxy ----

	// Set configuration on the real device so it starts generating reports
	uart_puts("\r\nSetting configuration on device...\r\n");
	usb_setup_t setup;
	setup.bmRequestType = 0x00;
	setup.bRequest = USB_REQ_SET_CONFIG;
	setup.wValue = desc.config_desc[5]; // bConfigurationValue
	setup.wIndex = 0;
	setup.wLength = 0;
	int ret = usb_host_control_transfer(desc.dev_addr, desc.ep0_maxpkt,
		&setup, NULL);
	if (ret < 0) {
		uart_puts("  SET_CONFIGURATION failed!\r\n");
	} else {
		uart_puts("  Device configured\r\n");
	}
	delay(10);

	// Initialize host-side interrupt polling for all endpoints
	uart_puts("Initializing interrupt polling...\r\n");

	// Send SET_IDLE(0) and SET_PROTOCOL(1=report) to each HID interface.
	// Some devices (including Razer) won't send interrupt reports until these
	// class-specific requests have been issued.
	for (uint8_t i = 0; i < desc.num_ifaces; i++) {
		// SET_IDLE: bmRequestType=0x21 (class, interface), bRequest=0x0A
		setup.bmRequestType = 0x21;
		setup.bRequest = 0x0A; // SET_IDLE
		setup.wValue = 0;      // duration=0 (indefinite), report ID=0 (all)
		setup.wIndex = i;      // interface number
		setup.wLength = 0;
		ret = usb_host_control_transfer(desc.dev_addr, desc.ep0_maxpkt,
			&setup, NULL);
		uart_puts("  SET_IDLE iface ");
		uart_putdec(i);
		uart_puts(ret < 0 ? " STALL (ok)\r\n" : " ok\r\n");

		// SET_PROTOCOL: bmRequestType=0x21, bRequest=0x0B
		setup.bmRequestType = 0x21;
		setup.bRequest = 0x0B; // SET_PROTOCOL
		setup.wValue = 1;      // 1 = Report Protocol (not Boot Protocol)
		setup.wIndex = i;
		setup.wLength = 0;
		ret = usb_host_control_transfer(desc.dev_addr, desc.ep0_maxpkt,
			&setup, NULL);
		uart_puts("  SET_PROTOCOL iface ");
		uart_putdec(i);
		uart_puts(ret < 0 ? " STALL (ok)\r\n" : " ok\r\n");
	}
	ep_mapping_t ep_map[MAX_INTR_EPS];
	uint8_t num_ep_mappings = 0;

	for (uint8_t i = 0; i < desc.num_ifaces; i++) {
		if (desc.ifaces[i].interrupt_ep == 0) continue;
		if (num_ep_mappings >= MAX_INTR_EPS) {
			uart_puts("  WARNING: too many interrupt EPs, skipping\r\n");
			break;
		}
		uint8_t slot = num_ep_mappings;
		uint8_t ep = desc.ifaces[i].interrupt_ep & 0x0F;

		usb_host_interrupt_init(slot, desc.dev_addr, ep,
			desc.ifaces[i].interrupt_maxpkt);

		ep_map[slot].host_slot       = slot;
		ep_map[slot].dev_ep_num      = ep;
		ep_map[slot].maxpkt          = desc.ifaces[i].interrupt_maxpkt;
		ep_map[slot].iface_protocol  = desc.ifaces[i].iface_protocol;
		num_ep_mappings++;
	}

	// Start PIT0 at the mouse's poll rate (from bInterval + device speed)
	{
		uint32_t interval_us = 1000; // default 1ms = 1kHz
		for (uint8_t i = 0; i < desc.num_ifaces; i++) {
			if (desc.ifaces[i].iface_protocol != 2) continue;
			uint8_t bint = desc.ifaces[i].interrupt_interval;
			if (bint == 0) bint = 1;
			if (speed == 2) {
				// High-speed: 2^(bInterval-1) * 125 µs
				interval_us = 125u << (bint > 1 ? bint - 1 : 0);
			} else {
				// Full/low speed: bInterval in ms
				interval_us = (uint32_t)bint * 1000u;
			}
			break; // use first mouse interface
		}
		// Clamp to [125µs, 10ms] — sane range for smooth injection
		if (interval_us < 125) interval_us = 125;
		if (interval_us > 10000) interval_us = 10000;
		uint32_t ldval = (150u * interval_us) - 1; // IPG = 150 MHz
		PIT_LDVAL0 = ldval;
		PIT_TCTRL0 = PIT_TCTRL_TIE | PIT_TCTRL_TEN;
		uart_puts("  Smooth timer: ");
		uart_putdec(interval_us);
		uart_puts(" us (bInterval=");
		uart_putdec(desc.ifaces[0].interrupt_interval);
		uart_puts(", speed=");
		uart_putdec(speed);
		uart_puts(")\r\n");
	}

	// Initialize USB1 device stack
	uart_puts("Initializing USB1 device stack...\r\n");
	if (!usb_device_init(&desc)) {
		led_blink_forever(9, 80, 120);
	}

	// Wait for host PC to enumerate us — must call usb_device_poll()
	// continuously so SETUP packets are answered within the USB spec
	// timeout.  Blocking LED patterns here cause multi-minute enumeration.
	uart_puts("Waiting for host PC enumeration...\r\n");
	led_off();
	uint32_t dev_wait_start = millis();
	uint32_t dev_led_toggle = millis();
	while (!usb_device_is_configured()) {
		usb_device_poll();
		// Non-blocking LED toggle every 250 ms
		if ((millis() - dev_led_toggle) >= 250) {
			led_toggle();
			dev_led_toggle = millis();
		}

		if ((millis() - dev_wait_start) > 30000) {
			// USB1 device side not being configured by PC host
			led_blink_forever(8, 80, 120);
		}
	}
	uart_puts("Host PC configured us!\r\n");
	led_off();

	// ---- Main proxy loop ----
	// Non-blocking: interrupt_poll primes on first call, checks on subsequent.
	// Hardware NAK rate-limits polling (1 frame = 1ms full-speed, 125us high-speed).
	uart_puts("\r\n=== PROXY ACTIVE (");
	uart_putdec(num_ep_mappings);
	uart_puts(" endpoints) ===\r\n");

	// Diagnostic dump of interrupt schedule state
	usb_host_interrupt_dump_state();

	uint8_t report_buf[64];
	uint32_t report_count = 0;
	uint32_t drop_count = 0;
	uint32_t loop_count = 0;
	uint32_t last_heartbeat = millis();
	uint32_t led_off_time = 0; // non-blocking LED pulse
	uint32_t last_stats = millis();

	while (1) {
		usb_device_poll();
		kmbox_poll();

		// Process smooth queue at PIT-driven 1kHz rate
		if (pit_tick_pending) {
			pit_tick_pending = false;
			int16_t sx, sy;
			smooth_process_frame(&sx, &sy);
			if (sx || sy) kmbox_inject_smooth(sx, sy);
		}

		for (uint8_t m = 0; m < num_ep_mappings; m++) {
			ret = usb_host_interrupt_poll(ep_map[m].host_slot,
				report_buf, ep_map[m].maxpkt);
			if (ret > 0) {
				kmbox_merge_report(ep_map[m].iface_protocol,
					report_buf, ret);
				bool sent = usb_device_send_report(
					ep_map[m].dev_ep_num, report_buf, ret);
				if (sent) {
					report_count++;
				} else {
					drop_count++;
				}

				// Print first few reports for debugging
				if (report_count + drop_count <= 5) {
					uart_puts(sent ? "  FWD " : "  DROP ");
					uart_puts("EP");
					uart_putdec(ep_map[m].dev_ep_num);
					uart_puts(" [");
					uart_putdec(ret);
					uart_puts("]: ");
					for (int i = 0; i < ret && i < 8; i++) {
						uart_puthex8(report_buf[i]);
						uart_putc(' ');
					}
					uart_puts("\r\n");
				}

				// Non-blocking LED pulse on forwarded report
				led_on();
				led_off_time = millis() + 2;
			}
		}

		// Turn off LED after pulse expires
		if (led_off_time && millis() >= led_off_time) {
			led_off();
			led_off_time = 0;
		}

		// Periodic stats every 5 seconds
		if ((millis() - last_stats) >= 5000) {
			uart_puts("  reports=");
			uart_putdec(report_count);
			uart_puts(" drops=");
			uart_putdec(drop_count);
			uart_puts(" PORTSC=0x");
			uart_puthex32(USB2_PORTSC1);
			uart_puts("\r\n");

			// Full diag dump every 5s for first 30s
			if ((millis() - last_stats) < 35000 || report_count == 0) {
				usb_host_interrupt_dump_state();
			}

			last_stats = millis();
		}

		// Send injected-only reports if no real report was merged this cycle
		kmbox_send_pending(&desc);

		// Disconnect check (every ~1024 iterations, not every loop)
		if ((++loop_count & 0x3FF) == 0) {
			if (!usb_host_device_connected()) {
				uart_puts("Mouse disconnected!\r\n");
				led_blink_forever(6, 80, 80);
			}
		}
	}
}
