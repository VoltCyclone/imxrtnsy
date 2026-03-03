#pragma once
#include <stdint.h>

void uart_init(void);
void uart_putc(char c);
void uart_puts(const char *s);
void uart_puthex8(uint8_t val);
void uart_puthex16(uint16_t val);
void uart_puthex32(uint32_t val);
void uart_putdec(uint32_t val);
void uart_hexdump(const uint8_t *data, uint32_t len);
