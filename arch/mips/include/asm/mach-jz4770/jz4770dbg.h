#include <asm/mach-jz4770/regs.h>
#include <asm/mach-jz4770/jz4770misc.h>
#include <asm/mach-jz4770/jz4770gpio.h>
#include <asm/mach-jz4770/jz4770uart.h>

static void dserial_putc (const char c)
{
	volatile u8 *uart_lsr = (volatile u8 *)(UART2_BASE + OFF_LSR);
	volatile u8 *uart_tdr = (volatile u8 *)(UART2_BASE + OFF_TDR);

	if (c == '\n') dserial_putc ('\r');

	/* Wait for fifo to shift out some bytes */
	while ( !((*uart_lsr & (UART_LSR_TDRQ | UART_LSR_TEMT)) == 0x60) );

	*uart_tdr = (u8)c;
}

static void dserial_puts (const char *s)
{
	while (*s) {
		dserial_putc (*s++);
	}
}

main()
{
	dserial_puts("\n");
	return 0;
}
