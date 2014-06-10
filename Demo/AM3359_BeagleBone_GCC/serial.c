/****
Hfo @ 2014
****/


#include "am335.h"
#include "serial.h"

static int calc_divisor (unsigned int baud)
{
	return (UART_CLK / 16 / baud);
}

void init_serial(unsigned int base)	{
	int clock_divisor;
	if (base == UART4_BASE)	{
		clock_divisor = calc_divisor(UART4_BAUDRATE);
		(*(REG32(base+0x4))) = 0x0;	
		serial_puts(UART0_BASE,"Init4\n");}
	else	{
		clock_divisor = calc_divisor(UART0_BAUDRATE);
		(*(REG32(base+0x4))) = 0x0;	}
	(*(REG32(base+0xC))) = 0x83; 
	(*(REG32(base+0x0))) = clock_divisor & 0xff;
	(*(REG32(base+0x4))) = (clock_divisor >> 8) & 0xff;
	(*(REG32(base+0xC))) = (MCR_DTR | MCR_RTS);
	(*(REG32(base+0x10))) = 0x3;
	(*(REG32(base+0x8))) = (FCR_FIFO_EN | FCR_RXSR | FCR_TXSR);
}

void serial_putc( unsigned int base, const char c )	{
	while (((*(REG32(base+0x14))) & LSR_THRE) == 0);
	(*(REG32(base+0x00))) = c;
}

void serial_puts (unsigned int base, const char *s)
{
	while (*s) {
		serial_putc (base,*s++);
	}
}

void serial_putsn (unsigned int base, const char *s, int n)
{
	int i;
	for (i=0; i<n; i++)	{
		serial_putc (base,s[i]);
	}
}
