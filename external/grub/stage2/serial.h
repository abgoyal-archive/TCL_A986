/* serial.h - serial device interface */

#ifndef GRUB_SERIAL_HEADER
#define GRUB_SERIAL_HEADER	1

/* Macros.  */

/* The offsets of UART registers.  */
#define UART_TX		0
#define UART_RX		0
#define UART_DLL	0
#define UART_IER	1
#define UART_DLH	1
#define UART_IIR	2
#define UART_FCR	2
#define UART_LCR	3
#define UART_MCR	4
#define UART_LSR	5
#define UART_MSR	6
#define UART_SR		7

/* For LSR bits.  */
#define UART_DATA_READY		0x01
#define UART_EMPTY_TRANSMITTER	0x20

/* The type of parity.  */
#define UART_NO_PARITY		0x00
#define UART_ODD_PARITY		0x08
#define UART_EVEN_PARITY	0x18

/* The type of word length.  */
#define UART_5BITS_WORD	0x00
#define UART_6BITS_WORD	0x01
#define UART_7BITS_WORD	0x02
#define UART_8BITS_WORD	0x03

/* The type of the length of stop bit.  */
#define UART_1_STOP_BIT		0x00
#define UART_2_STOP_BITS	0x04

/* the switch of DLAB.  */
#define UART_DLAB	0x80

/* Enable the FIFO.  */
#define UART_ENABLE_FIFO	0xC7

/* Turn on DTR, RTS, and OUT2.  */
#define UART_ENABLE_MODEM	0x0B


/* Function prototypes.  */

/* Fetch a key.  */
int serial_hw_fetch (void);

/* Put a character.  */
void serial_hw_put (int c);

/* Insert a delay.  */
void serial_hw_delay (void);

/* Return the port number for the UNITth serial device.  */
unsigned short serial_hw_get_port (int unit);

/* Initialize a serial device.  */
int serial_hw_init (unsigned short port, unsigned int speed,
		    int word_len, int parity, int stop_bit_len);

#ifdef GRUB_UTIL
void serial_set_device (const char *device);
#endif /* GRUB_UTIL */

#endif /* ! GRUB_SERIAL_HEADER */
