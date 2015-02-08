
#ifndef __ASM_ARCH_SERIAL_L7200_H
#define __ASM_ARCH_SERIAL_L7200_H

#include <mach/memory.h>

#define BASE_BAUD 3686400

#define UART1_BASE	(IO_BASE + 0x00044000)
#define UART2_BASE	(IO_BASE + 0x00045000)

#define UARTDR			0x00	/* Tx/Rx data */
#define RXSTAT			0x04	/* Rx status */
#define H_UBRLCR		0x08	/* mode register high */
#define M_UBRLCR		0x0C	/* mode reg mid (MSB of baud)*/
#define L_UBRLCR		0x10	/* mode reg low (LSB of baud)*/
#define UARTCON			0x14	/* control register */
#define UARTFLG			0x18	/* flag register */
#define UARTINTSTAT		0x1C	/* FIFO IRQ status register */
#define UARTINTMASK		0x20	/* FIFO IRQ mask register */

#define BR_110			0x827
#define BR_1200			0x06e
#define BR_2400			0x05f
#define BR_4800			0x02f
#define BR_9600			0x017
#define BR_14400		0x00f
#define BR_19200		0x00b
#define BR_38400		0x005
#define BR_57600		0x003
#define BR_76800 		0x002
#define BR_115200		0x001

#define RXSTAT_NO_ERR		0x00	/* No error */
#define RXSTAT_FRM_ERR		0x01	/* Framing error */
#define RXSTAT_PAR_ERR		0x02	/* Parity error */
#define RXSTAT_OVR_ERR		0x04	/* Overrun error */

#define UBRLCR_BRK		0x01	/* generate break on tx */
#define UBRLCR_PEN		0x02	/* enable parity */
#define UBRLCR_PDIS		0x00	/* disable parity */
#define UBRLCR_EVEN		0x04	/* 1= even parity,0 = odd parity */
#define UBRLCR_STP2		0x08	/* transmit 2 stop bits */
#define UBRLCR_FIFO		0x10	/* enable FIFO */
#define UBRLCR_LEN5		0x60	/* word length5 */
#define UBRLCR_LEN6		0x40	/* word length6 */
#define UBRLCR_LEN7		0x20	/* word length7 */
#define UBRLCR_LEN8		0x00	/* word length8 */

#define UARTCON_UARTEN		0x01	/* Enable UART */
#define UARTCON_DMAONERR	0x08	/* Mask RxDmaRq when errors occur */

#define UARTFLG_UTXFF		0x20	/* Transmit FIFO full */
#define UARTFLG_URXFE		0x10	/* Receiver FIFO empty */
#define UARTFLG_UBUSY		0x08	/* Transmitter busy */
#define UARTFLG_DCD		0x04	/* Data carrier detect */
#define UARTFLG_DSR		0x02	/* Data set ready */
#define UARTFLG_CTS		0x01	/* Clear to send */

#define UART_TXINT		0x01	/* TX interrupt */
#define UART_RXINT		0x02	/* RX interrupt */
#define UART_RXERRINT		0x04	/* RX error interrupt */
#define UART_MSINT		0x08	/* Modem Status interrupt */
#define UART_UDINT		0x10	/* UART Disabled interrupt */
#define UART_ALLIRQS		0x1f	/* All interrupts */

#endif
