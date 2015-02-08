

#define S3C64XX_GPACON			(S3C64XX_GPA_BASE + 0x00)
#define S3C64XX_GPADAT			(S3C64XX_GPA_BASE + 0x04)
#define S3C64XX_GPAPUD			(S3C64XX_GPA_BASE + 0x08)
#define S3C64XX_GPACONSLP		(S3C64XX_GPA_BASE + 0x0c)
#define S3C64XX_GPAPUDSLP		(S3C64XX_GPA_BASE + 0x10)

#define S3C64XX_GPA_CONMASK(__gpio)	(0xf << ((__gpio) * 4))
#define S3C64XX_GPA_INPUT(__gpio)	(0x0 << ((__gpio) * 4))
#define S3C64XX_GPA_OUTPUT(__gpio)	(0x1 << ((__gpio) * 4))

#define S3C64XX_GPA0_UART_RXD0		(0x02 << 0)
#define S3C64XX_GPA0_EINT_G1_0		(0x07 << 0)

#define S3C64XX_GPA1_UART_TXD0		(0x02 << 4)
#define S3C64XX_GPA1_EINT_G1_1		(0x07 << 4)

#define S3C64XX_GPA2_UART_nCTS0		(0x02 << 8)
#define S3C64XX_GPA2_EINT_G1_2		(0x07 << 8)

#define S3C64XX_GPA3_UART_nRTS0		(0x02 << 12)
#define S3C64XX_GPA3_EINT_G1_3		(0x07 << 12)

#define S3C64XX_GPA4_UART_RXD1		(0x02 << 16)
#define S3C64XX_GPA4_EINT_G1_4		(0x07 << 16)

#define S3C64XX_GPA5_UART_TXD1		(0x02 << 20)
#define S3C64XX_GPA5_EINT_G1_5		(0x07 << 20)

#define S3C64XX_GPA6_UART_nCTS1		(0x02 << 24)
#define S3C64XX_GPA6_EINT_G1_6		(0x07 << 24)

#define S3C64XX_GPA7_UART_nRTS1		(0x02 << 28)
#define S3C64XX_GPA7_EINT_G1_7		(0x07 << 28)

