
#ifndef _ASM_ETRAXGPIO_H
#define _ASM_ETRAXGPIO_H

/* etraxgpio _IOC_TYPE, bits 8 to 15 in ioctl cmd */
#ifdef CONFIG_ETRAX_ARCH_V10
#define ETRAXGPIO_IOCTYPE 43
#define GPIO_MINOR_A 0
#define GPIO_MINOR_B 1
#define GPIO_MINOR_LEDS 2
#define GPIO_MINOR_G 3
#define GPIO_MINOR_LAST 3
#endif

#ifdef CONFIG_ETRAXFS
#define ETRAXGPIO_IOCTYPE 43
#define GPIO_MINOR_A 0
#define GPIO_MINOR_B 1
#define GPIO_MINOR_LEDS 2
#define GPIO_MINOR_C 3
#define GPIO_MINOR_D 4
#define GPIO_MINOR_E 5
#ifdef CONFIG_ETRAX_VIRTUAL_GPIO
#define GPIO_MINOR_V 6
#define GPIO_MINOR_LAST 6
#else
#define GPIO_MINOR_LAST 5
#endif
#endif

#ifdef CONFIG_CRIS_MACH_ARTPEC3
#define ETRAXGPIO_IOCTYPE 43
#define GPIO_MINOR_A 0
#define GPIO_MINOR_B 1
#define GPIO_MINOR_LEDS 2
#define GPIO_MINOR_C 3
#define GPIO_MINOR_D 4
#ifdef CONFIG_ETRAX_VIRTUAL_GPIO
#define GPIO_MINOR_V 6
#define GPIO_MINOR_LAST 6
#else
#define GPIO_MINOR_LAST 4
#endif
#define GPIO_MINOR_PWM0 16
#define GPIO_MINOR_PWM1 17
#define GPIO_MINOR_PWM2 18
#define GPIO_MINOR_LAST_PWM GPIO_MINOR_PWM2
#endif

/* supported ioctl _IOC_NR's */

#define IO_READBITS  0x1  /* read and return current port bits (obsolete) */
#define IO_SETBITS   0x2  /* set the bits marked by 1 in the argument */
#define IO_CLRBITS   0x3  /* clear the bits marked by 1 in the argument */

/* the alarm is waited for by select() */

#define IO_HIGHALARM 0x4  /* set alarm on high for bits marked by 1 */
#define IO_LOWALARM  0x5  /* set alarm on low for bits marked by 1 */
#define IO_CLRALARM  0x6  /* clear alarm for bits marked by 1 */

/* LED ioctl */
#define IO_LEDACTIVE_SET 0x7 /* set active led
                              * 0=off, 1=green, 2=red, 3=yellow */

/* GPIO direction ioctl's */
#define IO_READDIR    0x8  /* Read direction 0=input 1=output  (obsolete) */
#define IO_SETINPUT   0x9  /* Set direction for bits set, 0=unchanged 1=input,
                              returns mask with current inputs (obsolete) */
#define IO_SETOUTPUT  0xA  /* Set direction for bits set, 0=unchanged 1=output,
                              returns mask with current outputs (obsolete)*/

/* LED ioctl extended */
#define IO_LED_SETBIT 0xB
#define IO_LED_CLRBIT 0xC

/* SHUTDOWN ioctl */
#define IO_SHUTDOWN   0xD
#define IO_GET_PWR_BT 0xE

/* Bit toggling in driver settings */
#define IO_CFG_WRITE_MODE 0xF
#define IO_CFG_WRITE_MODE_VALUE(msb, data_mask, clk_mask) \
	( (((msb)&1) << 16) | (((data_mask) &0xFF) << 8) | ((clk_mask) & 0xFF) )

#define IO_READ_INBITS   0x10 /* *arg is result of reading the input pins */
#define IO_READ_OUTBITS  0x11 /* *arg is result of reading the output shadow */
#define IO_SETGET_INPUT  0x12 /* bits set in *arg is set to input,
                               * *arg updated with current input pins.
                               */
#define IO_SETGET_OUTPUT 0x13 /* bits set in *arg is set to output,
                               * *arg updated with current output pins.
                               */

/* The following ioctl's are applicable to the PWM channels only */

#define IO_PWM_SET_MODE     0x20

enum io_pwm_mode {
	PWM_OFF = 0,		/* disabled, deallocated */
	PWM_STANDARD = 1,	/* 390 kHz, duty cycle 0..255/256 */
	PWM_FAST = 2,		/* variable freq, w/ 10ns active pulse len */
	PWM_VARFREQ = 3		/* individually configurable high/low periods */
};

struct io_pwm_set_mode {
	enum io_pwm_mode mode;
};

#define IO_PWM_SET_PERIOD   0x21

struct io_pwm_set_period {
	unsigned int lo;		/* 0..8191 */
	unsigned int hi;		/* 0..8191 */
};

#define IO_PWM_SET_DUTY     0x22

struct io_pwm_set_duty {
	int duty;		/* 0..255 */
};

#endif
