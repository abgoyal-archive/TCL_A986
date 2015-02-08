
#ifndef __ASM_ARCH_HARDWARE_H
#define __ASM_ARCH_HARDWARE_H

#define XBUS_SIZE		0x00100000
#define XBUS_BASE		0xff800000

#define ARMCSR_SIZE		0x00100000
#define ARMCSR_BASE		0xfe000000

#define WFLUSH_SIZE		0x00100000
#define WFLUSH_BASE		0xfd000000

#define PCIIACK_SIZE		0x00100000
#define PCIIACK_BASE		0xfc000000

#define PCICFG1_SIZE		0x01000000
#define PCICFG1_BASE		0xfb000000

#define PCICFG0_SIZE		0x01000000
#define PCICFG0_BASE		0xfa000000

#define PCIMEM_SIZE		0x01000000
#define PCIMEM_BASE		0xf0000000

#define XBUS_LEDS		((volatile unsigned char *)(XBUS_BASE + 0x12000))
#define XBUS_LED_AMBER		(1 << 0)
#define XBUS_LED_GREEN		(1 << 1)
#define XBUS_LED_RED		(1 << 2)
#define XBUS_LED_TOGGLE		(1 << 8)

#define XBUS_SWITCH		((volatile unsigned char *)(XBUS_BASE + 0x12000))
#define XBUS_SWITCH_SWITCH	((*XBUS_SWITCH) & 15)
#define XBUS_SWITCH_J17_13	((*XBUS_SWITCH) & (1 << 4))
#define XBUS_SWITCH_J17_11	((*XBUS_SWITCH) & (1 << 5))
#define XBUS_SWITCH_J17_9	((*XBUS_SWITCH) & (1 << 6))

#define UNCACHEABLE_ADDR	(ARMCSR_BASE + 0x108)


/* PIC irq control */
#define PIC_LO			0x20
#define PIC_MASK_LO		0x21
#define PIC_HI			0xA0
#define PIC_MASK_HI		0xA1

/* GPIO pins */
#define GPIO_CCLK		0x800
#define GPIO_DSCLK		0x400
#define GPIO_E2CLK		0x200
#define GPIO_IOLOAD		0x100
#define GPIO_RED_LED		0x080
#define GPIO_WDTIMER		0x040
#define GPIO_DATA		0x020
#define GPIO_IOCLK		0x010
#define GPIO_DONE		0x008
#define GPIO_FAN		0x004
#define GPIO_GREEN_LED		0x002
#define GPIO_RESET		0x001

/* CPLD pins */
#define CPLD_DS_ENABLE		8
#define CPLD_7111_DISABLE	4
#define CPLD_UNMUTE		2
#define CPLD_FLASH_WR_ENABLE	1

#ifndef __ASSEMBLY__
extern spinlock_t nw_gpio_lock;
extern void nw_gpio_modify_op(unsigned int mask, unsigned int set);
extern void nw_gpio_modify_io(unsigned int mask, unsigned int in);
extern unsigned int nw_gpio_read(void);
extern void nw_cpld_modify(unsigned int mask, unsigned int set);
#endif

#define pcibios_assign_all_busses()	1

#define PCIBIOS_MIN_IO		0x1000
#define PCIBIOS_MIN_MEM 	0x81000000

#endif
