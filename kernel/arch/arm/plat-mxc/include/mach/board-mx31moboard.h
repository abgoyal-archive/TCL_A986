

#ifndef __ASM_ARCH_MXC_BOARD_MX31MOBOARD_H__
#define __ASM_ARCH_MXC_BOARD_MX31MOBOARD_H__

#ifndef __ASSEMBLY__

enum mx31moboard_boards {
	MX31NOBOARD	= 0,
	MX31DEVBOARD	= 1,
	MX31MARXBOT	= 2,
	MX31SMARTBOT	= 3,
	MX31EYEBOT	= 4,
};


extern void mx31moboard_devboard_init(void);
extern void mx31moboard_marxbot_init(void);
extern void mx31moboard_smartbot_init(int board);

#endif

#endif /* __ASM_ARCH_MXC_BOARD_MX31MOBOARD_H__ */
