

#ifndef __MACH_STMP378X_H
#define __MACH_STMP378X_H

void stmp378x_map_io(void);
void stmp378x_init_irq(void);

extern struct platform_device stmp378x_pxp, stmp378x_i2c;
#endif /* __MACH_STMP378X_COMMON_H */
