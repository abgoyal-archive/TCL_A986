

#ifndef __ASM_ARM_ARCH_UDC_H
#define __ASM_ARM_ARCH_UDC_H

enum s3c2410_udc_cmd_e {
	S3C2410_UDC_P_ENABLE	= 1,	/* Pull-up enable        */
	S3C2410_UDC_P_DISABLE	= 2,	/* Pull-up disable       */
	S3C2410_UDC_P_RESET	= 3,	/* UDC reset, in case of */
};

struct s3c2410_udc_mach_info {
	void	(*udc_command)(enum s3c2410_udc_cmd_e);
 	void	(*vbus_draw)(unsigned int ma);
	unsigned int vbus_pin;
	unsigned char vbus_pin_inverted;
};

extern void __init s3c24xx_udc_set_platdata(struct s3c2410_udc_mach_info *);

#endif /* __ASM_ARM_ARCH_UDC_H */
