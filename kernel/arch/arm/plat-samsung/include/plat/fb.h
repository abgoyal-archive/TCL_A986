

#ifndef __PLAT_S3C_FB_H
#define __PLAT_S3C_FB_H __FILE__

#define S3C_FB_MAX_WIN	(5)

struct s3c_fb_pd_win {
	struct fb_videomode	win_mode;

	unsigned short		default_bpp;
	unsigned short		max_bpp;
	unsigned short		virtual_x;
	unsigned short		virtual_y;
};

struct s3c_fb_platdata {
	void	(*setup_gpio)(void);

	struct s3c_fb_pd_win	*win[S3C_FB_MAX_WIN];

	u32			 vidcon0;
	u32			 vidcon1;
};

extern void s3c_fb_set_platdata(struct s3c_fb_platdata *pd);

extern void s3c64xx_fb_gpio_setup_24bpp(void);

extern void s5pc100_fb_gpio_setup_24bpp(void);

extern void s5pv210_fb_gpio_setup_24bpp(void);

#endif /* __PLAT_S3C_FB_H */
