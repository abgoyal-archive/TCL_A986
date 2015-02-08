


#include <plat/regs-fb.h>

#define S3C_FB_MAX_WIN (5)  /* number of hardware windows available. */
#define VIDCON1_FSTATUS_EVEN	(1 << 15)

/* Video timing controls */
#define VIDTCON0				(0x10)
#define VIDTCON1				(0x14)
#define VIDTCON2				(0x18)

/* Window position controls */

#define WINCON(_win)				(0x20 + ((_win) * 4))

/* OSD1 and OSD4 do not have register D */

#define VIDOSD_A(_win)				(0x40 + ((_win) * 16))
#define VIDOSD_B(_win)				(0x44 + ((_win) * 16))
#define VIDOSD_C(_win)				(0x48 + ((_win) * 16))
#define VIDOSD_D(_win)				(0x4C + ((_win) * 16))


#define VIDINTCON0				(0x130)

#define WxKEYCONy(_win, _con)			((0x140 + ((_win) * 8)) + ((_con) * 4))

/* WINCONx */

#define WINCONx_CSCWIDTH_MASK			(0x3 << 26)
#define WINCONx_CSCWIDTH_SHIFT			(26)
#define WINCONx_CSCWIDTH_WIDE			(0x0 << 26)
#define WINCONx_CSCWIDTH_NARROW			(0x3 << 26)

#define WINCONx_ENLOCAL				(1 << 22)
#define WINCONx_BUFSTATUS			(1 << 21)
#define WINCONx_BUFSEL				(1 << 20)
#define WINCONx_BUFAUTOEN			(1 << 19)
#define WINCONx_YCbCr				(1 << 13)

#define WINCON1_LOCALSEL_CAMIF			(1 << 23)

#define WINCON2_LOCALSEL_CAMIF			(1 << 23)
#define WINCON2_BLD_PIX				(1 << 6)

#define WINCON2_ALPHA_SEL			(1 << 1)
#define WINCON2_BPPMODE_MASK			(0xf << 2)
#define WINCON2_BPPMODE_SHIFT			(2)
#define WINCON2_BPPMODE_1BPP			(0x0 << 2)
#define WINCON2_BPPMODE_2BPP			(0x1 << 2)
#define WINCON2_BPPMODE_4BPP			(0x2 << 2)
#define WINCON2_BPPMODE_8BPP_1232		(0x4 << 2)
#define WINCON2_BPPMODE_16BPP_565		(0x5 << 2)
#define WINCON2_BPPMODE_16BPP_A1555		(0x6 << 2)
#define WINCON2_BPPMODE_16BPP_I1555		(0x7 << 2)
#define WINCON2_BPPMODE_18BPP_666		(0x8 << 2)
#define WINCON2_BPPMODE_18BPP_A1665		(0x9 << 2)
#define WINCON2_BPPMODE_19BPP_A1666		(0xa << 2)
#define WINCON2_BPPMODE_24BPP_888		(0xb << 2)
#define WINCON2_BPPMODE_24BPP_A1887		(0xc << 2)
#define WINCON2_BPPMODE_25BPP_A1888		(0xd << 2)
#define WINCON2_BPPMODE_28BPP_A4888		(0xd << 2)

#define WINCON3_BLD_PIX				(1 << 6)

#define WINCON3_ALPHA_SEL			(1 << 1)
#define WINCON3_BPPMODE_MASK			(0xf << 2)
#define WINCON3_BPPMODE_SHIFT			(2)
#define WINCON3_BPPMODE_1BPP			(0x0 << 2)
#define WINCON3_BPPMODE_2BPP			(0x1 << 2)
#define WINCON3_BPPMODE_4BPP			(0x2 << 2)
#define WINCON3_BPPMODE_16BPP_565		(0x5 << 2)
#define WINCON3_BPPMODE_16BPP_A1555		(0x6 << 2)
#define WINCON3_BPPMODE_16BPP_I1555		(0x7 << 2)
#define WINCON3_BPPMODE_18BPP_666		(0x8 << 2)
#define WINCON3_BPPMODE_18BPP_A1665		(0x9 << 2)
#define WINCON3_BPPMODE_19BPP_A1666		(0xa << 2)
#define WINCON3_BPPMODE_24BPP_888		(0xb << 2)
#define WINCON3_BPPMODE_24BPP_A1887		(0xc << 2)
#define WINCON3_BPPMODE_25BPP_A1888		(0xd << 2)
#define WINCON3_BPPMODE_28BPP_A4888		(0xd << 2)

#define VIDINTCON0_FIFIOSEL_WINDOW2		(0x10 << 5)
#define VIDINTCON0_FIFIOSEL_WINDOW3		(0x20 << 5)
#define VIDINTCON0_FIFIOSEL_WINDOW4		(0x40 << 5)

#define DITHMODE				(0x170)
#define WINxMAP(_win)				(0x180 + ((_win) * 4))


#define DITHMODE_R_POS_MASK			(0x3 << 5)
#define DITHMODE_R_POS_SHIFT			(5)
#define DITHMODE_R_POS_8BIT			(0x0 << 5)
#define DITHMODE_R_POS_6BIT			(0x1 << 5)
#define DITHMODE_R_POS_5BIT			(0x2 << 5)

#define DITHMODE_G_POS_MASK			(0x3 << 3)
#define DITHMODE_G_POS_SHIFT			(3)
#define DITHMODE_G_POS_8BIT			(0x0 << 3)
#define DITHMODE_G_POS_6BIT			(0x1 << 3)
#define DITHMODE_G_POS_5BIT			(0x2 << 3)

#define DITHMODE_B_POS_MASK			(0x3 << 1)
#define DITHMODE_B_POS_SHIFT			(1)
#define DITHMODE_B_POS_8BIT			(0x0 << 1)
#define DITHMODE_B_POS_6BIT			(0x1 << 1)
#define DITHMODE_B_POS_5BIT			(0x2 << 1)

#define DITHMODE_DITH_EN			(1 << 0)

#define WPALCON					(0x1A0)

/* Palette control */
#define WPALCON_W4PAL_16BPP_A555		(1 << 8)
#define WPALCON_W3PAL_16BPP_A555		(1 << 7)
#define WPALCON_W2PAL_16BPP_A555		(1 << 6)



/* return true if window _win has OSD register D */
#define s3c_fb_has_osd_d(_win) ((_win) != 4 && (_win) != 0)

static inline unsigned int s3c_fb_win_pal_size(unsigned int win)
{
	if (win < 2)
		return 256;
	if (win < 4)
		return 16;
	if (win == 4)
		return 4;

	BUG();	/* shouldn't get here */
}

static inline int s3c_fb_validate_win_bpp(unsigned int win, unsigned int bpp)
{
	/* all windows can do 1/2 bpp */

	if ((bpp == 25 || bpp == 19) && win == 0)
		return 0;	/* win 0 does not have 19 or 25bpp modes */

	if (bpp == 4 && win == 4)
		return 0;

	if (bpp == 8 && (win >= 3))
		return 0;	/* win 3/4 cannot do 8bpp in any mode */

	return 1;
}

static inline int s3c_fb_pal_is16(unsigned int window)
{
	return window > 1;
}

struct s3c_fb_palette {
	struct fb_bitfield	r;
	struct fb_bitfield	g;
	struct fb_bitfield	b;
	struct fb_bitfield	a;
};

static inline void s3c_fb_init_palette(unsigned int window,
				       struct s3c_fb_palette *palette)
{
	if (window < 2) {
		/* Windows 0/1 are 8/8/8 or A/8/8/8 */
		palette->r.offset = 16;
		palette->r.length = 8;
		palette->g.offset = 8;
		palette->g.length = 8;
		palette->b.offset = 0;
		palette->b.length = 8;
	} else {
		/* currently we assume RGB 5/6/5 */
		palette->r.offset = 11;
		palette->r.length = 5;
		palette->g.offset = 5;
		palette->g.length = 6;
		palette->b.offset = 0;
		palette->b.length = 5;
	}
}

