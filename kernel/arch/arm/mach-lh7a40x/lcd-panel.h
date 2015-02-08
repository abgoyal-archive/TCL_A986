

#if !defined (__LCD_PANEL_H__)
#    define   __LCD_PANEL_H__

#if defined (MACH_LPD79520)\
 || defined (MACH_LPD79524)\
 || defined (MACH_LPD7A400)\
 || defined (MACH_LPD7A404)
# define USE_RGB555
#endif

struct clcd_panel_extra {
	unsigned int hrmode;
	unsigned int clsen;
	unsigned int spsen;
	unsigned int pcdel;
	unsigned int revdel;
	unsigned int lpdel;
	unsigned int spldel;
	unsigned int pc2del;
};

#define NS_TO_CLOCK(ns,c)	((((ns)*((c)/1000) + (1000000 - 1))/1000000))
#define CLOCK_TO_DIV(e,c)	(((c) + (e) - 1)/(e))

#if defined CONFIG_FB_ARMCLCD_SHARP_LQ035Q7DB02_HRTFT

	/* Logic Product Development LCD 3.5" QVGA HRTFT -10 */
	/* Sharp PN LQ035Q7DB02 w/HRTFT controller chip */

#define PIX_CLOCK_TARGET	(6800000)
#define PIX_CLOCK_DIVIDER	CLOCK_TO_DIV (PIX_CLOCK_TARGET, HCLK)
#define PIX_CLOCK		(HCLK/PIX_CLOCK_DIVIDER)

static struct clcd_panel lcd_panel = {
	.mode	= {
		.name		= "3.5in QVGA (LQ035Q7DB02)",
		.xres		= 240,
		.yres		= 320,
		.pixclock	= PIX_CLOCK,
		.left_margin	= 16,
		.right_margin	= 21,
		.upper_margin	= 8,			// line/8/8/8
		.lower_margin	= 5,
		.hsync_len	= 61,
		.vsync_len	= NS_TO_CLOCK (60, PIX_CLOCK),
		.vmode		= FB_VMODE_NONINTERLACED,
	},
	.width		= -1,
	.height		= -1,
	.tim2		= TIM2_IPC | (PIX_CLOCK_DIVIDER - 2),
	.cntl		= CNTL_LCDTFT | CNTL_WATERMARK,
	.bpp		= 16,
};

#define HAS_LCD_PANEL_EXTRA

static struct clcd_panel_extra lcd_panel_extra = {
	.hrmode = 1,
	.clsen = 1,
	.spsen = 1,
	.pcdel = 8,
	.revdel = 7,
	.lpdel = 13,
	.spldel = 77,
	.pc2del = 208,
};

#endif

#if defined CONFIG_FB_ARMCLCD_SHARP_LQ057Q3DC02

	/* Logic Product Development LCD 5.7" QVGA -10 */
	/* Sharp PN LQ057Q3DC02 */
	/* QVGA mode, V/Q=LOW */


/* The full horizontal cycle (Th) is clock/360/400/450. */
/* The full vertical   cycle (Tv) is line/251/262/280. */

#define PIX_CLOCK_TARGET	(6300000) /* -/6.3/7 MHz */
#define PIX_CLOCK_DIVIDER	CLOCK_TO_DIV (PIX_CLOCK_TARGET, HCLK)
#define PIX_CLOCK		(HCLK/PIX_CLOCK_DIVIDER)

static struct clcd_panel lcd_panel = {
	.mode	= {
		.name		= "5.7in QVGA (LQ057Q3DC02)",
		.xres		= 320,
		.yres		= 240,
		.pixclock	= PIX_CLOCK,
		.left_margin	= 11,
		.right_margin	= 400-11-320-2,
		.upper_margin	= 7,			// line/7/7/7
		.lower_margin	= 262-7-240-2,
		.hsync_len	= 2,			// clk/2/96/200
		.vsync_len	= 2,			// line/2/-/34
		.vmode		= FB_VMODE_NONINTERLACED,
	},
	.width		= -1,
	.height		= -1,
	.tim2		= TIM2_IHS | TIM2_IVS
			| (PIX_CLOCK_DIVIDER - 2),
	.cntl		= CNTL_LCDTFT | CNTL_WATERMARK,
	.bpp		= 16,
};

#endif

#if defined CONFIG_FB_ARMCLCD_SHARP_LQ64D343

	/* Logic Product Development LCD 6.4" VGA -10 */
	/* Sharp PN LQ64D343 */

/* The full horizontal cycle (Th) is clock/750/800/900. */
/* The full vertical   cycle (Tv) is line/515/525/560. */

#define PIX_CLOCK_TARGET	(28330000)
#define PIX_CLOCK_DIVIDER	CLOCK_TO_DIV (PIX_CLOCK_TARGET, HCLK)
#define PIX_CLOCK		(HCLK/PIX_CLOCK_DIVIDER)

static struct clcd_panel lcd_panel = {
	.mode	= {
		.name		= "6.4in QVGA (LQ64D343)",
		.xres		= 640,
		.yres		= 480,
		.pixclock	= PIX_CLOCK,
		.left_margin	= 32,
		.right_margin	= 800-32-640-96,
		.upper_margin	= 32,			// line/34/34/34
		.lower_margin	= 540-32-480-2,
		.hsync_len	= 96,			// clk/2/96/200
		.vsync_len	= 2,			// line/2/-/34
		.vmode		= FB_VMODE_NONINTERLACED,
	},
	.width		= -1,
	.height		= -1,
	.tim2		= TIM2_IHS | TIM2_IVS
			| (PIX_CLOCK_DIVIDER - 2),
	.cntl		= CNTL_LCDTFT | CNTL_WATERMARK,
	.bpp		= 16,
};

#endif

#if defined CONFIG_FB_ARMCLCD_SHARP_LQ10D368

	/* Logic Product Development LCD 10.4" VGA -10 */
	/* Sharp PN LQ10D368 */

#define PIX_CLOCK_TARGET	(28330000)
#define PIX_CLOCK_DIVIDER	CLOCK_TO_DIV (PIX_CLOCK_TARGET, HCLK)
#define PIX_CLOCK		(HCLK/PIX_CLOCK_DIVIDER)

static struct clcd_panel lcd_panel = {
	.mode	= {
		.name		= "10.4in VGA (LQ10D368)",
		.xres		= 640,
		.yres		= 480,
		.pixclock	= PIX_CLOCK,
		.left_margin	= 21,
		.right_margin	= 15,
		.upper_margin	= 34,
		.lower_margin	= 5,
		.hsync_len	= 96,
		.vsync_len	= 16,
		.vmode		= FB_VMODE_NONINTERLACED,
	},
	.width		= -1,
	.height		= -1,
	.tim2		= TIM2_IHS | TIM2_IVS
			| (PIX_CLOCK_DIVIDER - 2),
	.cntl		= CNTL_LCDTFT | CNTL_WATERMARK,
	.bpp		= 16,
};

#endif

#if defined CONFIG_FB_ARMCLCD_SHARP_LQ121S1DG41

	/* Logic Product Development LCD 12.1" SVGA -10 */
	/* Sharp PN LQ121S1DG41, was LQ121S1DG31 */



/* The full horizontal cycle (Th) is clock/832/1056/1395. */

#define PIX_CLOCK_TARGET	(20000000)
#define PIX_CLOCK_DIVIDER	CLOCK_TO_DIV (PIX_CLOCK_TARGET, HCLK)
#define PIX_CLOCK		(HCLK/PIX_CLOCK_DIVIDER)

static struct clcd_panel lcd_panel = {
	.mode	= {
		.name		= "12.1in SVGA (LQ121S1DG41)",
		.xres		= 800,
		.yres		= 600,
		.pixclock	= PIX_CLOCK,
		.left_margin	= 89,		// ns/5/-/(1/PIX_CLOCK)-10
		.right_margin	= 1056-800-89-128,
		.upper_margin	= 23,		// line/23/23/23
		.lower_margin	= 44,
		.hsync_len	= 128,		// clk/2/128/200
		.vsync_len	= 4,		// line/2/4/6
		.vmode		= FB_VMODE_NONINTERLACED,
	},
	.width		= -1,
	.height		= -1,
	.tim2		= TIM2_IHS | TIM2_IVS
			| (PIX_CLOCK_DIVIDER - 2),
	.cntl		= CNTL_LCDTFT | CNTL_WATERMARK,
	.bpp		= 16,
};

#endif

#if defined CONFIG_FB_ARMCLCD_HITACHI

	/* Hitachi*/
	/* Submitted by Michele Da Rold <michele.darold@ecsproject.com> */

#define PIX_CLOCK_TARGET	(49000000)
#define PIX_CLOCK_DIVIDER	CLOCK_TO_DIV (PIX_CLOCK_TARGET, HCLK)
#define PIX_CLOCK		(HCLK/PIX_CLOCK_DIVIDER)

static struct clcd_panel lcd_panel = {
	.mode	= {
		.name		= "Hitachi 800x480",
		.xres		= 800,
		.yres		= 480,
		.pixclock	= PIX_CLOCK,
		.left_margin	= 88,
		.right_margin	= 40,
		.upper_margin	= 32,
		.lower_margin	= 11,
		.hsync_len	= 128,
		.vsync_len	= 2,
		.vmode		= FB_VMODE_NONINTERLACED,
	},
	.width		= -1,
	.height		= -1,
	.tim2		= TIM2_IPC | TIM2_IHS | TIM2_IVS
			| (PIX_CLOCK_DIVIDER - 2),
	.cntl		= CNTL_LCDTFT | CNTL_WATERMARK,
	.bpp		= 16,
};

#endif


#if defined CONFIG_FB_ARMCLCD_AUO_A070VW01_WIDE

	/* AU Optotronics  A070VW01 7.0 Wide Screen color Display*/
	/* Submitted by Michele Da Rold <michele.darold@ecsproject.com> */

#define PIX_CLOCK_TARGET	(10000000)
#define PIX_CLOCK_DIVIDER	CLOCK_TO_DIV (PIX_CLOCK_TARGET, HCLK)
#define PIX_CLOCK		(HCLK/PIX_CLOCK_DIVIDER)

static struct clcd_panel lcd_panel = {
	.mode	= {
		.name		= "7.0in Wide (A070VW01)",
		.xres		= 480,
		.yres		= 234,
		.pixclock	= PIX_CLOCK,
		.left_margin	= 30,
		.right_margin	= 25,
		.upper_margin	= 14,
		.lower_margin	= 12,
		.hsync_len	= 100,
		.vsync_len	= 1,
		.vmode		= FB_VMODE_NONINTERLACED,
	},
	.width		= -1,
	.height		= -1,
	.tim2		= TIM2_IPC | TIM2_IHS | TIM2_IVS
			| (PIX_CLOCK_DIVIDER - 2),
	.cntl		= CNTL_LCDTFT | CNTL_WATERMARK,
	.bpp		= 16,
};

#endif

#undef NS_TO_CLOCK
#undef CLOCK_TO_DIV

#endif  /* __LCD_PANEL_H__ */
