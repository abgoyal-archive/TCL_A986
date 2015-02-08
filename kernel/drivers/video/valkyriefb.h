

#ifdef CONFIG_MAC
/* Valkyrie registers are word-aligned on m68k */
#define VALKYRIE_REG_PADSIZE	3
#else
#define VALKYRIE_REG_PADSIZE	7
#endif

struct cmap_regs {
	unsigned char addr;
	char pad1[VALKYRIE_REG_PADSIZE];
	unsigned char lut;
};


struct vpreg {			/* padded register */
	unsigned char r;
	char pad[VALKYRIE_REG_PADSIZE];
};


struct valkyrie_regs {
	struct vpreg mode;
	struct vpreg depth;
	struct vpreg status;
	struct vpreg reg3;
	struct vpreg intr;
	struct vpreg reg5;
	struct vpreg intr_enb;
	struct vpreg msense;
};

struct valkyrie_regvals {
	unsigned char mode;
	unsigned char clock_params[3];
	int	pitch[2];		/* bytes/line, indexed by color_mode */
	int	hres;
	int	vres;
};

#ifndef CONFIG_MAC
/* Register values for 1024x768, 75Hz mode (17) */

static struct valkyrie_regvals valkyrie_reg_init_17 = {
    15, 
    { 11, 28, 3 },  /* pixel clock = 79.55MHz for V=74.50Hz */
    { 1024, 0 },
	1024, 768
};

/* Register values for 1024x768, 72Hz mode (15) */
static struct valkyrie_regvals valkyrie_reg_init_15 = {
    15,
    { 12, 29, 3 },  /* pixel clock = 75.52MHz for V=69.71Hz? */
		    /* I interpolated the V=69.71 from the vmode 14 and old 15
		     * numbers. Is this result correct?
		     */
    { 1024, 0 },
	1024, 768
};

/* Register values for 1024x768, 60Hz mode (14) */
static struct valkyrie_regvals valkyrie_reg_init_14 = {
    14,
    { 15, 31, 3 },  /* pixel clock = 64.58MHz for V=59.62Hz */
    { 1024, 0 },
	1024, 768
};
#endif /* !defined CONFIG_MAC */

/* Register values for 832x624, 75Hz mode (13) */
static struct valkyrie_regvals valkyrie_reg_init_13 = {
    9,
    { 23, 42, 3 },  /* pixel clock = 57.07MHz for V=74.27Hz */
    { 832, 0 },
	832, 624
};

/* Register values for 800x600, 72Hz mode (11) */
static struct valkyrie_regvals valkyrie_reg_init_11 = {
    13,
    { 17, 27, 3 },  /* pixel clock = 49.63MHz for V=71.66Hz */
    { 800, 0 },
	800, 600
};

/* Register values for 800x600, 60Hz mode (10) */
static struct valkyrie_regvals valkyrie_reg_init_10 = {
    12,
    { 25, 32, 3 },  /* pixel clock = 40.0015MHz,
                     used to be 20,53,2, pixel clock 41.41MHz for V=59.78Hz */
    { 800, 1600 },
	800, 600
};

/* Register values for 640x480, 67Hz mode (6) */
static struct valkyrie_regvals valkyrie_reg_init_6 = {
    6,
    { 14, 27, 2 },  /* pixel clock = 30.13MHz for V=66.43Hz */
    { 640, 1280 },
	640, 480
};

/* Register values for 640x480, 60Hz mode (5) */
static struct valkyrie_regvals valkyrie_reg_init_5 = {
    11,
    { 23, 37, 2 },  /* pixel clock = 25.14MHz for V=59.85Hz */
    { 640, 1280 },
	640, 480
};

static struct valkyrie_regvals *valkyrie_reg_init[VMODE_MAX] = {
	NULL,
	NULL,
	NULL,
	NULL,
	&valkyrie_reg_init_5,
	&valkyrie_reg_init_6,
	NULL,
	NULL,
	NULL,
	&valkyrie_reg_init_10,
	&valkyrie_reg_init_11,
	NULL,
	&valkyrie_reg_init_13,
#ifndef CONFIG_MAC
	&valkyrie_reg_init_14,
	&valkyrie_reg_init_15,
	NULL,
	&valkyrie_reg_init_17,
#endif
};
