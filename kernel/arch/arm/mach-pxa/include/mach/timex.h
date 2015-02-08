


#if defined(CONFIG_PXA25x)
/* PXA250/210 timer base */
#define CLOCK_TICK_RATE 3686400
#elif defined(CONFIG_PXA27x)
/* PXA27x timer base */
#ifdef CONFIG_MACH_MAINSTONE
#define CLOCK_TICK_RATE 3249600
#else
#define CLOCK_TICK_RATE 3250000
#endif
#else
#define CLOCK_TICK_RATE 3250000
#endif
