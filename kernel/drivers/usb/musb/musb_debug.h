

#ifndef __MUSB_LINUX_DEBUG_H__
#define __MUSB_LINUX_DEBUG_H__

#define yprintk(facility, format, args...) \
	do { printk(facility "%s %d: " format , \
	__func__, __LINE__ , ## args); } while (0)
#define WARNING(fmt, args...) yprintk(KERN_WARNING, fmt, ## args)
#define INFO(fmt, args...) yprintk(KERN_INFO, fmt, ## args)
#define ERR(fmt, args...) yprintk(KERN_ERR, fmt, ## args)

#define xprintk(level, facility, format, args...) do { \
	if (_dbg_level(level)) { \
		printk(facility "%s %d: " format , \
				__func__, __LINE__ , ## args); \
	} } while (0)

extern unsigned musb_debug;

static inline int _dbg_level(unsigned l)
{
	return musb_debug >= l;
}

#define DBG(level, fmt, args...) xprintk(level, KERN_DEBUG, fmt, ## args)

extern const char *otg_state_string(struct musb *);

#ifdef CONFIG_DEBUG_FS
extern int musb_init_debugfs(struct musb *musb);
extern void musb_exit_debugfs(struct musb *musb);
#else
static inline int musb_init_debugfs(struct musb *musb)
{
	return 0;
}
static inline void musb_exit_debugfs(struct musb *musb)
{
}
#endif

#endif				/*  __MUSB_LINUX_DEBUG_H__ */
