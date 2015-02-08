

#ifndef _ATH5K_DEBUG_H
#define _ATH5K_DEBUG_H

struct ath5k_softc;
struct ath5k_hw;
struct sk_buff;
struct ath5k_buf;

struct ath5k_dbg_info {
	unsigned int		level;		/* debug level */
	/* debugfs entries */
	struct dentry		*debugfs_phydir;
	struct dentry		*debugfs_debug;
	struct dentry		*debugfs_registers;
	struct dentry		*debugfs_beacon;
	struct dentry		*debugfs_reset;
	struct dentry		*debugfs_antenna;
	struct dentry		*debugfs_frameerrors;
	struct dentry		*debugfs_ani;
};

enum ath5k_debug_level {
	ATH5K_DEBUG_RESET	= 0x00000001,
	ATH5K_DEBUG_INTR	= 0x00000002,
	ATH5K_DEBUG_MODE	= 0x00000004,
	ATH5K_DEBUG_XMIT	= 0x00000008,
	ATH5K_DEBUG_BEACON	= 0x00000010,
	ATH5K_DEBUG_CALIBRATE	= 0x00000020,
	ATH5K_DEBUG_TXPOWER	= 0x00000040,
	ATH5K_DEBUG_LED		= 0x00000080,
	ATH5K_DEBUG_DUMP_RX	= 0x00000100,
	ATH5K_DEBUG_DUMP_TX	= 0x00000200,
	ATH5K_DEBUG_DUMPBANDS	= 0x00000400,
	ATH5K_DEBUG_TRACE	= 0x00001000,
	ATH5K_DEBUG_ANI		= 0x00002000,
	ATH5K_DEBUG_ANY		= 0xffffffff
};

#ifdef CONFIG_ATH5K_DEBUG

#define ATH5K_TRACE(_sc) do { \
	if (unlikely((_sc)->debug.level & ATH5K_DEBUG_TRACE)) \
		printk(KERN_DEBUG "ath5k trace %s:%d\n", __func__, __LINE__); \
	} while (0)

#define ATH5K_DBG(_sc, _m, _fmt, ...) do { \
	if (unlikely((_sc)->debug.level & (_m) && net_ratelimit())) \
		ATH5K_PRINTK(_sc, KERN_DEBUG, "(%s:%d): " _fmt, \
			__func__, __LINE__, ##__VA_ARGS__); \
	} while (0)

#define ATH5K_DBG_UNLIMIT(_sc, _m, _fmt, ...) do { \
	if (unlikely((_sc)->debug.level & (_m))) \
		ATH5K_PRINTK(_sc, KERN_DEBUG, "(%s:%d): " _fmt, \
			__func__, __LINE__, ##__VA_ARGS__); \
	} while (0)

void
ath5k_debug_init(void);

void
ath5k_debug_init_device(struct ath5k_softc *sc);

void
ath5k_debug_finish(void);

void
ath5k_debug_finish_device(struct ath5k_softc *sc);

void
ath5k_debug_printrxbuffs(struct ath5k_softc *sc, struct ath5k_hw *ah);

void
ath5k_debug_dump_bands(struct ath5k_softc *sc);

void
ath5k_debug_dump_skb(struct ath5k_softc *sc,
			struct sk_buff *skb, const char *prefix, int tx);

void
ath5k_debug_printtxbuf(struct ath5k_softc *sc, struct ath5k_buf *bf);

#else /* no debugging */

#include <linux/compiler.h>

#define ATH5K_TRACE(_sc) typecheck(struct ath5k_softc *, (_sc))

static inline void __attribute__ ((format (printf, 3, 4)))
ATH5K_DBG(struct ath5k_softc *sc, unsigned int m, const char *fmt, ...) {}

static inline void __attribute__ ((format (printf, 3, 4)))
ATH5K_DBG_UNLIMIT(struct ath5k_softc *sc, unsigned int m, const char *fmt, ...)
{}

static inline void
ath5k_debug_init(void) {}

static inline void
ath5k_debug_init_device(struct ath5k_softc *sc) {}

static inline void
ath5k_debug_finish(void) {}

static inline void
ath5k_debug_finish_device(struct ath5k_softc *sc) {}

static inline void
ath5k_debug_printrxbuffs(struct ath5k_softc *sc, struct ath5k_hw *ah) {}

static inline void
ath5k_debug_dump_bands(struct ath5k_softc *sc) {}

static inline void
ath5k_debug_dump_skb(struct ath5k_softc *sc,
			struct sk_buff *skb, const char *prefix, int tx) {}

static inline void
ath5k_debug_printtxbuf(struct ath5k_softc *sc, struct ath5k_buf *bf) {}

#endif /* ifdef CONFIG_ATH5K_DEBUG */

#endif /* ifndef _ATH5K_DEBUG_H */
