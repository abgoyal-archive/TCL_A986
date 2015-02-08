

#ifndef LINUX_HWRANDOM_H_
#define LINUX_HWRANDOM_H_

#include <linux/types.h>
#include <linux/list.h>

struct hwrng {
	const char *name;
	int (*init)(struct hwrng *rng);
	void (*cleanup)(struct hwrng *rng);
	int (*data_present)(struct hwrng *rng, int wait);
	int (*data_read)(struct hwrng *rng, u32 *data);
	int (*read)(struct hwrng *rng, void *data, size_t max, bool wait);
	unsigned long priv;

	/* internal. */
	struct list_head list;
};

/** Register a new Hardware Random Number Generator driver. */
extern int hwrng_register(struct hwrng *rng);
/** Unregister a Hardware Random Number Generator driver. */
extern void hwrng_unregister(struct hwrng *rng);

#endif /* LINUX_HWRANDOM_H_ */
