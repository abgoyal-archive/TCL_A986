


#ifndef EHCA_TOOLS_H
#define EHCA_TOOLS_H

#include <linux/kernel.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/idr.h>
#include <linux/kthread.h>
#include <linux/mm.h>
#include <linux/mman.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/vmalloc.h>
#include <linux/notifier.h>
#include <linux/cpu.h>
#include <linux/device.h>

#include <asm/atomic.h>
#include <asm/abs_addr.h>
#include <asm/ibmebus.h>
#include <asm/io.h>
#include <asm/pgtable.h>
#include <asm/hvcall.h>

extern int ehca_debug_level;

#define ehca_dbg(ib_dev, format, arg...) \
	do { \
		if (unlikely(ehca_debug_level)) \
			dev_printk(KERN_DEBUG, (ib_dev)->dma_device, \
				   "PU%04x EHCA_DBG:%s " format "\n", \
				   raw_smp_processor_id(), __func__, \
				   ## arg); \
	} while (0)

#define ehca_info(ib_dev, format, arg...) \
	dev_info((ib_dev)->dma_device, "PU%04x EHCA_INFO:%s " format "\n", \
		 raw_smp_processor_id(), __func__, ## arg)

#define ehca_warn(ib_dev, format, arg...) \
	dev_warn((ib_dev)->dma_device, "PU%04x EHCA_WARN:%s " format "\n", \
		 raw_smp_processor_id(), __func__, ## arg)

#define ehca_err(ib_dev, format, arg...) \
	dev_err((ib_dev)->dma_device, "PU%04x EHCA_ERR:%s " format "\n", \
		raw_smp_processor_id(), __func__, ## arg)

/* use this one only if no ib_dev available */
#define ehca_gen_dbg(format, arg...) \
	do { \
		if (unlikely(ehca_debug_level)) \
			printk(KERN_DEBUG "PU%04x EHCA_DBG:%s " format "\n", \
			       raw_smp_processor_id(), __func__, ## arg); \
	} while (0)

#define ehca_gen_warn(format, arg...) \
	printk(KERN_INFO "PU%04x EHCA_WARN:%s " format "\n", \
	       raw_smp_processor_id(), __func__, ## arg)

#define ehca_gen_err(format, arg...) \
	printk(KERN_ERR "PU%04x EHCA_ERR:%s " format "\n", \
	       raw_smp_processor_id(), __func__, ## arg)

#define ehca_dmp(adr, len, format, args...) \
	do { \
		unsigned int x; \
		unsigned int l = (unsigned int)(len); \
		unsigned char *deb = (unsigned char *)(adr); \
		for (x = 0; x < l; x += 16) { \
			printk(KERN_INFO "EHCA_DMP:%s " format \
			       " adr=%p ofs=%04x %016llx %016llx\n", \
			       __func__, ##args, deb, x, \
			       *((u64 *)&deb[0]), *((u64 *)&deb[8])); \
			deb += 16; \
		} \
	} while (0)

/* define a bitmask, little endian version */
#define EHCA_BMASK(pos, length) (((pos) << 16) + (length))

/* define a bitmask, the ibm way... */
#define EHCA_BMASK_IBM(from, to) (((63 - to) << 16) + ((to) - (from) + 1))

/* internal function, don't use */
#define EHCA_BMASK_SHIFTPOS(mask) (((mask) >> 16) & 0xffff)

/* internal function, don't use */
#define EHCA_BMASK_MASK(mask) (~0ULL >> ((64 - (mask)) & 0xffff))

#define EHCA_BMASK_SET(mask, value) \
	((EHCA_BMASK_MASK(mask) & ((u64)(value))) << EHCA_BMASK_SHIFTPOS(mask))

#define EHCA_BMASK_GET(mask, value) \
	(EHCA_BMASK_MASK(mask) & (((u64)(value)) >> EHCA_BMASK_SHIFTPOS(mask)))

/* Converts ehca to ib return code */
int ehca2ib_return_code(u64 ehca_rc);

#endif /* EHCA_TOOLS_H */
