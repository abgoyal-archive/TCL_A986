
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/bitmap.h>
#include <linux/io.h>
#include <asm/ilsel.h>

#define ILSEL_BASE	0xb8100004
#define ILSEL_LEVELS	15

static unsigned long ilsel_level_map;

static inline unsigned int ilsel_offset(unsigned int bit)
{
	return ILSEL_LEVELS - bit - 1;
}

static inline unsigned long mk_ilsel_addr(unsigned int bit)
{
	return ILSEL_BASE + ((ilsel_offset(bit) >> 1) & ~0x1);
}

static inline unsigned int mk_ilsel_shift(unsigned int bit)
{
	return (ilsel_offset(bit) & 0x3) << 2;
}

static void __ilsel_enable(ilsel_source_t set, unsigned int bit)
{
	unsigned int tmp, shift;
	unsigned long addr;

	addr = mk_ilsel_addr(bit);
	shift = mk_ilsel_shift(bit);

	pr_debug("%s: bit#%d: addr - 0x%08lx (shift %d, set %d)\n",
		 __func__, bit, addr, shift, set);

	tmp = __raw_readw(addr);
	tmp &= ~(0xf << shift);
	tmp |= set << shift;
	__raw_writew(tmp, addr);
}

int ilsel_enable(ilsel_source_t set)
{
	unsigned int bit;

	/* Aliased sources must use ilsel_enable_fixed() */
	BUG_ON(set > ILSEL_KEY);

	do {
		bit = find_first_zero_bit(&ilsel_level_map, ILSEL_LEVELS);
	} while (test_and_set_bit(bit, &ilsel_level_map));

	__ilsel_enable(set, bit);

	return bit;
}
EXPORT_SYMBOL_GPL(ilsel_enable);

int ilsel_enable_fixed(ilsel_source_t set, unsigned int level)
{
	unsigned int bit = ilsel_offset(level - 1);

	if (test_and_set_bit(bit, &ilsel_level_map))
		return -EBUSY;

	__ilsel_enable(set, bit);

	return bit;
}
EXPORT_SYMBOL_GPL(ilsel_enable_fixed);

void ilsel_disable(unsigned int irq)
{
	unsigned long addr;
	unsigned int tmp;

	addr = mk_ilsel_addr(irq);

	tmp = __raw_readw(addr);
	tmp &= ~(0xf << mk_ilsel_shift(irq));
	__raw_writew(tmp, addr);

	clear_bit(irq, &ilsel_level_map);
}
EXPORT_SYMBOL_GPL(ilsel_disable);
