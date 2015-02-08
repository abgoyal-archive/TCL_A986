

#include <linux/mm.h>
#include <asm/page.h>

/* References to section boundaries */
extern const void __nosave_begin, __nosave_end;


int pfn_is_nosave(unsigned long pfn)
{
	unsigned long nosave_begin_pfn = __pa(&__nosave_begin) >> PAGE_SHIFT;
	unsigned long nosave_end_pfn = PAGE_ALIGN(__pa(&__nosave_end)) >> PAGE_SHIFT;
	return (pfn >= nosave_begin_pfn) && (pfn < nosave_end_pfn);
}
