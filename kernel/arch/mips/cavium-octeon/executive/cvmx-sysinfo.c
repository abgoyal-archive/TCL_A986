

#include <linux/module.h>

#include <asm/octeon/cvmx.h>
#include <asm/octeon/cvmx-spinlock.h>
#include <asm/octeon/cvmx-sysinfo.h>

static struct {
	struct cvmx_sysinfo sysinfo;	   /* system information */
	cvmx_spinlock_t lock;	   /* mutex spinlock */

} state = {
	.lock = CVMX_SPINLOCK_UNLOCKED_INITIALIZER
};


uint64_t linux_mem32_min;
uint64_t linux_mem32_max;
uint64_t linux_mem32_wired;
uint64_t linux_mem32_offset;

struct cvmx_sysinfo *cvmx_sysinfo_get(void)
{
	return &(state.sysinfo);
}
EXPORT_SYMBOL(cvmx_sysinfo_get);

int cvmx_sysinfo_minimal_initialize(void *phy_mem_desc_ptr,
				    uint16_t board_type,
				    uint8_t board_rev_major,
				    uint8_t board_rev_minor,
				    uint32_t cpu_clock_hz)
{

	/* The sysinfo structure was already initialized */
	if (state.sysinfo.board_type)
		return 0;

	memset(&(state.sysinfo), 0x0, sizeof(state.sysinfo));
	state.sysinfo.phy_mem_desc_ptr = phy_mem_desc_ptr;
	state.sysinfo.board_type = board_type;
	state.sysinfo.board_rev_major = board_rev_major;
	state.sysinfo.board_rev_minor = board_rev_minor;
	state.sysinfo.cpu_clock_hz = cpu_clock_hz;

	return 1;
}
