
#include <mach/csp/chipcHw_def.h>

#define CLK_TYPE_PRIMARY         1	/* primary clock must NOT have a parent */
#define CLK_TYPE_PLL1            2	/* PPL1 */
#define CLK_TYPE_PLL2            4	/* PPL2 */
#define CLK_TYPE_PROGRAMMABLE    8	/* programmable clock rate */
#define CLK_TYPE_BYPASSABLE      16	/* parent can be changed */

#define CLK_MODE_XTAL            1	/* clock source is from crystal */

struct clk {
	const char *name;	/* clock name */
	unsigned int type;	/* clock type */
	unsigned int mode;	/* current mode */
	volatile int use_bypass;	/* indicate if it's in bypass mode */
	chipcHw_CLOCK_e csp_id;	/* clock ID for CSP CHIPC */
	unsigned long rate_hz;	/* clock rate in Hz */
	unsigned int use_cnt;	/* usage count */
	struct clk *parent;	/* parent clock */
};
