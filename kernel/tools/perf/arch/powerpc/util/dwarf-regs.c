

#include <libio.h>
#include <dwarf-regs.h>


struct pt_regs_dwarfnum {
	const char *name;
	unsigned int dwarfnum;
};

#define STR(s) #s
#define REG_DWARFNUM_NAME(r, num) {.name = r, .dwarfnum = num}
#define GPR_DWARFNUM_NAME(num)	\
	{.name = STR(%gpr##num), .dwarfnum = num}
#define REG_DWARFNUM_END {.name = NULL, .dwarfnum = 0}

static const struct pt_regs_dwarfnum regdwarfnum_table[] = {
	GPR_DWARFNUM_NAME(0),
	GPR_DWARFNUM_NAME(1),
	GPR_DWARFNUM_NAME(2),
	GPR_DWARFNUM_NAME(3),
	GPR_DWARFNUM_NAME(4),
	GPR_DWARFNUM_NAME(5),
	GPR_DWARFNUM_NAME(6),
	GPR_DWARFNUM_NAME(7),
	GPR_DWARFNUM_NAME(8),
	GPR_DWARFNUM_NAME(9),
	GPR_DWARFNUM_NAME(10),
	GPR_DWARFNUM_NAME(11),
	GPR_DWARFNUM_NAME(12),
	GPR_DWARFNUM_NAME(13),
	GPR_DWARFNUM_NAME(14),
	GPR_DWARFNUM_NAME(15),
	GPR_DWARFNUM_NAME(16),
	GPR_DWARFNUM_NAME(17),
	GPR_DWARFNUM_NAME(18),
	GPR_DWARFNUM_NAME(19),
	GPR_DWARFNUM_NAME(20),
	GPR_DWARFNUM_NAME(21),
	GPR_DWARFNUM_NAME(22),
	GPR_DWARFNUM_NAME(23),
	GPR_DWARFNUM_NAME(24),
	GPR_DWARFNUM_NAME(25),
	GPR_DWARFNUM_NAME(26),
	GPR_DWARFNUM_NAME(27),
	GPR_DWARFNUM_NAME(28),
	GPR_DWARFNUM_NAME(29),
	GPR_DWARFNUM_NAME(30),
	GPR_DWARFNUM_NAME(31),
	REG_DWARFNUM_NAME("%msr",   66),
	REG_DWARFNUM_NAME("%ctr",   109),
	REG_DWARFNUM_NAME("%link",  108),
	REG_DWARFNUM_NAME("%xer",   101),
	REG_DWARFNUM_NAME("%dar",   119),
	REG_DWARFNUM_NAME("%dsisr", 118),
	REG_DWARFNUM_END,
};

const char *get_arch_regstr(unsigned int n)
{
	const struct pt_regs_dwarfnum *roff;
	for (roff = regdwarfnum_table; roff->name != NULL; roff++)
		if (roff->dwarfnum == n)
			return roff->name;
	return NULL;
}
