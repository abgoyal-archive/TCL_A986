


#include <asm/firmware.h>
#include <asm/prom.h>
#include <asm/udbg.h>

#include "pseries.h"

typedef struct {
    unsigned long val;
    char * name;
} firmware_feature_t;

static __initdata firmware_feature_t
firmware_features_table[FIRMWARE_MAX_FEATURES] = {
	{FW_FEATURE_PFT,		"hcall-pft"},
	{FW_FEATURE_TCE,		"hcall-tce"},
	{FW_FEATURE_SPRG0,		"hcall-sprg0"},
	{FW_FEATURE_DABR,		"hcall-dabr"},
	{FW_FEATURE_COPY,		"hcall-copy"},
	{FW_FEATURE_ASR,		"hcall-asr"},
	{FW_FEATURE_DEBUG,		"hcall-debug"},
	{FW_FEATURE_PERF,		"hcall-perf"},
	{FW_FEATURE_DUMP,		"hcall-dump"},
	{FW_FEATURE_INTERRUPT,		"hcall-interrupt"},
	{FW_FEATURE_MIGRATE,		"hcall-migrate"},
	{FW_FEATURE_PERFMON,		"hcall-perfmon"},
	{FW_FEATURE_CRQ,		"hcall-crq"},
	{FW_FEATURE_VIO,		"hcall-vio"},
	{FW_FEATURE_RDMA,		"hcall-rdma"},
	{FW_FEATURE_LLAN,		"hcall-lLAN"},
	{FW_FEATURE_BULK_REMOVE,	"hcall-bulk"},
	{FW_FEATURE_XDABR,		"hcall-xdabr"},
	{FW_FEATURE_MULTITCE,		"hcall-multi-tce"},
	{FW_FEATURE_SPLPAR,		"hcall-splpar"},
};

void __init fw_feature_init(const char *hypertas, unsigned long len)
{
	const char *s;
	int i;

	pr_debug(" -> fw_feature_init()\n");

	for (s = hypertas; s < hypertas + len; s += strlen(s) + 1) {
		for (i = 0; i < FIRMWARE_MAX_FEATURES; i++) {
			/* check value against table of strings */
			if (!firmware_features_table[i].name ||
			    strcmp(firmware_features_table[i].name, s))
				continue;

			/* we have a match */
			powerpc_firmware_features |=
				firmware_features_table[i].val;
			break;
		}
	}

	pr_debug(" <- fw_feature_init()\n");
}
