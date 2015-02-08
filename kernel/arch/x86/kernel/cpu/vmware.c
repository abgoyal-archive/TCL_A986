

#include <linux/dmi.h>
#include <linux/module.h>
#include <linux/jiffies.h>
#include <asm/div64.h>
#include <asm/x86_init.h>
#include <asm/hypervisor.h>

#define CPUID_VMWARE_INFO_LEAF	0x40000000
#define VMWARE_HYPERVISOR_MAGIC	0x564D5868
#define VMWARE_HYPERVISOR_PORT	0x5658

#define VMWARE_PORT_CMD_GETVERSION	10
#define VMWARE_PORT_CMD_GETHZ		45

#define VMWARE_PORT(cmd, eax, ebx, ecx, edx)				\
	__asm__("inl (%%dx)" :						\
			"=a"(eax), "=c"(ecx), "=d"(edx), "=b"(ebx) :	\
			"0"(VMWARE_HYPERVISOR_MAGIC),			\
			"1"(VMWARE_PORT_CMD_##cmd),			\
			"2"(VMWARE_HYPERVISOR_PORT), "3"(UINT_MAX) :	\
			"memory");

static inline int __vmware_platform(void)
{
	uint32_t eax, ebx, ecx, edx;
	VMWARE_PORT(GETVERSION, eax, ebx, ecx, edx);
	return eax != (uint32_t)-1 && ebx == VMWARE_HYPERVISOR_MAGIC;
}

static unsigned long vmware_get_tsc_khz(void)
{
	uint64_t tsc_hz, lpj;
	uint32_t eax, ebx, ecx, edx;

	VMWARE_PORT(GETHZ, eax, ebx, ecx, edx);

	tsc_hz = eax | (((uint64_t)ebx) << 32);
	do_div(tsc_hz, 1000);
	BUG_ON(tsc_hz >> 32);
	printk(KERN_INFO "TSC freq read from hypervisor : %lu.%03lu MHz\n",
			 (unsigned long) tsc_hz / 1000,
			 (unsigned long) tsc_hz % 1000);

	if (!preset_lpj) {
		lpj = ((u64)tsc_hz * 1000);
		do_div(lpj, HZ);
		preset_lpj = lpj;
	}

	return tsc_hz;
}

static void __init vmware_platform_setup(void)
{
	uint32_t eax, ebx, ecx, edx;

	VMWARE_PORT(GETHZ, eax, ebx, ecx, edx);

	if (ebx != UINT_MAX)
		x86_platform.calibrate_tsc = vmware_get_tsc_khz;
	else
		printk(KERN_WARNING
		       "Failed to get TSC freq from the hypervisor\n");
}

static bool __init vmware_platform(void)
{
	if (cpu_has_hypervisor) {
		unsigned int eax;
		unsigned int hyper_vendor_id[3];

		cpuid(CPUID_VMWARE_INFO_LEAF, &eax, &hyper_vendor_id[0],
		      &hyper_vendor_id[1], &hyper_vendor_id[2]);
		if (!memcmp(hyper_vendor_id, "VMwareVMware", 12))
			return true;
	} else if (dmi_available && dmi_name_in_serial("VMware") &&
		   __vmware_platform())
		return true;

	return false;
}

static void __cpuinit vmware_set_cpu_features(struct cpuinfo_x86 *c)
{
	set_cpu_cap(c, X86_FEATURE_CONSTANT_TSC);
	set_cpu_cap(c, X86_FEATURE_TSC_RELIABLE);
}

const __refconst struct hypervisor_x86 x86_hyper_vmware = {
	.name			= "VMware",
	.detect			= vmware_platform,
	.set_cpu_features	= vmware_set_cpu_features,
	.init_platform		= vmware_platform_setup,
};
EXPORT_SYMBOL(x86_hyper_vmware);
