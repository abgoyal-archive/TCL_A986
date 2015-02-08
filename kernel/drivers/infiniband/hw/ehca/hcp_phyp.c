

#include "ehca_classes.h"
#include "hipz_hw.h"

int hcall_map_page(u64 physaddr, u64 *mapaddr)
{
	*mapaddr = (u64)(ioremap(physaddr, EHCA_PAGESIZE));
	return 0;
}

int hcall_unmap_page(u64 mapaddr)
{
	iounmap((volatile void __iomem *) mapaddr);
	return 0;
}

int hcp_galpas_ctor(struct h_galpas *galpas, int is_user,
		    u64 paddr_kernel, u64 paddr_user)
{
	if (!is_user) {
		int ret = hcall_map_page(paddr_kernel, &galpas->kernel.fw_handle);
		if (ret)
			return ret;
	} else
		galpas->kernel.fw_handle = 0;

	galpas->user.fw_handle = paddr_user;

	return 0;
}

int hcp_galpas_dtor(struct h_galpas *galpas)
{
	if (galpas->kernel.fw_handle) {
		int ret = hcall_unmap_page(galpas->kernel.fw_handle);
		if (ret)
			return ret;
	}

	galpas->user.fw_handle = galpas->kernel.fw_handle = 0;

	return 0;
}
