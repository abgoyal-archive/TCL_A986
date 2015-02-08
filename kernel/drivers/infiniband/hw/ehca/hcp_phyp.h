

#ifndef __HCP_PHYP_H__
#define __HCP_PHYP_H__


struct h_galpa {
	u64 fw_handle;
	/* for pSeries this is a 64bit memory address where
	   I/O memory is mapped into CPU address space (kv) */
};

struct h_galpas {
	u32 pid;		/*PID of userspace galpa checking */
	struct h_galpa user;	/* user space accessible resource,
				   set to 0 if unused */
	struct h_galpa kernel;	/* kernel space accessible resource,
				   set to 0 if unused */
};

static inline u64 hipz_galpa_load(struct h_galpa galpa, u32 offset)
{
	u64 addr = galpa.fw_handle + offset;
	return *(volatile u64 __force *)addr;
}

static inline void hipz_galpa_store(struct h_galpa galpa, u32 offset, u64 value)
{
	u64 addr = galpa.fw_handle + offset;
	*(volatile u64 __force *)addr = value;
}

int hcp_galpas_ctor(struct h_galpas *galpas, int is_user,
		    u64 paddr_kernel, u64 paddr_user);

int hcp_galpas_dtor(struct h_galpas *galpas);

int hcall_map_page(u64 physaddr, u64 * mapaddr);

int hcall_unmap_page(u64 mapaddr);

#endif
