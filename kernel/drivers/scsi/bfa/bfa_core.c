

#include <bfa.h>
#include <defs/bfa_defs_pci.h>
#include <cs/bfa_debug.h>
#include <bfa_iocfc.h>

#define DEF_CFG_NUM_FABRICS         1
#define DEF_CFG_NUM_LPORTS          256
#define DEF_CFG_NUM_CQS             4
#define DEF_CFG_NUM_IOIM_REQS       (BFA_IOIM_MAX)
#define DEF_CFG_NUM_TSKIM_REQS      128
#define DEF_CFG_NUM_FCXP_REQS       64
#define DEF_CFG_NUM_UF_BUFS         64
#define DEF_CFG_NUM_RPORTS          1024
#define DEF_CFG_NUM_ITNIMS          (DEF_CFG_NUM_RPORTS)
#define DEF_CFG_NUM_TINS            256

#define DEF_CFG_NUM_SGPGS           2048
#define DEF_CFG_NUM_REQQ_ELEMS      256
#define DEF_CFG_NUM_RSPQ_ELEMS      64
#define DEF_CFG_NUM_SBOOT_TGTS      16
#define DEF_CFG_NUM_SBOOT_LUNS      16

void
bfa_cfg_get_meminfo(struct bfa_iocfc_cfg_s *cfg, struct bfa_meminfo_s *meminfo)
{
	int             i;
	u32        km_len = 0, dm_len = 0;

	bfa_assert((cfg != NULL) && (meminfo != NULL));

	bfa_os_memset((void *)meminfo, 0, sizeof(struct bfa_meminfo_s));
	meminfo->meminfo[BFA_MEM_TYPE_KVA - 1].mem_type =
		BFA_MEM_TYPE_KVA;
	meminfo->meminfo[BFA_MEM_TYPE_DMA - 1].mem_type =
		BFA_MEM_TYPE_DMA;

	bfa_iocfc_meminfo(cfg, &km_len, &dm_len);

	for (i = 0; hal_mods[i]; i++)
		hal_mods[i]->meminfo(cfg, &km_len, &dm_len);

	dm_len += bfa_port_meminfo();

	meminfo->meminfo[BFA_MEM_TYPE_KVA - 1].mem_len = km_len;
	meminfo->meminfo[BFA_MEM_TYPE_DMA - 1].mem_len = dm_len;
}

static void
bfa_com_port_attach(struct bfa_s *bfa, struct bfa_meminfo_s *mi)
{
	struct bfa_port_s       *port = &bfa->modules.port;
	uint32_t                dm_len;
	uint8_t                 *dm_kva;
	uint64_t                dm_pa;

	dm_len = bfa_port_meminfo();
	dm_kva = bfa_meminfo_dma_virt(mi);
	dm_pa  = bfa_meminfo_dma_phys(mi);

	memset(port, 0, sizeof(struct bfa_port_s));
	bfa_port_attach(port, &bfa->ioc, bfa, bfa->trcmod, bfa->logm);
	bfa_port_mem_claim(port, dm_kva, dm_pa);

	bfa_meminfo_dma_virt(mi) = dm_kva + dm_len;
	bfa_meminfo_dma_phys(mi) = dm_pa + dm_len;
}

void
bfa_attach(struct bfa_s *bfa, void *bfad, struct bfa_iocfc_cfg_s *cfg,
	       struct bfa_meminfo_s *meminfo, struct bfa_pcidev_s *pcidev)
{
	int             i;
	struct bfa_mem_elem_s *melem;

	bfa->fcs = BFA_FALSE;

	bfa_assert((cfg != NULL) && (meminfo != NULL));

	/**
	 * initialize all memory pointers for iterative allocation
	 */
	for (i = 0; i < BFA_MEM_TYPE_MAX; i++) {
		melem = meminfo->meminfo + i;
		melem->kva_curp = melem->kva;
		melem->dma_curp = melem->dma;
	}

	bfa_iocfc_attach(bfa, bfad, cfg, meminfo, pcidev);

	for (i = 0; hal_mods[i]; i++)
		hal_mods[i]->attach(bfa, bfad, cfg, meminfo, pcidev);

	bfa_com_port_attach(bfa, meminfo);
}

void
bfa_detach(struct bfa_s *bfa)
{
	int	i;

	for (i = 0; hal_mods[i]; i++)
		hal_mods[i]->detach(bfa);

	bfa_iocfc_detach(bfa);
}


void
bfa_init_trc(struct bfa_s *bfa, struct bfa_trc_mod_s *trcmod)
{
	bfa->trcmod = trcmod;
}


void
bfa_init_log(struct bfa_s *bfa, struct bfa_log_mod_s *logmod)
{
	bfa->logm = logmod;
}


void
bfa_init_aen(struct bfa_s *bfa, struct bfa_aen_s *aen)
{
	bfa->aen = aen;
}

void
bfa_init_plog(struct bfa_s *bfa, struct bfa_plog_s *plog)
{
	bfa->plog = plog;
}

void
bfa_init(struct bfa_s *bfa)
{
	bfa_iocfc_init(bfa);
}

void
bfa_start(struct bfa_s *bfa)
{
	bfa_iocfc_start(bfa);
}

void
bfa_stop(struct bfa_s *bfa)
{
	bfa_iocfc_stop(bfa);
}

void
bfa_comp_deq(struct bfa_s *bfa, struct list_head *comp_q)
{
	INIT_LIST_HEAD(comp_q);
	list_splice_tail_init(&bfa->comp_q, comp_q);
}

void
bfa_comp_process(struct bfa_s *bfa, struct list_head *comp_q)
{
	struct list_head        *qe;
	struct list_head        *qen;
	struct bfa_cb_qe_s   *hcb_qe;

	list_for_each_safe(qe, qen, comp_q) {
		hcb_qe = (struct bfa_cb_qe_s *) qe;
		hcb_qe->cbfn(hcb_qe->cbarg, BFA_TRUE);
	}
}

void
bfa_comp_free(struct bfa_s *bfa, struct list_head *comp_q)
{
	struct list_head        *qe;
	struct bfa_cb_qe_s   *hcb_qe;

	while (!list_empty(comp_q)) {
		bfa_q_deq(comp_q, &qe);
		hcb_qe = (struct bfa_cb_qe_s *) qe;
		hcb_qe->cbfn(hcb_qe->cbarg, BFA_FALSE);
	}
}

void
bfa_attach_fcs(struct bfa_s *bfa)
{
	bfa->fcs = BFA_TRUE;
}

void
bfa_timer_tick(struct bfa_s *bfa)
{
	bfa_timer_beat(&bfa->timer_mod);
}

#ifndef BFA_BIOS_BUILD
void
bfa_get_pciids(struct bfa_pciid_s **pciids, int *npciids)
{
	static struct bfa_pciid_s __pciids[] = {
		{BFA_PCI_VENDOR_ID_BROCADE, BFA_PCI_DEVICE_ID_FC_8G2P},
		{BFA_PCI_VENDOR_ID_BROCADE, BFA_PCI_DEVICE_ID_FC_8G1P},
		{BFA_PCI_VENDOR_ID_BROCADE, BFA_PCI_DEVICE_ID_CT},
	};

	*npciids = sizeof(__pciids) / sizeof(__pciids[0]);
	*pciids = __pciids;
}

void
bfa_cfg_get_default(struct bfa_iocfc_cfg_s *cfg)
{
	cfg->fwcfg.num_fabrics = DEF_CFG_NUM_FABRICS;
	cfg->fwcfg.num_lports = DEF_CFG_NUM_LPORTS;
	cfg->fwcfg.num_rports = DEF_CFG_NUM_RPORTS;
	cfg->fwcfg.num_ioim_reqs = DEF_CFG_NUM_IOIM_REQS;
	cfg->fwcfg.num_tskim_reqs = DEF_CFG_NUM_TSKIM_REQS;
	cfg->fwcfg.num_fcxp_reqs = DEF_CFG_NUM_FCXP_REQS;
	cfg->fwcfg.num_uf_bufs = DEF_CFG_NUM_UF_BUFS;
	cfg->fwcfg.num_cqs = DEF_CFG_NUM_CQS;

	cfg->drvcfg.num_reqq_elems = DEF_CFG_NUM_REQQ_ELEMS;
	cfg->drvcfg.num_rspq_elems = DEF_CFG_NUM_RSPQ_ELEMS;
	cfg->drvcfg.num_sgpgs = DEF_CFG_NUM_SGPGS;
	cfg->drvcfg.num_sboot_tgts = DEF_CFG_NUM_SBOOT_TGTS;
	cfg->drvcfg.num_sboot_luns = DEF_CFG_NUM_SBOOT_LUNS;
	cfg->drvcfg.path_tov = BFA_FCPIM_PATHTOV_DEF;
	cfg->drvcfg.ioc_recover = BFA_FALSE;
	cfg->drvcfg.delay_comp = BFA_FALSE;

}

void
bfa_cfg_get_min(struct bfa_iocfc_cfg_s *cfg)
{
	bfa_cfg_get_default(cfg);
	cfg->fwcfg.num_ioim_reqs   = BFA_IOIM_MIN;
	cfg->fwcfg.num_tskim_reqs  = BFA_TSKIM_MIN;
	cfg->fwcfg.num_fcxp_reqs   = BFA_FCXP_MIN;
	cfg->fwcfg.num_uf_bufs     = BFA_UF_MIN;
	cfg->fwcfg.num_rports      = BFA_RPORT_MIN;

	cfg->drvcfg.num_sgpgs      = BFA_SGPG_MIN;
	cfg->drvcfg.num_reqq_elems = BFA_REQQ_NELEMS_MIN;
	cfg->drvcfg.num_rspq_elems = BFA_RSPQ_NELEMS_MIN;
	cfg->drvcfg.min_cfg        = BFA_TRUE;
}

void
bfa_get_attr(struct bfa_s *bfa, struct bfa_ioc_attr_s *ioc_attr)
{
	bfa_ioc_get_attr(&bfa->ioc, ioc_attr);
}

bfa_status_t
bfa_debug_fwsave(struct bfa_s *bfa, void *trcdata, int *trclen)
{
	return bfa_ioc_debug_fwsave(&bfa->ioc, trcdata, trclen);
}

void
bfa_debug_fwsave_clear(struct bfa_s *bfa)
{
	bfa_ioc_debug_fwsave_clear(&bfa->ioc);
}

bfa_status_t
bfa_debug_fwtrc(struct bfa_s *bfa, void *trcdata, int *trclen)
{
	return bfa_ioc_debug_fwtrc(&bfa->ioc, trcdata, trclen);
}

void
bfa_chip_reset(struct bfa_s *bfa)
{
	bfa_ioc_ownership_reset(&bfa->ioc);
	bfa_ioc_pll_init(&bfa->ioc);
}
#endif
