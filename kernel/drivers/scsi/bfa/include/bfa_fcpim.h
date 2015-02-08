

#ifndef __BFA_FCPIM_H__
#define __BFA_FCPIM_H__

#include <bfa.h>
#include <bfa_svc.h>
#include <bfi/bfi_fcpim.h>
#include <defs/bfa_defs_fcpim.h>

struct bfa_itnim_s;
struct bfa_ioim_s;
struct bfa_tskim_s;
struct bfad_ioim_s;
struct bfad_tskim_s;

void		bfa_fcpim_path_tov_set(struct bfa_s *bfa, u16 path_tov);
u16	bfa_fcpim_path_tov_get(struct bfa_s *bfa);
void		bfa_fcpim_qdepth_set(struct bfa_s *bfa, u16 q_depth);
u16	bfa_fcpim_qdepth_get(struct bfa_s *bfa);
bfa_status_t bfa_fcpim_get_modstats(struct bfa_s *bfa,
			struct bfa_fcpim_stats_s *modstats);
bfa_status_t bfa_fcpim_clr_modstats(struct bfa_s *bfa);

struct bfa_itnim_s *bfa_itnim_create(struct bfa_s *bfa,
					struct bfa_rport_s *rport, void *itnim);
void		bfa_itnim_delete(struct bfa_itnim_s *itnim);
void		bfa_itnim_online(struct bfa_itnim_s *itnim,
				 bfa_boolean_t seq_rec);
void		bfa_itnim_offline(struct bfa_itnim_s *itnim);
void		bfa_itnim_get_stats(struct bfa_itnim_s *itnim,
			struct bfa_itnim_hal_stats_s *stats);
void		bfa_itnim_clear_stats(struct bfa_itnim_s *itnim);


void            bfa_cb_itnim_online(void *itnim);

void            bfa_cb_itnim_offline(void *itnim);
void            bfa_cb_itnim_tov_begin(void *itnim);
void            bfa_cb_itnim_tov(void *itnim);

void            bfa_cb_itnim_sler(void *itnim);

struct bfa_ioim_s	*bfa_ioim_alloc(struct bfa_s *bfa,
					struct bfad_ioim_s *dio,
					struct bfa_itnim_s *itnim,
					u16 nsgles);

void		bfa_ioim_free(struct bfa_ioim_s *ioim);
void		bfa_ioim_start(struct bfa_ioim_s *ioim);
void		bfa_ioim_abort(struct bfa_ioim_s *ioim);
void 		bfa_ioim_delayed_comp(struct bfa_ioim_s *ioim,
				      bfa_boolean_t iotov);


void            bfa_cb_ioim_done(void *bfad, struct bfad_ioim_s *dio,
				  enum bfi_ioim_status io_status,
				  u8 scsi_status, int sns_len,
				  u8 *sns_info, s32 residue);

void            bfa_cb_ioim_good_comp(void *bfad, struct bfad_ioim_s *dio);

void            bfa_cb_ioim_abort(void *bfad, struct bfad_ioim_s *dio);
void		bfa_cb_ioim_resfree(void *hcb_bfad);

void 			bfa_cb_ioim_resfree(void *hcb_bfad);

struct bfa_tskim_s	*bfa_tskim_alloc(struct bfa_s *bfa,
					struct bfad_tskim_s *dtsk);
void		bfa_tskim_free(struct bfa_tskim_s *tskim);
void		bfa_tskim_start(struct bfa_tskim_s *tskim,
				struct bfa_itnim_s *itnim, lun_t lun,
				enum fcp_tm_cmnd tm, u8 t_secs);
void		bfa_cb_tskim_done(void *bfad, struct bfad_tskim_s *dtsk,
				  enum bfi_tskim_status tsk_status);

#endif /* __BFA_FCPIM_H__ */

