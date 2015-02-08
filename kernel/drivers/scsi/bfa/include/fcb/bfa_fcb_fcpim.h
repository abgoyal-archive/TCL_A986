


#ifndef __BFAD_FCB_FCPIM_H__
#define __BFAD_FCB_FCPIM_H__

struct bfad_itnim_s;


void bfa_fcb_itnim_alloc(struct bfad_s *bfad, struct bfa_fcs_itnim_s **itnim,
				    struct bfad_itnim_s **itnim_drv);

void            bfa_fcb_itnim_free(struct bfad_s *bfad,
				   struct bfad_itnim_s *itnim_drv);

void            bfa_fcb_itnim_online(struct bfad_itnim_s *itnim_drv);

void            bfa_fcb_itnim_offline(struct bfad_itnim_s *itnim_drv);

void            bfa_fcb_itnim_tov(struct bfad_itnim_s *itnim_drv);

#endif /* __BFAD_FCB_FCPIM_H__ */
