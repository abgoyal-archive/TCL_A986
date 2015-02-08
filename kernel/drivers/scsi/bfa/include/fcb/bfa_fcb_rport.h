


#ifndef __BFA_FCB_RPORT_H__
#define __BFA_FCB_RPORT_H__



struct bfad_rport_s;


void bfa_fcb_rport_add(struct bfad_rport_s *rport_drv);

void bfa_fcb_rport_remove(struct bfad_rport_s *rport_drv);

bfa_status_t bfa_fcb_rport_alloc(struct bfad_s *bfad,
			struct bfa_fcs_rport_s **rport,
			struct bfad_rport_s **rport_drv);

void bfa_fcb_rport_free(struct bfad_s *bfad, struct bfad_rport_s **rport_drv);



#endif /* __BFA_FCB_RPORT_H__ */
