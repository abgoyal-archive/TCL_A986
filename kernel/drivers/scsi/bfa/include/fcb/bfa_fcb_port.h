


#ifndef __BFA_FCB_PORT_H__
#define __BFA_FCB_PORT_H__

#include <fcb/bfa_fcb_vport.h>

struct bfad_port_s;


struct bfad_port_s *bfa_fcb_port_new(struct bfad_s *bfad,
			struct bfa_fcs_port_s *port,
			enum bfa_port_role roles, struct bfad_vf_s *vf_drv,
			struct bfad_vport_s *vp_drv);

void bfa_fcb_port_delete(struct bfad_s *bfad, enum bfa_port_role roles,
			struct bfad_vf_s *vf_drv, struct bfad_vport_s *vp_drv);

void bfa_fcb_port_online(struct bfad_s *bfad, enum bfa_port_role roles,
			struct bfad_vf_s *vf_drv, struct bfad_vport_s *vp_drv);

void bfa_fcb_port_offline(struct bfad_s *bfad, enum bfa_port_role roles,
			struct bfad_vf_s *vf_drv, struct bfad_vport_s *vp_drv);


#endif /* __BFA_FCB_PORT_H__ */
