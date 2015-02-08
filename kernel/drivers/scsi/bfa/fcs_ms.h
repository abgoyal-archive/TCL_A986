

#ifndef __FCS_MS_H__
#define __FCS_MS_H__

/* MS FCS routines */
void bfa_fcs_port_ms_init(struct bfa_fcs_port_s *port);
void bfa_fcs_port_ms_offline(struct bfa_fcs_port_s *port);
void bfa_fcs_port_ms_online(struct bfa_fcs_port_s *port);
void bfa_fcs_port_ms_fabric_rscn(struct bfa_fcs_port_s *port);

/* FDMI FCS routines */
void bfa_fcs_port_fdmi_init(struct bfa_fcs_port_ms_s *ms);
void bfa_fcs_port_fdmi_offline(struct bfa_fcs_port_ms_s *ms);
void bfa_fcs_port_fdmi_online(struct bfa_fcs_port_ms_s *ms);

#endif
