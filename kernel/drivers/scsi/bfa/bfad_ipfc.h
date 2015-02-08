
#ifndef __BFA_DRV_IPFC_H__
#define __BFA_DRV_IPFC_H__


#define IPFC_NAME ""

#define bfad_ipfc_module_init(x) do {} while (0)
#define bfad_ipfc_module_exit(x) do {} while (0)
#define bfad_ipfc_probe(x) do {} while (0)
#define bfad_ipfc_probe_undo(x) do {} while (0)
#define bfad_ipfc_port_config(x, y) BFA_STATUS_OK
#define bfad_ipfc_port_unconfig(x, y) do {} while (0)

#define bfad_ipfc_probe_post(x) do {} while (0)
#define bfad_ipfc_port_new(x, y, z) BFA_STATUS_OK
#define bfad_ipfc_port_delete(x, y) do {} while (0)
#define bfad_ipfc_port_online(x, y) do {} while (0)
#define bfad_ipfc_port_offline(x, y) do {} while (0)

#define bfad_ip_get_attr(x) BFA_STATUS_FAILED
#define bfad_ip_reset_drv_stats(x) BFA_STATUS_FAILED
#define bfad_ip_get_drv_stats(x, y) BFA_STATUS_FAILED
#define bfad_ip_enable_ipfc(x, y, z) BFA_STATUS_FAILED


#endif
