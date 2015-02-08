

#ifndef __BFA_DEFS_DRIVER_H__
#define __BFA_DEFS_DRIVER_H__

struct bfa_driver_stats_s {
	u16    tm_io_abort;
    u16    tm_io_abort_comp;
    u16    tm_lun_reset;
    u16    tm_lun_reset_comp;
    u16    tm_target_reset;
    u16    tm_bus_reset;
    u16    ioc_restart;        /*  IOC restart count */
    u16    io_pending;         /*  outstanding io count per-IOC */
    u64    control_req;
    u64    input_req;
    u64    output_req;
    u64    input_words;
    u64    output_words;
};


#endif /* __BFA_DEFS_DRIVER_H__ */
