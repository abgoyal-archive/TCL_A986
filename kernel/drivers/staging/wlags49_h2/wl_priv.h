

#ifndef __WL_PRIV_H__
#define __WL_PRIV_H__




#ifdef WIRELESS_EXT


int wvlan_set_netname( struct net_device *,  struct iw_request_info *, union iwreq_data *, char *extra );

int wvlan_get_netname( struct net_device *,  struct iw_request_info *, union iwreq_data *, char *extra );

int wvlan_set_station_nickname( struct net_device *,  struct iw_request_info *, union iwreq_data *, char *extra );

int wvlan_get_station_nickname( struct net_device *,  struct iw_request_info *, union iwreq_data *, char *extra );

int wvlan_set_porttype( struct net_device *,  struct iw_request_info *, union iwreq_data *, char *extra );

int wvlan_get_porttype( struct net_device *,  struct iw_request_info *, union iwreq_data *, char *extra );


#endif  // WIRELESS_EXT




#ifdef USE_UIL

int wvlan_uil( struct uilreq *urq, struct wl_private *lp );

// int wvlan_uil_connect( struct uilreq *urq, struct wl_private *lp );
// int wvlan_uil_disconnect( struct uilreq *urq, struct wl_private *lp );
// int wvlan_uil_action( struct uilreq *urq, struct wl_private *lp );
// int wvlan_uil_block( struct uilreq *urq, struct wl_private *lp );
// int wvlan_uil_unblock( struct uilreq *urq, struct wl_private *lp );
// int wvlan_uil_send_diag_msg( struct uilreq *urq, struct wl_private *lp );
// int wvlan_uil_put_info( struct uilreq *urq, struct wl_private *lp );
// int wvlan_uil_get_info( struct uilreq *urq, struct wl_private *lp );

//int cfg_driver_info( struct uilreq *urq, struct wl_private *lp );
//int cfg_driver_identity( struct uilreq *urq, struct wl_private *lp );

#endif  // USE_UIL


#ifdef USE_RTS

int wvlan_rts( struct rtsreq *rrq, __u32 io_base );
int wvlan_rts_read( __u16 reg, __u16 *val, __u32 io_base );
int wvlan_rts_write( __u16 reg, __u16 val, __u32 io_base );
int wvlan_rts_batch_read( struct rtsreq *rrq, __u32 io_base );
int wvlan_rts_batch_write( struct rtsreq *rrq, __u32 io_base );

#endif  // USE_RTS


#endif  // __WL_PRIV_H__
