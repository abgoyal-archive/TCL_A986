
#ifndef WPA_CTRL_H
#define WPA_CTRL_H

#ifdef  __cplusplus
extern "C" {
#endif

/* wpa_supplicant control interface - fixed message prefixes */

/** Interactive request for identity/password/pin */
#define WPA_CTRL_REQ "CTRL-REQ-"

/** Response to identity/password/pin request */
#define WPA_CTRL_RSP "CTRL-RSP-"

/* Event messages with fixed prefix */
/** Authentication completed successfully and data connection enabled */
#define WPA_EVENT_CONNECTED "CTRL-EVENT-CONNECTED "
/** Disconnected, data connection is not available */
#define WPA_EVENT_DISCONNECTED "CTRL-EVENT-DISCONNECTED "
/** wpa_supplicant is exiting */
#define WPA_EVENT_TERMINATING "CTRL-EVENT-TERMINATING "
/** Password change was completed successfully */
#define WPA_EVENT_PASSWORD_CHANGED "CTRL-EVENT-PASSWORD-CHANGED "
/** EAP-Request/Notification received */
#define WPA_EVENT_EAP_NOTIFICATION "CTRL-EVENT-EAP-NOTIFICATION "
/** EAP authentication started (EAP-Request/Identity received) */
#define WPA_EVENT_EAP_STARTED "CTRL-EVENT-EAP-STARTED "
/** EAP method selected */
#define WPA_EVENT_EAP_METHOD "CTRL-EVENT-EAP-METHOD "
/** EAP authentication completed successfully */
#define WPA_EVENT_EAP_SUCCESS "CTRL-EVENT-EAP-SUCCESS "
/** EAP authentication failed (EAP-Failure received) */
#define WPA_EVENT_EAP_FAILURE "CTRL-EVENT-EAP-FAILURE "
/** Scan results are ready */
#define WPA_EVENT_SCAN_RESULTS "CTRL-EVENT-SCAN-RESULTS "
/** wpa_supplicant state change */
#define WPA_EVENT_STATE_CHANGE "CTRL-EVENT-STATE-CHANGE "
/** AP to STA speed */
#define WPA_EVENT_LINK_SPEED "CTRL-EVENT-LINK-SPEED "
/** Driver state change */
#define WPA_EVENT_DRIVER_STATE "CTRL-EVENT-DRIVER-STATE "

/* wpa_supplicant/hostapd control interface access */

struct wpa_ctrl * wpa_ctrl_open(const char *ctrl_path);


void wpa_ctrl_close(struct wpa_ctrl *ctrl);


int wpa_ctrl_request(struct wpa_ctrl *ctrl, const char *cmd, size_t cmd_len,
		     char *reply, size_t *reply_len,
		     void (*msg_cb)(char *msg, size_t len));


int wpa_ctrl_attach(struct wpa_ctrl *ctrl);


int wpa_ctrl_detach(struct wpa_ctrl *ctrl);


int wpa_ctrl_recv(struct wpa_ctrl *ctrl, char *reply, size_t *reply_len);


int wpa_ctrl_pending(struct wpa_ctrl *ctrl);


int wpa_ctrl_get_fd(struct wpa_ctrl *ctrl);

#ifdef ANDROID
void wpa_ctrl_cleanup(void);
#endif  /* ANDROID */

#ifdef CONFIG_CTRL_IFACE_UDP
#define WPA_CTRL_IFACE_PORT 9877
#define WPA_GLOBAL_CTRL_IFACE_PORT 9878
#endif /* CONFIG_CTRL_IFACE_UDP */


#ifdef  __cplusplus
}
#endif

#endif /* WPA_CTRL_H */
