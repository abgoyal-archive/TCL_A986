

#ifndef CTRL_IFACE_H
#define CTRL_IFACE_H

#ifdef CONFIG_CTRL_IFACE

/* Shared functions from ctrl_iface.c; to be called by ctrl_iface backends */

char * wpa_supplicant_ctrl_iface_process(struct wpa_supplicant *wpa_s,
					 char *buf, size_t *resp_len);

char * wpa_supplicant_global_ctrl_iface_process(struct wpa_global *global,
						char *buf, size_t *resp_len);


/* Functions that each ctrl_iface backend must implement */

struct ctrl_iface_priv *
wpa_supplicant_ctrl_iface_init(struct wpa_supplicant *wpa_s);

void wpa_supplicant_ctrl_iface_deinit(struct ctrl_iface_priv *priv);

void wpa_supplicant_ctrl_iface_wait(struct ctrl_iface_priv *priv);

struct ctrl_iface_global_priv *
wpa_supplicant_global_ctrl_iface_init(struct wpa_global *global);

void wpa_supplicant_global_ctrl_iface_deinit(
	struct ctrl_iface_global_priv *priv);

#else /* CONFIG_CTRL_IFACE */

static inline struct ctrl_iface_priv *
wpa_supplicant_ctrl_iface_init(struct wpa_supplicant *wpa_s)
{
	return (void *) -1;
}

static inline void
wpa_supplicant_ctrl_iface_deinit(struct ctrl_iface_priv *priv)
{
}

static inline void
wpa_supplicant_ctrl_iface_send(struct ctrl_iface_priv *priv, int level,
			       char *buf, size_t len)
{
}

static inline void
wpa_supplicant_ctrl_iface_wait(struct ctrl_iface_priv *priv)
{
}

static inline struct ctrl_iface_global_priv *
wpa_supplicant_global_ctrl_iface_init(struct wpa_global *global)
{
	return (void *) 1;
}

static inline void
wpa_supplicant_global_ctrl_iface_deinit(struct ctrl_iface_global_priv *priv)
{
}

#endif /* CONFIG_CTRL_IFACE */

#endif /* CTRL_IFACE_H */
