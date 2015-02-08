
struct enabled_interfaces {
	gboolean hfp;
	gboolean headset;
	gboolean gateway;
	gboolean sink;
	gboolean source;
	gboolean control;
};

int audio_manager_init(DBusConnection *conn, GKeyFile *config,
							gboolean *enable_sco);
void audio_manager_exit(void);

gboolean server_is_enabled(bdaddr_t *src, uint16_t svc);

struct audio_device *manager_find_device(const char *path,
					const bdaddr_t *src,
					const bdaddr_t *dst,
					const char *interface,
					gboolean connected);

struct audio_device *manager_get_device(const bdaddr_t *src,
					const bdaddr_t *dst,
					gboolean create);

gboolean manager_allow_headset_connection(struct audio_device *device);
