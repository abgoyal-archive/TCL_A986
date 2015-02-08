
int connection_init(DBusConnection *conn);
void connection_exit(void);
int connection_register(struct btd_device *device, const char *path,
			bdaddr_t *src, bdaddr_t *dst, uint16_t id);
void connection_unregister(const char *path, uint16_t id);
