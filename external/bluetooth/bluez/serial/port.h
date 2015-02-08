
void port_release_all(void);

int port_register(DBusConnection *conn, const char *path, bdaddr_t *src,
		  bdaddr_t *dst, const char *name, uint8_t channel);

int port_unregister(const char *path);
