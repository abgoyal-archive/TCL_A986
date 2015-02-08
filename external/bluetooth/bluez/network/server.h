
int server_init(DBusConnection *conn, gboolean secure);
void server_exit(void);
int server_register(struct btd_adapter *adapter);
int server_unregister(struct btd_adapter *adapter);

int server_find_data(const char *path, const char *pattern);
