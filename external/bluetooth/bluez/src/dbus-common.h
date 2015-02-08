
#define MAX_PATH_LENGTH 64

void hcid_dbus_exit(void);
int hcid_dbus_init(void);
void hcid_dbus_unregister(void);

void dict_append_entry(DBusMessageIter *dict,
			const char *key, int type, void *val);

void dict_append_array(DBusMessageIter *dict, const char *key, int type,
			void *val, int n_elements);

dbus_bool_t emit_property_changed(DBusConnection *conn,
					const char *path,
					const char *interface,
					const char *name,
					int type, void *value);

dbus_bool_t emit_array_property_changed(DBusConnection *conn,
					const char *path,
					const char *interface,
					const char *name,
					int type, void *value);
