
#include <bluetooth/bluetooth.h>
#include <dbus/dbus.h>

#define MANAGER_INTERFACE "org.bluez.Manager"

dbus_bool_t manager_init(DBusConnection *conn, const char *path);
void manager_cleanup(DBusConnection *conn, const char *path);

const char *manager_get_base_path(void);
struct btd_adapter *manager_find_adapter(const bdaddr_t *sba);
struct btd_adapter *manager_find_adapter_by_address(const char *address);
struct btd_adapter *manager_find_adapter_by_path(const char *path);
struct btd_adapter *manager_find_adapter_by_id(int id);
GSList *manager_get_adapters(void);
int manager_register_adapter(int id, gboolean devup);
int manager_unregister_adapter(int id);
int manager_start_adapter(int id);
int manager_stop_adapter(int id);
void manager_add_adapter(const char *path);
int manager_get_default_adapter();
void manager_set_default_adapter(int id);
void manager_update_svc(struct btd_adapter *adapter, uint8_t svc);
void btd_manager_set_offline(gboolean offline);
