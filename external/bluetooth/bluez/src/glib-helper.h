
int set_nonblocking(int fd);

typedef void (*bt_callback_t) (sdp_list_t *recs, int err, gpointer user_data);
typedef void (*bt_destroy_t) (gpointer user_data);
typedef void (*bt_hci_result_t) (uint8_t status, gpointer user_data);

int bt_discover_services(const bdaddr_t *src, const bdaddr_t *dst,
		bt_callback_t cb, void *user_data, bt_destroy_t destroy);

int bt_search_service(const bdaddr_t *src, const bdaddr_t *dst,
			uuid_t *uuid, bt_callback_t cb, void *user_data,
			bt_destroy_t destroy);
int bt_cancel_discovery(const bdaddr_t *src, const bdaddr_t *dst);

gchar *bt_uuid2string(uuid_t *uuid);
uint16_t bt_name2class(const char *string);
char *bt_name2string(const char *string);
int bt_string2uuid(uuid_t *uuid, const char *string);
gchar *bt_list2string(GSList *list);
GSList *bt_string2list(const gchar *str);

int bt_acl_encrypt(const bdaddr_t *src, const bdaddr_t *dst,
			bt_hci_result_t cb, gpointer user_data);
