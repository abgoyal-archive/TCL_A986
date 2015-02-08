
/* When all services should trust a remote device */
#define GLOBAL_TRUST "[all]"


#define MODE_OFF		0x00
#define MODE_CONNECTABLE	0x01
#define MODE_DISCOVERABLE	0x02
#define MODE_LIMITED		0x03
#define MODE_UNKNOWN		0xff

#define HCID_DEFAULT_DISCOVERABLE_TIMEOUT 180 /* 3 minutes */

/* Timeout for hci_send_req (milliseconds) */
#define HCI_REQ_TIMEOUT		5000

struct main_opts {
	char		host_name[40];
	unsigned long	flags;
	char		*name;
	uint32_t	class;
	uint16_t	pageto;
	uint32_t	discovto;
	uint32_t	pairto;
	uint16_t	link_mode;
	uint16_t	link_policy;
	gboolean	remember_powered;
	gboolean	reverse_sdp;
	gboolean	name_resolv;
	gboolean	debug_keys;

	uint8_t		scan;
	uint8_t		mode;
	uint8_t		discov_interval;
	char		deviceid[15]; /* FIXME: */

	int		sock;
};

enum {
	HCID_SET_NAME,
	HCID_SET_CLASS,
	HCID_SET_PAGETO,
	HCID_SET_DISCOVTO,
};

extern struct main_opts main_opts;

char *expand_name(char *dst, int size, char *str, int dev_id);

void hci_req_queue_remove(int dev_id, bdaddr_t *dba);

void start_security_manager(int hdev);
void stop_security_manager(int hdev);

void btd_start_exit_timer(void);
void btd_stop_exit_timer(void);

void set_pin_length(bdaddr_t *sba, int length);

gboolean plugin_init(GKeyFile *config);
void plugin_cleanup(void);

void rfkill_init(void);
void rfkill_exit(void);

void __probe_servers(const char *adapter);
void __remove_servers(const char *adapter);
