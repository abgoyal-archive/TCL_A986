
#define DUN_CONFIG_DIR	"/etc/bluetooth/dun"

#define DUN_DEFAULT_CHANNEL	1

#define DUN_MAX_PPP_OPTS	40

int dun_init(void);
int dun_cleanup(void);

int dun_show_connections(void);
int dun_kill_connection(uint8_t *dst);
int dun_kill_all_connections(void);

int dun_open_connection(int sk, char *pppd, char **pppd_opts, int wait);

int ms_dun(int fd, int server, int timeo);
