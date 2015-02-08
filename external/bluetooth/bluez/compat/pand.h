
int bnep_init(void);
int bnep_cleanup(void);

int bnep_str2svc(char *svc, uint16_t *uuid);
char *bnep_svc2str(uint16_t uuid);

int bnep_show_connections(void);
int bnep_kill_connection(uint8_t *dst);
int bnep_kill_all_connections(void);

int bnep_accept_connection(int sk, uint16_t role, char *dev);
int bnep_create_connection(int sk, uint16_t role, uint16_t svc, char *dev);
