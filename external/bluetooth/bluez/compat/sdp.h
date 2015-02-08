
#define LANACCESS	0
#define MROUTER		1
#define ACTIVESYNC	2
#define DIALUP		3

int get_stored_device_info(const bdaddr_t *src, const bdaddr_t *dst, struct hidp_connadd_req *req);
int get_sdp_device_info(const bdaddr_t *src, const bdaddr_t *dst, struct hidp_connadd_req *req);
int get_alternate_device_info(const bdaddr_t *src, const bdaddr_t *dst, uint16_t *uuid, uint8_t *channel, char *name, size_t len);

int bnep_sdp_search(bdaddr_t *src, bdaddr_t *dst, uint16_t service);
int bnep_sdp_register(bdaddr_t *device, uint16_t role);
void bnep_sdp_unregister(void);

int dun_sdp_search(bdaddr_t *src, bdaddr_t *dst, int *channel, int type);
int dun_sdp_register(bdaddr_t *device, uint8_t channel, int type);
void dun_sdp_unregister(void);
