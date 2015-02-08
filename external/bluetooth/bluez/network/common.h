
#define PANU_UUID	"00001115-0000-1000-8000-00805f9b34fb"
#define NAP_UUID	"00001116-0000-1000-8000-00805f9b34fb"
#define GN_UUID		"00001117-0000-1000-8000-00805f9b34fb"
#define BNEP_SVC_UUID	"0000000f-0000-1000-8000-00805f9b34fb"

int bnep_init(void);
int bnep_cleanup(void);

uint16_t bnep_service_id(const char *svc);
const char *bnep_uuid(uint16_t id);
const char *bnep_name(uint16_t id);

int bnep_kill_connection(bdaddr_t *dst);
int bnep_kill_all_connections(void);

int bnep_connadd(int sk, uint16_t role, char *dev);
int bnep_if_up(const char *devname);
int bnep_if_down(const char *devname);
int bnep_add_to_bridge(const char *devname, const char *bridge);
