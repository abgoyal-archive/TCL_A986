
#define L2CAP_PSM_HIDP_CTRL 0x11
#define L2CAP_PSM_HIDP_INTR 0x13

int epox_presenter(const bdaddr_t *src, const bdaddr_t *dst, uint8_t channel);
int headset_presenter(const bdaddr_t *src, const bdaddr_t *dst, uint8_t channel);
int jthree_keyboard(const bdaddr_t *src, const bdaddr_t *dst, uint8_t channel);
int celluon_keyboard(const bdaddr_t *src, const bdaddr_t *dst, uint8_t channel);
