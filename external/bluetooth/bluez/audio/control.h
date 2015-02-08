
#define AUDIO_CONTROL_INTERFACE "org.bluez.Control"

typedef enum {
	AVCTP_STATE_DISCONNECTED = 0,
	AVCTP_STATE_CONNECTING,
	AVCTP_STATE_CONNECTED
} avctp_state_t;

typedef void (*avctp_state_cb) (struct audio_device *dev,
				avctp_state_t old_state,
				avctp_state_t new_state,
				void *user_data);

unsigned int avctp_add_state_cb(avctp_state_cb cb, void *user_data);
gboolean avctp_remove_state_cb(unsigned int id);

int avrcp_register(DBusConnection *conn, const bdaddr_t *src, GKeyFile *config);
void avrcp_unregister(const bdaddr_t *src);

gboolean avrcp_connect(struct audio_device *dev);
void avrcp_disconnect(struct audio_device *dev);

struct control *control_init(struct audio_device *dev, uint16_t uuid16);
void control_update(struct audio_device *dev, uint16_t uuid16);
void control_unregister(struct audio_device *dev);
gboolean control_is_active(struct audio_device *dev);
