
#define AUDIO_GATEWAY_INTERFACE "org.bluez.HandsfreeGateway"

#define DEFAULT_HFP_HS_CHANNEL 7

typedef enum {
	GATEWAY_STATE_DISCONNECTED,
	GATEWAY_STATE_CONNECTING,
	GATEWAY_STATE_CONNECTED,
	GATEWAY_STATE_PLAYING,
} gateway_state_t;

typedef void (*gateway_stream_cb_t) (struct audio_device *dev, GError *err,
		void *user_data);

void gateway_unregister(struct audio_device *dev);
struct gateway *gateway_init(struct audio_device *device);
gboolean gateway_is_connected(struct audio_device *dev);
int gateway_connect_rfcomm(struct audio_device *dev, GIOChannel *io);
int gateway_connect_sco(struct audio_device *dev, GIOChannel *chan);
void gateway_start_service(struct audio_device *device);
gboolean gateway_request_stream(struct audio_device *dev,
			gateway_stream_cb_t cb, void *user_data);
int gateway_config_stream(struct audio_device *dev, gateway_stream_cb_t cb,
			void *user_data);
gboolean gateway_cancel_stream(struct audio_device *dev, unsigned int id);
int gateway_get_sco_fd(struct audio_device *dev);
void gateway_suspend_stream(struct audio_device *dev);
