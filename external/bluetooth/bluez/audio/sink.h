
#define AUDIO_SINK_INTERFACE "org.bluez.AudioSink"

typedef enum {
	SINK_STATE_DISCONNECTED,
	SINK_STATE_CONNECTING,
	SINK_STATE_CONNECTED,
	SINK_STATE_PLAYING,
} sink_state_t;

typedef void (*sink_state_cb) (struct audio_device *dev,
				sink_state_t old_state,
				sink_state_t new_state,
				void *user_data);

unsigned int sink_add_state_cb(sink_state_cb cb, void *user_data);
gboolean sink_remove_state_cb(unsigned int id);

struct sink *sink_init(struct audio_device *dev);
void sink_unregister(struct audio_device *dev);
gboolean sink_is_active(struct audio_device *dev);
gboolean sink_is_streaming(struct audio_device *dev);
avdtp_state_t sink_get_state(struct audio_device *dev);
gboolean sink_new_stream(struct audio_device *dev, struct avdtp *session,
				struct avdtp_stream *stream);
gboolean sink_setup_stream(struct sink *sink, struct avdtp *session);
gboolean sink_shutdown(struct sink *sink);
