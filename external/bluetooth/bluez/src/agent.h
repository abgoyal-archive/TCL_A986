
struct agent;

typedef void (*agent_cb) (struct agent *agent, DBusError *err,
				void *user_data);

typedef void (*agent_pincode_cb) (struct agent *agent, DBusError *err,
					const char *pincode, void *user_data);

typedef void (*agent_passkey_cb) (struct agent *agent, DBusError *err,
					uint32_t passkey, void *user_data);

typedef void (*agent_oob_data_cb) (struct agent *agent, DBusError *err,
					uint8_t *hash, uint8_t *randomizer,
					void *user_data);

typedef void (*agent_remove_cb) (struct agent *agent, void *user_data);

struct agent *agent_create(struct btd_adapter *adapter, const char *name,
				const char *path, uint8_t capability,
				gboolean oob, agent_remove_cb cb,
				void *remove_cb_data);

void agent_free(struct agent *agent);

int agent_authorize(struct agent *agent, const char *path,
			const char *uuid, agent_cb cb, void *user_data,
			GDestroyNotify destroy);

int agent_request_pincode(struct agent *agent, struct btd_device *device,
				agent_pincode_cb cb, void *user_data,
				GDestroyNotify destroy);

int agent_confirm_mode_change(struct agent *agent, const char *new_mode,
				agent_cb cb, void *user_data,
				GDestroyNotify destroy);

int agent_request_passkey(struct agent *agent, struct btd_device *device,
				agent_passkey_cb cb, void *user_data,
				GDestroyNotify destroy);

int agent_request_oob_data(struct agent *agent, struct btd_device *device,
				agent_oob_data_cb cb, void *user_data,
				GDestroyNotify destroy);

int agent_request_oob_availability(struct agent *agent,
                                        const char *path, agent_cb cb,
                                        void *user_data, GDestroyNotify destroy);

int agent_request_confirmation(struct agent *agent, struct btd_device *device,
				uint32_t passkey, agent_cb cb,
				void *user_data, GDestroyNotify destroy);

int agent_display_passkey(struct agent *agent, struct btd_device *device,
				uint32_t passkey);

int agent_cancel(struct agent *agent);

gboolean agent_is_busy(struct agent *agent, void *user_data);

uint8_t agent_get_io_capability(struct agent *agent);
gboolean agent_get_oob_capability(struct agent *agent);

gboolean agent_matches(struct agent *agent, const char *name, const char *path);

void agent_init(void);
void agent_exit(void);

