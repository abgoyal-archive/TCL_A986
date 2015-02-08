
struct fake_hid;
struct fake_input;

struct fake_hid {
	uint16_t vendor;
	uint16_t product;
	gboolean (*connect) (struct fake_input *fake_input, GError **err);
	int (*disconnect) (struct fake_input *fake_input);
	gboolean (*event) (GIOChannel *chan, GIOCondition cond, gpointer data);
	int (*setup_uinput) (struct fake_input *fake, struct fake_hid *fake_hid);
};

struct fake_hid *get_fake_hid(uint16_t vendor, uint16_t product);

int fake_hid_connadd(struct fake_input *fake, GIOChannel *intr_io,
						struct fake_hid *fake_hid);
