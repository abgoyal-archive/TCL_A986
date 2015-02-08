
void unix_device_removed(struct audio_device *dev);

void unix_delay_report(struct audio_device *dev, uint8_t seid, uint16_t delay);

int unix_init(void);
void unix_exit(void);
