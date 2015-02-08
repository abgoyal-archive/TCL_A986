


#ifdef CONFIG_IIO_TRIGGER
int iio_device_register_trigger_consumer(struct iio_dev *dev_info);

int iio_device_unregister_trigger_consumer(struct iio_dev *dev_info);

#else

static int iio_device_register_trigger_consumer(struct iio_dev *dev_info)
{
	return 0;
};

static int iio_device_unregister_trigger_consumer(struct iio_dev *dev_info)
{
	return 0;
};

#endif /* CONFIG_TRIGGER_CONSUMER */



