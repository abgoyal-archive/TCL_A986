

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/time.h>
#include <linux/cper.h>
#include <linux/acpi.h>

u64 cper_next_record_id(void)
{
	static atomic64_t seq;

	if (!atomic64_read(&seq))
		atomic64_set(&seq, ((u64)get_seconds()) << 32);

	return atomic64_inc_return(&seq);
}
EXPORT_SYMBOL_GPL(cper_next_record_id);

int apei_estatus_check_header(const struct acpi_hest_generic_status *estatus)
{
	if (estatus->data_length &&
	    estatus->data_length < sizeof(struct acpi_hest_generic_data))
		return -EINVAL;
	if (estatus->raw_data_length &&
	    estatus->raw_data_offset < sizeof(*estatus) + estatus->data_length)
		return -EINVAL;

	return 0;
}
EXPORT_SYMBOL_GPL(apei_estatus_check_header);

int apei_estatus_check(const struct acpi_hest_generic_status *estatus)
{
	struct acpi_hest_generic_data *gdata;
	unsigned int data_len, gedata_len;
	int rc;

	rc = apei_estatus_check_header(estatus);
	if (rc)
		return rc;
	data_len = estatus->data_length;
	gdata = (struct acpi_hest_generic_data *)(estatus + 1);
	while (data_len > sizeof(*gdata)) {
		gedata_len = gdata->error_data_length;
		if (gedata_len > data_len - sizeof(*gdata))
			return -EINVAL;
		data_len -= gedata_len + sizeof(*gdata);
	}
	if (data_len)
		return -EINVAL;

	return 0;
}
EXPORT_SYMBOL_GPL(apei_estatus_check);
