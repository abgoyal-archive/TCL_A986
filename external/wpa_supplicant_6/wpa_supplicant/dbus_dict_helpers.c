

#include "includes.h"
#include <dbus/dbus.h>

#include "common.h"
#include "dbus_dict_helpers.h"


dbus_bool_t wpa_dbus_dict_open_write(DBusMessageIter *iter,
				     DBusMessageIter *iter_dict)
{
	dbus_bool_t result;

	if (!iter || !iter_dict)
		return FALSE;

	result = dbus_message_iter_open_container(
		iter,
		DBUS_TYPE_ARRAY,
		DBUS_DICT_ENTRY_BEGIN_CHAR_AS_STRING
		DBUS_TYPE_STRING_AS_STRING
		DBUS_TYPE_VARIANT_AS_STRING
		DBUS_DICT_ENTRY_END_CHAR_AS_STRING,
		iter_dict);
	return result;
}


dbus_bool_t wpa_dbus_dict_close_write(DBusMessageIter *iter,
				      DBusMessageIter *iter_dict)
{
	if (!iter || !iter_dict)
		return FALSE;

	return dbus_message_iter_close_container(iter, iter_dict);
}


static const char * _wpa_get_type_as_string_from_type(const int type)
{
	switch(type) {
	case DBUS_TYPE_BYTE:
		return DBUS_TYPE_BYTE_AS_STRING;
	case DBUS_TYPE_BOOLEAN:
		return DBUS_TYPE_BOOLEAN_AS_STRING;
	case DBUS_TYPE_INT16:
		return DBUS_TYPE_INT16_AS_STRING;
	case DBUS_TYPE_UINT16:
		return DBUS_TYPE_UINT16_AS_STRING;
	case DBUS_TYPE_INT32:
		return DBUS_TYPE_INT32_AS_STRING;
	case DBUS_TYPE_UINT32:
		return DBUS_TYPE_UINT32_AS_STRING;
	case DBUS_TYPE_INT64:
		return DBUS_TYPE_INT64_AS_STRING;
	case DBUS_TYPE_UINT64:
		return DBUS_TYPE_UINT64_AS_STRING;
	case DBUS_TYPE_DOUBLE:
		return DBUS_TYPE_DOUBLE_AS_STRING;
	case DBUS_TYPE_STRING:
		return DBUS_TYPE_STRING_AS_STRING;
	case DBUS_TYPE_OBJECT_PATH:
		return DBUS_TYPE_OBJECT_PATH_AS_STRING;
	case DBUS_TYPE_ARRAY:
		return DBUS_TYPE_ARRAY_AS_STRING;
	default:
		return NULL;
	}
}


static dbus_bool_t _wpa_dbus_add_dict_entry_start(
	DBusMessageIter *iter_dict, DBusMessageIter *iter_dict_entry,
	const char *key, const int value_type)
{
	if (!dbus_message_iter_open_container(iter_dict,
					      DBUS_TYPE_DICT_ENTRY, NULL,
					      iter_dict_entry))
		return FALSE;

	if (!dbus_message_iter_append_basic(iter_dict_entry, DBUS_TYPE_STRING,
					    &key))
		return FALSE;

	return TRUE;
}


static dbus_bool_t _wpa_dbus_add_dict_entry_end(
	DBusMessageIter *iter_dict, DBusMessageIter *iter_dict_entry,
	DBusMessageIter *iter_dict_val)
{
	if (!dbus_message_iter_close_container(iter_dict_entry, iter_dict_val))
		return FALSE;
	if (!dbus_message_iter_close_container(iter_dict, iter_dict_entry))
		return FALSE;

	return TRUE;
}


static dbus_bool_t _wpa_dbus_add_dict_entry_basic(DBusMessageIter *iter_dict,
						  const char *key,
						  const int value_type,
						  const void *value)
{
	DBusMessageIter iter_dict_entry, iter_dict_val;
	const char *type_as_string = NULL;

	type_as_string = _wpa_get_type_as_string_from_type(value_type);
	if (!type_as_string)
		return FALSE;

	if (!_wpa_dbus_add_dict_entry_start(iter_dict, &iter_dict_entry,
					    key, value_type))
		return FALSE;

	if (!dbus_message_iter_open_container(&iter_dict_entry,
					      DBUS_TYPE_VARIANT,
					      type_as_string, &iter_dict_val))
		return FALSE;

	if (!dbus_message_iter_append_basic(&iter_dict_val, value_type, value))
		return FALSE;

	if (!_wpa_dbus_add_dict_entry_end(iter_dict, &iter_dict_entry,
					  &iter_dict_val))
		return FALSE;

	return TRUE;
}


static dbus_bool_t _wpa_dbus_add_dict_entry_byte_array(
	DBusMessageIter *iter_dict, const char *key,
	const char *value, const dbus_uint32_t value_len)
{
	DBusMessageIter iter_dict_entry, iter_dict_val, iter_array;
	dbus_uint32_t i;

	if (!_wpa_dbus_add_dict_entry_start(iter_dict, &iter_dict_entry,
					    key, DBUS_TYPE_ARRAY))
		return FALSE;

	if (!dbus_message_iter_open_container(&iter_dict_entry,
					      DBUS_TYPE_VARIANT,
					      DBUS_TYPE_ARRAY_AS_STRING
					      DBUS_TYPE_BYTE_AS_STRING,
					      &iter_dict_val))
		return FALSE;

	if (!dbus_message_iter_open_container(&iter_dict_val, DBUS_TYPE_ARRAY,
					      DBUS_TYPE_BYTE_AS_STRING,
					      &iter_array))
		return FALSE;

	for (i = 0; i < value_len; i++) {
		if (!dbus_message_iter_append_basic(&iter_array,
						    DBUS_TYPE_BYTE,
						    &(value[i])))
			return FALSE;
	}

	if (!dbus_message_iter_close_container(&iter_dict_val, &iter_array))
		return FALSE;

	if (!_wpa_dbus_add_dict_entry_end(iter_dict, &iter_dict_entry,
					  &iter_dict_val))
		return FALSE;

	return TRUE;
}


dbus_bool_t wpa_dbus_dict_append_string(DBusMessageIter *iter_dict,
					const char *key, const char *value)
{
	if (!key || !value)
		return FALSE;
	return _wpa_dbus_add_dict_entry_basic(iter_dict, key, DBUS_TYPE_STRING,
					      &value);
}


dbus_bool_t wpa_dbus_dict_append_byte(DBusMessageIter *iter_dict,
				      const char *key, const char value)
{
	if (!key)
		return FALSE;
	return _wpa_dbus_add_dict_entry_basic(iter_dict, key, DBUS_TYPE_BYTE,
					      &value);
}


dbus_bool_t wpa_dbus_dict_append_bool(DBusMessageIter *iter_dict,
				      const char *key, const dbus_bool_t value)
{
	if (!key)
		return FALSE;
	return _wpa_dbus_add_dict_entry_basic(iter_dict, key,
					      DBUS_TYPE_BOOLEAN, &value);
}


dbus_bool_t wpa_dbus_dict_append_int16(DBusMessageIter *iter_dict,
				       const char *key,
				       const dbus_int16_t value)
{
	if (!key)
		return FALSE;
	return _wpa_dbus_add_dict_entry_basic(iter_dict, key, DBUS_TYPE_INT16,
					      &value);
}


dbus_bool_t wpa_dbus_dict_append_uint16(DBusMessageIter *iter_dict,
					const char *key,
					const dbus_uint16_t value)
{
	if (!key)
		return FALSE;
	return _wpa_dbus_add_dict_entry_basic(iter_dict, key, DBUS_TYPE_UINT16,
					      &value);
}


dbus_bool_t wpa_dbus_dict_append_int32(DBusMessageIter *iter_dict,
				       const char *key,
				       const dbus_int32_t value)
{
	if (!key)
		return FALSE;
	return _wpa_dbus_add_dict_entry_basic(iter_dict, key, DBUS_TYPE_INT32,
					      &value);
}


dbus_bool_t wpa_dbus_dict_append_uint32(DBusMessageIter *iter_dict,
					const char *key,
					const dbus_uint32_t value)
{
	if (!key)
		return FALSE;
	return _wpa_dbus_add_dict_entry_basic(iter_dict, key, DBUS_TYPE_UINT32,
					      &value);
}


dbus_bool_t wpa_dbus_dict_append_int64(DBusMessageIter *iter_dict,
				       const char *key,
				       const dbus_int64_t value)
{
	if (!key)
		return FALSE;
	return _wpa_dbus_add_dict_entry_basic(iter_dict, key, DBUS_TYPE_INT64,
					      &value);
}


dbus_bool_t wpa_dbus_dict_append_uint64(DBusMessageIter *iter_dict,
					const char *key,
					const dbus_uint64_t value)
{
	if (!key)
		return FALSE;
	return _wpa_dbus_add_dict_entry_basic(iter_dict, key, DBUS_TYPE_UINT64,
					      &value);
}


dbus_bool_t wpa_dbus_dict_append_double(DBusMessageIter *iter_dict,
					const char * key,
					const double value)
{
	if (!key)
		return FALSE;
	return _wpa_dbus_add_dict_entry_basic(iter_dict, key, DBUS_TYPE_DOUBLE,
					      &value);
}


dbus_bool_t wpa_dbus_dict_append_object_path(DBusMessageIter *iter_dict,
					     const char *key,
					     const char *value)
{
	if (!key || !value)
		return FALSE;
	return _wpa_dbus_add_dict_entry_basic(iter_dict, key,
					      DBUS_TYPE_OBJECT_PATH, &value);
}


dbus_bool_t wpa_dbus_dict_append_byte_array(DBusMessageIter *iter_dict,
					    const char *key,
					    const char *value,
					    const dbus_uint32_t value_len)
{
	if (!key)
		return FALSE;
	if (!value && (value_len != 0))
		return FALSE;
	return _wpa_dbus_add_dict_entry_byte_array(iter_dict, key, value,
						   value_len);
}


dbus_bool_t wpa_dbus_dict_begin_string_array(DBusMessageIter *iter_dict,
					     const char *key,
					     DBusMessageIter *iter_dict_entry,
					     DBusMessageIter *iter_dict_val,
					     DBusMessageIter *iter_array)
{
	if (!iter_dict || !iter_dict_entry || !iter_dict_val || !iter_array)
		return FALSE;

	if (!_wpa_dbus_add_dict_entry_start(iter_dict, iter_dict_entry,
					    key, DBUS_TYPE_ARRAY))
		return FALSE;

	if (!dbus_message_iter_open_container(iter_dict_entry,
					      DBUS_TYPE_VARIANT,
					      DBUS_TYPE_ARRAY_AS_STRING
					      DBUS_TYPE_STRING_AS_STRING,
					      iter_dict_val))
		return FALSE;

	if (!dbus_message_iter_open_container(iter_dict_val, DBUS_TYPE_ARRAY,
					      DBUS_TYPE_BYTE_AS_STRING,
					      iter_array))
		return FALSE;

	return TRUE;
}


dbus_bool_t wpa_dbus_dict_string_array_add_element(DBusMessageIter *iter_array,
						   const char *elem)
{
	if (!iter_array || !elem)
		return FALSE;

	return dbus_message_iter_append_basic(iter_array, DBUS_TYPE_STRING,
					      &elem);
}


dbus_bool_t wpa_dbus_dict_end_string_array(DBusMessageIter *iter_dict,
					   DBusMessageIter *iter_dict_entry,
					   DBusMessageIter *iter_dict_val,
					   DBusMessageIter *iter_array)
{
	if (!iter_dict || !iter_dict_entry || !iter_dict_val || !iter_array)
		return FALSE;

	if (!dbus_message_iter_close_container(iter_dict_val, iter_array))
		return FALSE;

	if (!_wpa_dbus_add_dict_entry_end(iter_dict, iter_dict_entry,
					  iter_dict_val))
		return FALSE;

	return TRUE;
}


dbus_bool_t wpa_dbus_dict_append_string_array(DBusMessageIter *iter_dict,
					      const char *key,
					      const char **items,
					      const dbus_uint32_t num_items)
{
	DBusMessageIter iter_dict_entry, iter_dict_val, iter_array;
	dbus_uint32_t i;

	if (!key)
		return FALSE;
	if (!items && (num_items != 0))
		return FALSE;

	if (!wpa_dbus_dict_begin_string_array(iter_dict, key,
					      &iter_dict_entry, &iter_dict_val,
					      &iter_array))
		return FALSE;

	for (i = 0; i < num_items; i++) {
		if (!wpa_dbus_dict_string_array_add_element(&iter_array,
							    items[i]))
			return FALSE;
	}

	if (!wpa_dbus_dict_end_string_array(iter_dict, &iter_dict_entry,
					    &iter_dict_val, &iter_array))
		return FALSE;

	return TRUE;
}


/*****************************************************/
/* Stuff for reading dicts                           */
/*****************************************************/

dbus_bool_t wpa_dbus_dict_open_read(DBusMessageIter *iter,
				    DBusMessageIter *iter_dict)
{
	if (!iter || !iter_dict)
		return FALSE;

	if (dbus_message_iter_get_arg_type(iter) != DBUS_TYPE_ARRAY ||
	    dbus_message_iter_get_element_type(iter) != DBUS_TYPE_DICT_ENTRY)
		return FALSE;

	dbus_message_iter_recurse(iter, iter_dict);
	return TRUE;
}


#define BYTE_ARRAY_CHUNK_SIZE 34
#define BYTE_ARRAY_ITEM_SIZE (sizeof(char))

static dbus_bool_t _wpa_dbus_dict_entry_get_byte_array(
	DBusMessageIter *iter, int array_type,
	struct wpa_dbus_dict_entry *entry)
{
	dbus_uint32_t count = 0;
	dbus_bool_t success = FALSE;
	char *buffer;

	entry->bytearray_value = NULL;
	entry->array_type = DBUS_TYPE_BYTE;

	buffer = os_zalloc(BYTE_ARRAY_ITEM_SIZE * BYTE_ARRAY_CHUNK_SIZE);
	if (!buffer) {
		perror("_wpa_dbus_dict_entry_get_byte_array[dbus]: out of "
		       "memory");
		goto done;
	}

	entry->bytearray_value = buffer;
	entry->array_len = 0;
	while (dbus_message_iter_get_arg_type(iter) == DBUS_TYPE_BYTE) {
		char byte;

		if ((count % BYTE_ARRAY_CHUNK_SIZE) == 0 && count != 0) {
			buffer = realloc(buffer, BYTE_ARRAY_ITEM_SIZE *
					 (count + BYTE_ARRAY_CHUNK_SIZE));
			if (buffer == NULL) {
				perror("_wpa_dbus_dict_entry_get_byte_array["
				       "dbus] out of memory trying to "
				       "retrieve the string array");
				goto done;
			}
		}
		entry->bytearray_value = buffer;

		dbus_message_iter_get_basic(iter, &byte);
		entry->bytearray_value[count] = byte;
		entry->array_len = ++count;
		dbus_message_iter_next(iter);
	}

	/* Zero-length arrays are valid. */
	if (entry->array_len == 0) {
		free(entry->bytearray_value);
		entry->bytearray_value = NULL;
	}

	success = TRUE;

done:
	return success;
}


#define STR_ARRAY_CHUNK_SIZE 8
#define STR_ARRAY_ITEM_SIZE (sizeof(char *))

static dbus_bool_t _wpa_dbus_dict_entry_get_string_array(
	DBusMessageIter *iter, int array_type,
	struct wpa_dbus_dict_entry *entry)
{
	dbus_uint32_t count = 0;
	dbus_bool_t success = FALSE;
	char **buffer;

	entry->strarray_value = NULL;
	entry->array_type = DBUS_TYPE_STRING;

	buffer = os_zalloc(STR_ARRAY_ITEM_SIZE * STR_ARRAY_CHUNK_SIZE);
	if (buffer == NULL) {
		perror("_wpa_dbus_dict_entry_get_string_array[dbus] out of "
		       "memory trying to retrieve a string array");
		goto done;
	}

	entry->strarray_value = buffer;
	entry->array_len = 0;
	while (dbus_message_iter_get_arg_type(iter) == DBUS_TYPE_STRING) {
		const char *value;
		char *str;

		if ((count % STR_ARRAY_CHUNK_SIZE) == 0 && count != 0) {
			buffer = realloc(buffer, STR_ARRAY_ITEM_SIZE *
					 (count + STR_ARRAY_CHUNK_SIZE));
			if (buffer == NULL) {
				perror("_wpa_dbus_dict_entry_get_string_array["
				       "dbus] out of memory trying to "
				       "retrieve the string array");
				goto done;
			}
		}
		entry->strarray_value = buffer;

		dbus_message_iter_get_basic(iter, &value);
		str = strdup(value);
		if (str == NULL) {
			perror("_wpa_dbus_dict_entry_get_string_array[dbus] "
			       "out of memory trying to duplicate the string "
			       "array");
			goto done;
		}
		entry->strarray_value[count] = str;
		entry->array_len = ++count;
		dbus_message_iter_next(iter);
	}

	/* Zero-length arrays are valid. */
	if (entry->array_len == 0) {
		free(entry->strarray_value);
		entry->strarray_value = NULL;
	}

	success = TRUE;

done:
	return success;
}


static dbus_bool_t _wpa_dbus_dict_entry_get_array(
	DBusMessageIter *iter_dict_val, struct wpa_dbus_dict_entry *entry)
{
	int array_type = dbus_message_iter_get_element_type(iter_dict_val);
	dbus_bool_t success = FALSE;
	DBusMessageIter iter_array;

	if (!entry)
		return FALSE;

	dbus_message_iter_recurse(iter_dict_val, &iter_array);

 	switch (array_type) {
	case DBUS_TYPE_BYTE:
		success = _wpa_dbus_dict_entry_get_byte_array(&iter_array,
							      array_type,
							      entry);
		break;
	case DBUS_TYPE_STRING:
		success = _wpa_dbus_dict_entry_get_string_array(&iter_array,
								array_type,
								entry);
		break;
	default:
		break;
	}

	return success;
}


static dbus_bool_t _wpa_dbus_dict_fill_value_from_variant(
	struct wpa_dbus_dict_entry *entry, DBusMessageIter *iter_dict_val)
{
	dbus_bool_t success = TRUE;

	switch (entry->type) {
	case DBUS_TYPE_STRING: {
		const char *v;
		dbus_message_iter_get_basic(iter_dict_val, &v);
		entry->str_value = strdup(v);
		break;
	}
	case DBUS_TYPE_BOOLEAN: {
		dbus_bool_t v;
		dbus_message_iter_get_basic(iter_dict_val, &v);
		entry->bool_value = v;
		break;
	}
	case DBUS_TYPE_BYTE: {
		char v;
		dbus_message_iter_get_basic(iter_dict_val, &v);
		entry->byte_value = v;
		break;
	}
	case DBUS_TYPE_INT16: {
		dbus_int16_t v;
		dbus_message_iter_get_basic(iter_dict_val, &v);
		entry->int16_value = v;
		break;
	}
	case DBUS_TYPE_UINT16: {
		dbus_uint16_t v;
		dbus_message_iter_get_basic(iter_dict_val, &v);
		entry->uint16_value = v;
		break;
	}
	case DBUS_TYPE_INT32: {
		dbus_int32_t v;
		dbus_message_iter_get_basic(iter_dict_val, &v);
		entry->int32_value = v;
		break;
	}
	case DBUS_TYPE_UINT32: {
		dbus_uint32_t v;
		dbus_message_iter_get_basic(iter_dict_val, &v);
		entry->uint32_value = v;
		break;
	}
	case DBUS_TYPE_INT64: {
		dbus_int64_t v;
		dbus_message_iter_get_basic(iter_dict_val, &v);
		entry->int64_value = v;
		break;
	}
	case DBUS_TYPE_UINT64: {
		dbus_uint64_t v;
		dbus_message_iter_get_basic(iter_dict_val, &v);
		entry->uint64_value = v;
		break;
	}
	case DBUS_TYPE_DOUBLE: {
		double v;
		dbus_message_iter_get_basic(iter_dict_val, &v);
		entry->double_value = v;
		break;
	}
	case DBUS_TYPE_OBJECT_PATH: {
		char *v;
		dbus_message_iter_get_basic(iter_dict_val, &v);
		entry->str_value = strdup(v);
		break;
	}
	case DBUS_TYPE_ARRAY: {
		success = _wpa_dbus_dict_entry_get_array(iter_dict_val, entry);
		break;
	}
	default:
		success = FALSE;
		break;
	}

	return success;
}


dbus_bool_t wpa_dbus_dict_get_entry(DBusMessageIter *iter_dict,
				    struct wpa_dbus_dict_entry * entry)
{
	DBusMessageIter iter_dict_entry, iter_dict_val;
	int type;
	const char *key;

	if (!iter_dict || !entry)
		goto error;

	if (dbus_message_iter_get_arg_type(iter_dict) != DBUS_TYPE_DICT_ENTRY)
		goto error;

	dbus_message_iter_recurse(iter_dict, &iter_dict_entry);
	dbus_message_iter_get_basic(&iter_dict_entry, &key);
	entry->key = key;

	if (!dbus_message_iter_next(&iter_dict_entry))
		goto error;
	type = dbus_message_iter_get_arg_type(&iter_dict_entry);
	if (type != DBUS_TYPE_VARIANT)
		goto error;

	dbus_message_iter_recurse(&iter_dict_entry, &iter_dict_val);
	entry->type = dbus_message_iter_get_arg_type(&iter_dict_val);
	if (!_wpa_dbus_dict_fill_value_from_variant(entry, &iter_dict_val))
		goto error;

	dbus_message_iter_next(iter_dict);
	return TRUE;

error:
	if (entry) {
		wpa_dbus_dict_entry_clear(entry);
		entry->type = DBUS_TYPE_INVALID;
		entry->array_type = DBUS_TYPE_INVALID;
	}

	return FALSE;
}


dbus_bool_t wpa_dbus_dict_has_dict_entry(DBusMessageIter *iter_dict)
{
	if (!iter_dict) {
		perror("wpa_dbus_dict_has_dict_entry[dbus]: out of memory");
		return FALSE;
	}
	return dbus_message_iter_get_arg_type(iter_dict) ==
		DBUS_TYPE_DICT_ENTRY;
}


void wpa_dbus_dict_entry_clear(struct wpa_dbus_dict_entry *entry)
{
	unsigned int i;

	if (!entry)
		return;
	switch (entry->type) {
	case DBUS_TYPE_OBJECT_PATH:
	case DBUS_TYPE_STRING:
		free(entry->str_value);
		break;
	case DBUS_TYPE_ARRAY:
		switch (entry->array_type) {
		case DBUS_TYPE_BYTE:
			free(entry->bytearray_value);
			break;
		case DBUS_TYPE_STRING:
			for (i = 0; i < entry->array_len; i++)
				free(entry->strarray_value[i]);
			free(entry->strarray_value);
			break;
		}
		break;
	}

	memset(entry, 0, sizeof(struct wpa_dbus_dict_entry));
}
