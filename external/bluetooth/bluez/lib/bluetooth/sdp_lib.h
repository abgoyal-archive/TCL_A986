
#ifndef __SDP_LIB_H
#define __SDP_LIB_H

#include <sys/socket.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef void(*sdp_list_func_t)(void *, void *);
typedef void(*sdp_free_func_t)(void *);
typedef int (*sdp_comp_func_t)(const void *, const void *);

sdp_list_t *sdp_list_append(sdp_list_t *list, void *d);
sdp_list_t *sdp_list_remove(sdp_list_t *list, void *d);
sdp_list_t *sdp_list_insert_sorted(sdp_list_t *list, void *data, sdp_comp_func_t f);
void        sdp_list_free(sdp_list_t *list, sdp_free_func_t f);

static inline int sdp_list_len(const sdp_list_t *list)
{
	int n = 0;
	for (; list; list = list->next)
		n++;
	return n;
}

static inline sdp_list_t *sdp_list_find(sdp_list_t *list, void *u, sdp_comp_func_t f)
{
	for (; list; list = list->next)
		if (f(list->data, u) == 0)
			return list;
	return NULL;
}

static inline void sdp_list_foreach(sdp_list_t *list, sdp_list_func_t f, void *u)
{
	for (; list; list = list->next)
		f(list->data, u);
}

#define SDP_RECORD_PERSIST	0x01
#define SDP_DEVICE_RECORD	0x02

#define SDP_RETRY_IF_BUSY	0x01
#define SDP_WAIT_ON_CLOSE	0x02
#define SDP_NON_BLOCKING	0x04

typedef struct {
	int sock;
	int state;
	int local;
	int flags;
	uint16_t tid;	// Current transaction ID
	void *priv;
} sdp_session_t;

typedef enum {
	/*
	 *  Attributes are specified as individual elements
	 */
	SDP_ATTR_REQ_INDIVIDUAL = 1,
	/*
	 *  Attributes are specified as a range
	 */
	SDP_ATTR_REQ_RANGE
} sdp_attrreq_type_t;

typedef void sdp_callback_t(uint8_t type, uint16_t status, uint8_t *rsp, size_t size, void *udata);

sdp_session_t *sdp_connect(const bdaddr_t *src, const bdaddr_t *dst, uint32_t flags);
int sdp_close(sdp_session_t *session);
int sdp_get_socket(const sdp_session_t *session);

sdp_session_t *sdp_create(int sk, uint32_t flags);
int sdp_get_error(sdp_session_t *session);
int sdp_process(sdp_session_t *session);
int sdp_set_notify(sdp_session_t *session, sdp_callback_t *func, void *udata);

int sdp_service_search_async(sdp_session_t *session, const sdp_list_t *search, uint16_t max_rec_num);
int sdp_service_attr_async(sdp_session_t *session, uint32_t handle, sdp_attrreq_type_t reqtype, const sdp_list_t *attrid_list);
int sdp_service_search_attr_async(sdp_session_t *session, const sdp_list_t *search, sdp_attrreq_type_t reqtype, const sdp_list_t *attrid_list);

uint16_t sdp_gen_tid(sdp_session_t *session);

int sdp_general_inquiry(inquiry_info *ii, int dev_num, int duration, uint8_t *found);

/* flexible extraction of basic attributes - Jean II */
int sdp_get_int_attr(const sdp_record_t *rec, uint16_t attr, int *value);
int sdp_get_string_attr(const sdp_record_t *rec, uint16_t attr, char *value, int valuelen);

sdp_data_t *sdp_data_alloc(uint8_t dtd, const void *value);
sdp_data_t *sdp_data_alloc_with_length(uint8_t dtd, const void *value, uint32_t length);
void sdp_data_free(sdp_data_t *data);
sdp_data_t *sdp_data_get(const sdp_record_t *rec, uint16_t attr_id);

sdp_data_t *sdp_seq_alloc(void **dtds, void **values, int len);
sdp_data_t *sdp_seq_alloc_with_length(void **dtds, void **values, int *length, int len);
sdp_data_t *sdp_seq_append(sdp_data_t *seq, sdp_data_t *data);

int sdp_attr_add(sdp_record_t *rec, uint16_t attr, sdp_data_t *data);
void sdp_attr_remove(sdp_record_t *rec, uint16_t attr);
void sdp_attr_replace(sdp_record_t *rec, uint16_t attr, sdp_data_t *data);
int sdp_set_uuidseq_attr(sdp_record_t *rec, uint16_t attr, sdp_list_t *seq);
int sdp_get_uuidseq_attr(const sdp_record_t *rec, uint16_t attr, sdp_list_t **seqp);


int sdp_attr_add_new(sdp_record_t *rec, uint16_t attr, uint8_t dtd, const void *p);

void sdp_set_info_attr(sdp_record_t *rec, const char *name, const char *prov, const char *desc);

static inline int sdp_set_service_classes(sdp_record_t *rec, sdp_list_t *seq)
{
	return sdp_set_uuidseq_attr(rec, SDP_ATTR_SVCLASS_ID_LIST, seq);
}

static inline int sdp_get_service_classes(const sdp_record_t *rec, sdp_list_t **seqp)
{
	return sdp_get_uuidseq_attr(rec, SDP_ATTR_SVCLASS_ID_LIST, seqp);
}

static inline int sdp_set_browse_groups(sdp_record_t *rec, sdp_list_t *seq)
{
	return sdp_set_uuidseq_attr(rec, SDP_ATTR_BROWSE_GRP_LIST, seq);
}

int sdp_set_access_protos(sdp_record_t *rec, const sdp_list_t *proto);

int sdp_set_add_access_protos(sdp_record_t *rec, const sdp_list_t *proto);

int sdp_get_proto_port(const sdp_list_t *list, int proto);

sdp_data_t *sdp_get_proto_desc(sdp_list_t *list, int proto);

int sdp_set_lang_attr(sdp_record_t *rec, const sdp_list_t *list);

static inline int sdp_set_service_ttl(sdp_record_t *rec, uint32_t ttl)
{
	return sdp_attr_add_new(rec, SDP_ATTR_SVCINFO_TTL, SDP_UINT32, &ttl);
}

static inline int sdp_set_record_state(sdp_record_t *rec, uint32_t state)
{
	return sdp_attr_add_new(rec, SDP_ATTR_RECORD_STATE, SDP_UINT32, &state);
}

void sdp_set_service_id(sdp_record_t *rec, uuid_t uuid);

void sdp_set_group_id(sdp_record_t *rec, uuid_t grouuuid);

static inline int sdp_set_service_avail(sdp_record_t *rec, uint8_t avail)
{
	return sdp_attr_add_new(rec, SDP_ATTR_SERVICE_AVAILABILITY, SDP_UINT8, &avail);
}

int sdp_set_profile_descs(sdp_record_t *rec, const sdp_list_t *desc);

void sdp_set_url_attr(sdp_record_t *rec, const char *clientExecURL, const char *docURL, const char *iconURL);

int sdp_service_search_req(sdp_session_t *session, const sdp_list_t *search, uint16_t max_rec_num, sdp_list_t **rsp_list);

sdp_record_t *sdp_service_attr_req(sdp_session_t *session, uint32_t handle, sdp_attrreq_type_t reqtype, const sdp_list_t *attrid_list);

int sdp_service_search_attr_req(sdp_session_t *session, const sdp_list_t *search, sdp_attrreq_type_t reqtype, const sdp_list_t *attrid_list, sdp_list_t **rsp_list);

sdp_record_t *sdp_record_alloc(void);
void sdp_record_free(sdp_record_t *rec);

int sdp_device_record_register_binary(sdp_session_t *session, bdaddr_t *device, uint8_t *data, uint32_t size, uint8_t flags, uint32_t *handle);
int sdp_device_record_register(sdp_session_t *session, bdaddr_t *device, sdp_record_t *rec, uint8_t flags);
int sdp_record_register(sdp_session_t *session, sdp_record_t *rec, uint8_t flags);

int sdp_device_record_unregister_binary(sdp_session_t *session, bdaddr_t *device, uint32_t handle);
int sdp_device_record_unregister(sdp_session_t *session, bdaddr_t *device, sdp_record_t *rec);
int sdp_record_unregister(sdp_session_t *session, sdp_record_t *rec);

int sdp_device_record_update_binary(sdp_session_t *session, bdaddr_t *device, uint32_t handle, uint8_t *data, uint32_t size);
int sdp_device_record_update(sdp_session_t *session, bdaddr_t *device, const sdp_record_t *rec);
int sdp_record_update(sdp_session_t *sess, const sdp_record_t *rec);

void sdp_record_print(const sdp_record_t *rec);

uuid_t *sdp_uuid16_create(uuid_t *uuid, uint16_t data);
uuid_t *sdp_uuid32_create(uuid_t *uuid, uint32_t data);
uuid_t *sdp_uuid128_create(uuid_t *uuid, const void *data);
int sdp_uuid16_cmp(const void *p1, const void *p2);
int sdp_uuid128_cmp(const void *p1, const void *p2);
int sdp_uuid_cmp(const void *p1, const void *p2);
uuid_t *sdp_uuid_to_uuid128(const uuid_t *uuid);
void sdp_uuid16_to_uuid128(uuid_t *uuid128, const uuid_t *uuid16);
void sdp_uuid32_to_uuid128(uuid_t *uuid128, const uuid_t *uuid32);
int sdp_uuid128_to_uuid(uuid_t *uuid);
int sdp_uuid_to_proto(uuid_t *uuid);
int sdp_uuid_extract(const uint8_t *buffer, int bufsize, uuid_t *uuid, int *scanned);
void sdp_uuid_print(const uuid_t *uuid);

#define MAX_LEN_UUID_STR 37
#define MAX_LEN_PROTOCOL_UUID_STR 8
#define MAX_LEN_SERVICECLASS_UUID_STR 28
#define MAX_LEN_PROFILEDESCRIPTOR_UUID_STR 28

int sdp_uuid2strn(const uuid_t *uuid, char *str, size_t n);
int sdp_proto_uuid2strn(const uuid_t *uuid, char *str, size_t n);
int sdp_svclass_uuid2strn(const uuid_t *uuid, char *str, size_t n);
int sdp_profile_uuid2strn(const uuid_t *uuid, char *str, size_t n);


int sdp_get_access_protos(const sdp_record_t *rec, sdp_list_t **protos);

int sdp_get_add_access_protos(const sdp_record_t *rec, sdp_list_t **protos);

static inline int sdp_get_browse_groups(const sdp_record_t *rec, sdp_list_t **seqp)
{
	return sdp_get_uuidseq_attr(rec, SDP_ATTR_BROWSE_GRP_LIST, seqp);
}

int sdp_get_lang_attr(const sdp_record_t *rec, sdp_list_t **langSeq);

int sdp_get_profile_descs(const sdp_record_t *rec, sdp_list_t **profDesc);

int sdp_get_server_ver(const sdp_record_t *rec, sdp_list_t **pVnumList);

int sdp_get_service_id(const sdp_record_t *rec, uuid_t *uuid);
int sdp_get_group_id(const sdp_record_t *rec, uuid_t *uuid);
int sdp_get_record_state(const sdp_record_t *rec, uint32_t *svcRecState);
int sdp_get_service_avail(const sdp_record_t *rec, uint8_t *svcAvail);
int sdp_get_service_ttl(const sdp_record_t *rec, uint32_t *svcTTLInfo);
int sdp_get_database_state(const sdp_record_t *rec, uint32_t *svcDBState);

static inline int sdp_get_service_name(const sdp_record_t *rec, char *str, int len)
{
	return sdp_get_string_attr(rec, SDP_ATTR_SVCNAME_PRIMARY, str, len);
}

static inline int sdp_get_service_desc(const sdp_record_t *rec, char *str, int len)
{
	return sdp_get_string_attr(rec, SDP_ATTR_SVCDESC_PRIMARY, str, len);
}

static inline int sdp_get_provider_name(const sdp_record_t *rec, char *str, int len)
{
	return sdp_get_string_attr(rec, SDP_ATTR_PROVNAME_PRIMARY, str, len);
}

static inline int sdp_get_doc_url(const sdp_record_t *rec, char *str, int len)
{
	return sdp_get_string_attr(rec, SDP_ATTR_DOC_URL, str, len);
}

static inline int sdp_get_clnt_exec_url(const sdp_record_t *rec, char *str, int len)
{
	return sdp_get_string_attr(rec, SDP_ATTR_CLNT_EXEC_URL, str, len);
}

static inline int sdp_get_icon_url(const sdp_record_t *rec, char *str, int len)
{
	return sdp_get_string_attr(rec, SDP_ATTR_ICON_URL, str, len);
}

int sdp_set_supp_feat(sdp_record_t *rec, const sdp_list_t *sf);

int sdp_get_supp_feat(const sdp_record_t *rec, sdp_list_t **seqp);

sdp_record_t *sdp_extract_pdu(const uint8_t *pdata, int bufsize, int *scanned);
sdp_record_t *sdp_copy_record(sdp_record_t *rec);

void sdp_data_print(sdp_data_t *data);
void sdp_print_service_attr(sdp_list_t *alist);

int sdp_attrid_comp_func(const void *key1, const void *key2);

void sdp_set_seq_len(uint8_t *ptr, uint32_t length);
void sdp_set_attrid(sdp_buf_t *pdu, uint16_t id);
void sdp_append_to_pdu(sdp_buf_t *dst, sdp_data_t *d);
void sdp_append_to_buf(sdp_buf_t *dst, uint8_t *data, uint32_t len);

int sdp_gen_pdu(sdp_buf_t *pdu, sdp_data_t *data);
int sdp_gen_record_pdu(const sdp_record_t *rec, sdp_buf_t *pdu);

int sdp_extract_seqtype(const uint8_t *buf, int bufsize, uint8_t *dtdp, int *size);

sdp_data_t *sdp_extract_attr(const uint8_t *pdata, int bufsize, int *extractedLength, sdp_record_t *rec);

void sdp_pattern_add_uuid(sdp_record_t *rec, uuid_t *uuid);
void sdp_pattern_add_uuidseq(sdp_record_t *rec, sdp_list_t *seq);

int sdp_send_req_w4_rsp(sdp_session_t *session, uint8_t *req, uint8_t *rsp, uint32_t reqsize, uint32_t *rspsize);

#ifdef __cplusplus
}
#endif

#endif /* __SDP_LIB_H */
