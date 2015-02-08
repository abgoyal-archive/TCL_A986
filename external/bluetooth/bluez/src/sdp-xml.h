

#ifndef __SDP_XML_H
#define __SDP_XML_H

#include <bluetooth/sdp.h>

#define SDP_XML_ENCODING_NORMAL	0
#define SDP_XML_ENCODING_HEX	1

void convert_sdp_record_to_xml(sdp_record_t *rec,
		void *user_data, void (*append_func) (void *, const char *));

sdp_data_t *sdp_xml_parse_nil(const char *data);
sdp_data_t *sdp_xml_parse_text(const char *data, char encoding);
sdp_data_t *sdp_xml_parse_url(const char *data);
sdp_data_t *sdp_xml_parse_int(const char *data, uint8_t dtd);
sdp_data_t *sdp_xml_parse_uuid(const char *data, sdp_record_t *record);

struct sdp_xml_data {
	char *text;			/* Pointer to the current buffer */
	int size;			/* Size of the current buffer */
	sdp_data_t *data;		/* The current item being built */
	struct sdp_xml_data *next;	/* Next item on the stack */
	char type;			/* 0 = Text or Hexadecimal */
	char *name;			/* Name, optional in the dtd */
	/* TODO: What is it used for? */
};

struct sdp_xml_data *sdp_xml_data_alloc();
void sdp_xml_data_free(struct sdp_xml_data *elem);
struct sdp_xml_data *sdp_xml_data_expand(struct sdp_xml_data *elem);

sdp_data_t *sdp_xml_parse_datatype(const char *el, struct sdp_xml_data *elem,
							sdp_record_t *record);

#endif /* __SDP_XML_H */
