
#include <gst/gst.h>

#include "sbc.h"

G_BEGIN_DECLS

#define GST_TYPE_SBC_PARSE \
	(gst_sbc_parse_get_type())
#define GST_SBC_PARSE(obj) \
	(G_TYPE_CHECK_INSTANCE_CAST((obj),GST_TYPE_SBC_PARSE,GstSbcParse))
#define GST_SBC_PARSE_CLASS(klass) \
	(G_TYPE_CHECK_CLASS_CAST((klass),GST_TYPE_SBC_PARSE,GstSbcParseClass))
#define GST_IS_SBC_PARSE(obj) \
	(G_TYPE_CHECK_INSTANCE_TYPE((obj),GST_TYPE_SBC_PARSE))
#define GST_IS_SBC_PARSE_CLASS(obj) \
	(G_TYPE_CHECK_CLASS_TYPE((klass),GST_TYPE_SBC_PARSE))

typedef struct _GstSbcParse GstSbcParse;
typedef struct _GstSbcParseClass GstSbcParseClass;

struct _GstSbcParse {
	GstElement element;

	GstPad *sinkpad;
	GstPad *srcpad;

	GstBuffer *buffer;

	sbc_t sbc;
	sbc_t new_sbc;
	GstCaps *outcaps;
	gboolean first_parsing;

	gint channels;
	gint rate;
};

struct _GstSbcParseClass {
	GstElementClass parent_class;
};

//GType gst_sbc_parse_get_type(void);

gboolean gst_sbc_parse_plugin_init(GstPlugin *plugin);

G_END_DECLS
