
#include <gst/gst.h>

#include "sbc.h"

G_BEGIN_DECLS

#define GST_TYPE_SBC_DEC \
	(gst_sbc_dec_get_type())
#define GST_SBC_DEC(obj) \
	(G_TYPE_CHECK_INSTANCE_CAST((obj),GST_TYPE_SBC_DEC,GstSbcDec))
#define GST_SBC_DEC_CLASS(klass) \
	(G_TYPE_CHECK_CLASS_CAST((klass),GST_TYPE_SBC_DEC,GstSbcDecClass))
#define GST_IS_SBC_DEC(obj) \
	(G_TYPE_CHECK_INSTANCE_TYPE((obj),GST_TYPE_SBC_DEC))
#define GST_IS_SBC_DEC_CLASS(obj) \
	(G_TYPE_CHECK_CLASS_TYPE((klass),GST_TYPE_SBC_DEC))

typedef struct _GstSbcDec GstSbcDec;
typedef struct _GstSbcDecClass GstSbcDecClass;

struct _GstSbcDec {
	GstElement element;

	GstPad *sinkpad;
	GstPad *srcpad;

	GstBuffer *buffer;

	/* caps for outgoing buffers */
	GstCaps *outcaps;

	sbc_t sbc;
};

struct _GstSbcDecClass {
	GstElementClass parent_class;
};

//GType gst_sbc_dec_get_type(void);

gboolean gst_sbc_dec_plugin_init(GstPlugin *plugin);

G_END_DECLS
