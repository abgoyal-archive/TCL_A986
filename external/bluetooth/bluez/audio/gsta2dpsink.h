
#ifndef __GST_A2DP_SINK_H__
#define __GST_A2DP_SINK_H__

#include <gst/gst.h>
#include <gst/rtp/gstbasertppayload.h>
#include "gstavdtpsink.h"

G_BEGIN_DECLS

#define GST_TYPE_A2DP_SINK \
	(gst_a2dp_sink_get_type())
#define GST_A2DP_SINK(obj) \
	(G_TYPE_CHECK_INSTANCE_CAST((obj),GST_TYPE_A2DP_SINK,GstA2dpSink))
#define GST_A2DP_SINK_CLASS(klass) \
	(G_TYPE_CHECK_CLASS_CAST((klass),GST_TYPE_A2DP_SINK,GstA2dpSinkClass))
#define GST_IS_A2DP_SINK(obj) \
	(G_TYPE_CHECK_INSTANCE_TYPE((obj),GST_TYPE_A2DP_SINK))
#define GST_IS_A2DP_SINK_CLASS(obj) \
	(G_TYPE_CHECK_CLASS_TYPE((klass),GST_TYPE_A2DP_SINK))

typedef struct _GstA2dpSink GstA2dpSink;
typedef struct _GstA2dpSinkClass GstA2dpSinkClass;

struct _GstA2dpSink {
	GstBin bin;

	GstBaseRTPPayload *rtp;
	GstAvdtpSink *sink;
	GstElement *capsfilter;
	GstElement *fakesink;

	gchar *device;
	gboolean autoconnect;
	gboolean sink_is_in_bin;

	GstGhostPad *ghostpad;
	GstPadSetCapsFunction ghostpad_setcapsfunc;
	GstPadEventFunction ghostpad_eventfunc;

	GstEvent *newseg_event;
	/* Store the tags received before the a2dpsender sink is created
	 * when it is created we forward this to it */
	GstTagList *taglist;

	GMutex *cb_mutex;
};

struct _GstA2dpSinkClass {
	GstBinClass parent_class;
};

//GType gst_a2dp_sink_get_type(void);

gboolean gst_a2dp_sink_plugin_init (GstPlugin * plugin);

GstCaps *gst_a2dp_sink_get_device_caps(GstA2dpSink *self);

G_END_DECLS

#endif

