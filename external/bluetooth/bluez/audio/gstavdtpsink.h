
#ifndef __GST_AVDTP_SINK_H
#define __GST_AVDTP_SINK_H

#include <gst/gst.h>
#include <gst/base/gstbasesink.h>

G_BEGIN_DECLS

#define GST_TYPE_AVDTP_SINK \
	(gst_avdtp_sink_get_type())
#define GST_AVDTP_SINK(obj) \
	(G_TYPE_CHECK_INSTANCE_CAST((obj),GST_TYPE_AVDTP_SINK,\
		GstAvdtpSink))
#define GST_AVDTP_SINK_CLASS(klass) \
	(G_TYPE_CHECK_CLASS_CAST((klass),GST_TYPE_AVDTP_SINK,\
		GstAvdtpSinkClass))
#define GST_IS_AVDTP_SINK(obj) \
	(G_TYPE_CHECK_INSTANCE_TYPE((obj),GST_TYPE_AVDTP_SINK))
#define GST_IS_AVDTP_SINK_CLASS(obj) \
	(G_TYPE_CHECK_CLASS_TYPE((klass),GST_TYPE_AVDTP_SINK))

typedef struct _GstAvdtpSink GstAvdtpSink;
typedef struct _GstAvdtpSinkClass GstAvdtpSinkClass;

struct bluetooth_data;

struct _GstAvdtpSink {
	GstBaseSink sink;

	gchar *device;
	GIOChannel *stream;

	struct bluetooth_data *data;
	gboolean autoconnect;
	GIOChannel *server;

	/* mp3 stream data (outside caps data)*/
	gint mp3_using_crc;
	gint channel_mode;

	/* stream connection data */
	GstCaps *stream_caps;

	GstCaps *dev_caps;

	GMutex *sink_lock;

	guint watch_id;
};

struct _GstAvdtpSinkClass {
	GstBaseSinkClass parent_class;
};

//GType gst_avdtp_sink_get_type(void);

GstCaps *gst_avdtp_sink_get_device_caps(GstAvdtpSink *sink);
gboolean gst_avdtp_sink_set_device_caps(GstAvdtpSink *sink,
			GstCaps *caps);

guint gst_avdtp_sink_get_link_mtu(GstAvdtpSink *sink);

void gst_avdtp_sink_set_device(GstAvdtpSink *sink,
		const gchar* device);

gchar *gst_avdtp_sink_get_device(GstAvdtpSink *sink);

gboolean gst_avdtp_sink_plugin_init(GstPlugin *plugin);

void gst_avdtp_sink_set_crc(GstAvdtpSink *self, gboolean crc);

void gst_avdtp_sink_set_channel_mode(GstAvdtpSink *self,
			const gchar *mode);


G_END_DECLS

#endif /* __GST_AVDTP_SINK_H */
