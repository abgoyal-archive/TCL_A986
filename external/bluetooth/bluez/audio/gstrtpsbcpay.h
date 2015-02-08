
#include <gst/gst.h>
#include <gst/rtp/gstbasertppayload.h>
#include <gst/base/gstadapter.h>
#include <gst/rtp/gstrtpbuffer.h>

G_BEGIN_DECLS

#define GST_TYPE_RTP_SBC_PAY \
	(gst_rtp_sbc_pay_get_type())
#define GST_RTP_SBC_PAY(obj) \
	(G_TYPE_CHECK_INSTANCE_CAST((obj),GST_TYPE_RTP_SBC_PAY,\
		GstRtpSBCPay))
#define GST_RTP_SBC_PAY_CLASS(klass) \
	(G_TYPE_CHECK_CLASS_CAST((klass),GST_TYPE_RTP_SBC_PAY,\
		GstRtpSBCPayClass))
#define GST_IS_RTP_SBC_PAY(obj) \
	(G_TYPE_CHECK_INSTANCE_TYPE((obj),GST_TYPE_RTP_SBC_PAY))
#define GST_IS_RTP_SBC_PAY_CLASS(obj) \
	(G_TYPE_CHECK_CLASS_TYPE((klass),GST_TYPE_RTP_SBC_PAY))

typedef struct _GstRtpSBCPay GstRtpSBCPay;
typedef struct _GstRtpSBCPayClass GstRtpSBCPayClass;

struct _GstRtpSBCPay {
	GstBaseRTPPayload base;

	GstAdapter *adapter;
	GstClockTime timestamp;

	guint frame_length;

	guint min_frames;
};

struct _GstRtpSBCPayClass {
	GstBaseRTPPayloadClass parent_class;
};

//GType gst_rtp_sbc_pay_get_type(void);

gboolean gst_rtp_sbc_pay_plugin_init (GstPlugin * plugin);

G_END_DECLS
