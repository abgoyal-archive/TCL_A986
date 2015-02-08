

#ifndef __MUSB_GADGET_H
#define __MUSB_GADGET_H

struct musb_request {
	struct usb_request	request;
	struct musb_ep		*ep;
	struct musb		*musb;
	u8 tx;			/* endpoint direction */
	u8 epnum;
	u8 mapped;
};

static inline struct musb_request *to_musb_request(struct usb_request *req)
{
	return req ? container_of(req, struct musb_request, request) : NULL;
}

extern struct usb_request *
musb_alloc_request(struct usb_ep *ep, gfp_t gfp_flags);
extern void musb_free_request(struct usb_ep *ep, struct usb_request *req);


struct musb_ep {
	/* stuff towards the head is basically write-once. */
	struct usb_ep			end_point;
	char				name[12];
	struct musb_hw_ep		*hw_ep;
	struct musb			*musb;
	u8				current_epnum;

	/* ... when enabled/disabled ... */
	u8				type;
	u8				is_in;
	u16				packet_sz;
	const struct usb_endpoint_descriptor	*desc;
	struct dma_channel		*dma;

	/* later things are modified based on usage */
	struct list_head		req_list;

	u8				wedged;

	/* true if lock must be dropped but req_list may not be advanced */
	u8				busy;
};

static inline struct musb_ep *to_musb_ep(struct usb_ep *ep)
{
	return ep ? container_of(ep, struct musb_ep, end_point) : NULL;
}

static inline struct usb_request *next_request(struct musb_ep *ep)
{
	struct list_head	*queue = &ep->req_list;

	if (list_empty(queue))
		return NULL;
	return container_of(queue->next, struct usb_request, list);
}

extern void musb_g_tx(struct musb *musb, u8 epnum);
extern void musb_g_rx(struct musb *musb, u8 epnum);

extern const struct usb_ep_ops musb_g_ep0_ops;

extern int musb_gadget_setup(struct musb *);
extern void musb_gadget_cleanup(struct musb *);

extern void musb_g_giveback(struct musb_ep *, struct usb_request *, int);

#endif		/* __MUSB_GADGET_H */
