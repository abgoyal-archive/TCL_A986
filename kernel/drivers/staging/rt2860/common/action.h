

#ifndef	__ACTION_H__
#define	__ACTION_H__

struct PACKED rt_ht_information_octet {
	u8 Request:1;
	u8 Forty_MHz_Intolerant:1;
	u8 STA_Channel_Width:1;
	u8 Reserved:5;
};

struct PACKED rt_frame_ht_info {
	struct rt_header_802_11 Hdr;
	u8 Category;
	u8 Action;
	struct rt_ht_information_octet HT_Info;
};

#endif /* __ACTION_H__ */
