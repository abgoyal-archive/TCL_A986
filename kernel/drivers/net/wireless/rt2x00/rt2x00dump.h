


#ifndef RT2X00DUMP_H
#define RT2X00DUMP_H


enum rt2x00_dump_type {
	DUMP_FRAME_RXDONE = 1,
	DUMP_FRAME_TX = 2,
	DUMP_FRAME_TXDONE = 3,
	DUMP_FRAME_BEACON = 4,
};

struct rt2x00dump_hdr {
	__le32 version;
#define DUMP_HEADER_VERSION	2

	__le32 header_length;
	__le32 desc_length;
	__le32 data_length;

	__le16 chip_rt;
	__le16 chip_rf;
	__le32 chip_rev;

	__le16 type;
	__u8 queue_index;
	__u8 entry_index;

	__le32 timestamp_sec;
	__le32 timestamp_usec;
};

#endif /* RT2X00DUMP_H */
