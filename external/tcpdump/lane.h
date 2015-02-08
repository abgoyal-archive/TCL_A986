

/* $Id: lane.h,v 1.7 2002/12/11 07:13:54 guy Exp $ */

#ifndef ETHER_ADDR_LEN
#define ETHER_ADDR_LEN 6
#endif

struct lecdatahdr_8023 {
  u_int16_t le_header;
  u_int8_t h_dest[ETHER_ADDR_LEN];
  u_int8_t h_source[ETHER_ADDR_LEN];
  u_int16_t h_type;
};

struct lane_controlhdr {
  u_int16_t lec_header;
  u_int8_t lec_proto;
  u_int8_t lec_vers;
  u_int16_t lec_opcode;
};
