
/* @(#) $Header: /tcpdump/master/tcpdump/sctpHeader.h,v 1.6 2002/12/11 07:14:11 guy Exp $ (LBL) */



#ifndef __sctpHeader_h__
#define __sctpHeader_h__

#include <sctpConstants.h>

#ifdef	__cplusplus
extern "C" {
#endif

/* the sctp common header */

#ifdef TRU64
 #define _64BITS 1
#endif

struct sctpHeader{
  u_int16_t source;
  u_int16_t destination;
  u_int32_t verificationTag;
  u_int32_t adler32;
};

/* various descriptor parsers */

struct sctpChunkDesc{
  u_int8_t chunkID;
  u_int8_t chunkFlg;
  u_int16_t chunkLength;
};

struct sctpParamDesc{
  u_int16_t paramType;
  u_int16_t paramLength;
};


struct sctpRelChunkDesc{
  struct sctpChunkDesc chk;
  u_int32_t serialNumber;
};

struct sctpVendorSpecificParam {
  struct sctpParamDesc p;  /* type must be 0xfffe */
  u_int32_t vendorId;	   /* vendor ID from RFC 1700 */
  u_int16_t vendorSpecificType;
  u_int16_t vendorSpecificLen;
};


/* Structures for the control parts */



/* Sctp association init request/ack */

/* this is used for init ack, too */
struct sctpInitiation{
  u_int32_t initTag;		/* tag of mine */
  u_int32_t rcvWindowCredit;	/* rwnd */
  u_int16_t NumPreopenStreams;	/* OS */
  u_int16_t MaxInboundStreams;     /* MIS */
  u_int32_t initialTSN;
  /* optional param's follow in sctpParamDesc form */
};

struct sctpV4IpAddress{
  struct sctpParamDesc p;	/* type is set to SCTP_IPV4_PARAM_TYPE, len=10 */
  u_int32_t  ipAddress;
};


struct sctpV6IpAddress{
  struct sctpParamDesc p;	/* type is set to SCTP_IPV6_PARAM_TYPE, len=22 */
  u_int8_t  ipAddress[16];
};

struct sctpDNSName{
  struct sctpParamDesc param;
  u_int8_t name[1];
};


struct sctpCookiePreserve{
  struct sctpParamDesc p;	/* type is set to SCTP_COOKIE_PRESERVE, len=8 */
  u_int32_t extraTime;
};


struct sctpTimeStamp{
  u_int32_t ts_sec;
  u_int32_t ts_usec;
};

/* wire structure of my cookie */
struct cookieMessage{
  u_int32_t TieTag_curTag;		/* copied from assoc if present */
  u_int32_t TieTag_hisTag; 		/* copied from assoc if present */
  int32_t cookieLife;			/* life I will award this cookie */
  struct sctpTimeStamp timeEnteringState; /* the time I built cookie */
  struct sctpInitiation initAckISent;	/* the INIT-ACK that I sent to my peer */
  u_int32_t addressWhereISent[4];	/* I make this 4 ints so I get 128bits for future */
  int32_t addrtype;			/* address type */
  u_int16_t locScope;			/* V6 local scope flag */
  u_int16_t siteScope;			/* V6 site scope flag */
  /* at the end is tacked on the INIT chunk sent in
   * its entirety and of course our
   * signature.
   */
};


struct sctpUnifiedInit{
  struct sctpChunkDesc uh;
  struct sctpInitiation initm;
};

struct sctpSendableInit{
  struct sctpHeader mh;
  struct sctpUnifiedInit msg;
};



struct sctpSelectiveAck{
  u_int32_t highestConseqTSN;
  u_int32_t updatedRwnd;
  u_int16_t numberOfdesc;
  u_int16_t numDupTsns;
};

struct sctpSelectiveFrag{
  u_int16_t fragmentStart;
  u_int16_t fragmentEnd;
};


struct sctpUnifiedSack{
  struct sctpChunkDesc uh;
  struct sctpSelectiveAck sack;
};


struct sctpHBrequest {
  u_int32_t time_value_1;
  u_int32_t time_value_2;
};

/* here is what I read and respond with to. */
struct sctpHBunified{
  struct sctpChunkDesc hdr;
  struct sctpParamDesc hb;
};


/* here is what I send */
struct sctpHBsender{
  struct sctpChunkDesc hdr;
  struct sctpParamDesc hb;
  struct sctpHBrequest rtt;
  int8_t addrFmt[SCTP_ADDRMAX];
  u_int16_t userreq;
};



struct sctpUnifiedAbort{
  struct sctpChunkDesc uh;
};

struct sctpUnifiedAbortLight{
  struct sctpHeader mh;
  struct sctpChunkDesc uh;
};

struct sctpUnifiedAbortHeavy{
  struct sctpHeader mh;
  struct sctpChunkDesc uh;
  u_int16_t causeCode;
  u_int16_t causeLen;
};

struct sctpShutdown {
  u_int32_t TSN_Seen;
};

struct sctpUnifiedShutdown{
  struct sctpChunkDesc uh;
  struct sctpShutdown shut;
};

struct sctpOpErrorCause{
  u_int16_t cause;
  u_int16_t causeLen;
};

struct sctpUnifiedOpError{
  struct sctpChunkDesc uh;
  struct sctpOpErrorCause c;
};

struct sctpUnifiedStreamError{
  struct sctpHeader mh;
  struct sctpChunkDesc uh;
  struct sctpOpErrorCause c;
  u_int16_t strmNum;
  u_int16_t reserved;
};

struct staleCookieMsg{
  struct sctpHeader mh;
  struct sctpChunkDesc uh;
  struct sctpOpErrorCause c;
  u_int32_t moretime;
};


struct sctpUnifiedSingleMsg{
  struct sctpHeader mh;
  struct sctpChunkDesc uh;
};

struct sctpDataPart{
  u_int32_t TSN;
  u_int16_t streamId;
  u_int16_t sequence;
  u_int32_t payloadtype;
};

struct sctpUnifiedDatagram{
  struct sctpChunkDesc uh;
  struct sctpDataPart dp;
};

struct sctpECN_echo{
  struct sctpChunkDesc uh;
  u_int32_t Lowest_TSN;
};


struct sctpCWR{
  struct sctpChunkDesc uh;
  u_int32_t TSN_reduced_at;
};

#ifdef	__cplusplus
}
#endif

#endif
