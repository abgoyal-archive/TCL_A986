
#ifndef _android_gsm_h
#define _android_gsm_h


typedef unsigned char  byte_t;
typedef byte_t*        bytes_t;
typedef const byte_t*  cbytes_t;


/* convert a 8-bit value into the corresponding nibble-bcd byte */
extern byte_t   gsm_int_to_bcdi( int  value );

/* convert a nibble-bcd byte into an int, invalid nibbles are silently converted to 0 */
extern int      gsm_int_from_bcdi( byte_t  value );


extern int      gsm_hex_to_bytes  ( cbytes_t  hex, int  hexlen, bytes_t  dst );

extern void     gsm_hex_to_bytes0 ( cbytes_t  hex, int  hexlen, bytes_t  dst );

/* convert a byte string into a hex string, assumes 'hex' is properly sized */
extern void     gsm_hex_from_bytes( char*  hex, cbytes_t  src, int  srclen );

/* convert a hexchar to an int, returns -1 on error */
extern int      gsm_hexchar_to_int( char  c );

/* convert a hexchar to an int, returns 0 on error */
extern int      gsm_hexchar_to_int0( char  c );

/* convert a 2-char hex value into an int, returns -1 on error */
extern int      gsm_hex2_to_byte( const char*  hex );

/* convert a 2-char hex value into an int, returns 0 on error */
extern int      gsm_hex2_to_byte0( const char*  hex );

/* convert a 4-char hex value into an int, returns -1 on error */
extern int      gsm_hex4_to_short( const char*  hex );

/* convert a 4-char hex value into an int, returns 0 on error */
extern int      gsm_hex4_to_short0( const char*  hex );

/* write a byte to a 2-byte hex string */
extern void     gsm_hex_from_byte( char*  hex, int  val );

extern void     gsm_hex_from_short( char*  hex, int  val );


/* check that a given utf8 string is well-formed, returns 1 on success, 0 otherwise */
extern int      utf8_check( cbytes_t  utf8, int  utf8len );

extern int      utf8_check_gsm7( cbytes_t  utf8, int  utf8len );

/* try to skip enough utf8 characters to generate gsm7len GSM septets */
extern cbytes_t utf8_skip_gsm7( cbytes_t  utf8, cbytes_t  utf8end, int  gsm7len );

extern int      utf8_to_gsm7( cbytes_t  utf8, int  utf8len, bytes_t  dst, int  offset );

extern int      utf8_to_gsm8( cbytes_t  utf8, int  utf8len, bytes_t  dst );

extern int      utf8_from_gsm7( cbytes_t  src, int  offset, int  count, bytes_t  utf8 );

extern int      utf8_from_gsm8( cbytes_t  src, int  count, bytes_t  utf8 );



extern int      ucs2_check_gsm7( cbytes_t  ucs2, int  ucs2len );

extern int      ucs2_to_gsm7( cbytes_t  ucs2, int  ucs2len, bytes_t  dst, int  offset );

extern int      ucs2_to_gsm8( cbytes_t  ucs2, int  ucs2len, bytes_t  dst );

extern int      ucs2_from_gsm7( bytes_t   ucs2, cbytes_t  src, int  offset, int  count );

extern int      ucs2_from_gsm8( bytes_t   ucs2, cbytes_t  src, int  count );



extern int      ucs2_to_utf8( cbytes_t  ucs2, int  ucs2len, bytes_t  utf8 );

extern int      utf8_to_ucs2( cbytes_t  utf8, int  utf8len, bytes_t  ucs2 );

/* try to skip a given number of characters in a utf-8 byte string, return new position */
extern cbytes_t  utf8_skip( cbytes_t   utf8, cbytes_t   utf8end, int  count);


extern int  gsm_bcdnum_to_ascii  ( cbytes_t  bcd, int  num_digits, bytes_t  dst );

/* convert an ASCII dial-number into a bcd-coded string, returns the number of 4-bit nibbles written, */
extern int  gsm_bcdnum_from_ascii( cbytes_t  ascii, int  asciilen, bytes_t  dst );

#define  SIM_ADN_MAX_ALPHA        20  /* maximum number of characters in ADN alpha tag */
#define  SIM_ADN_MAX_NUMBER       20  /* maximum digits in ADN number */

typedef struct {
    byte_t  alpha [ SIM_ADN_MAX_ALPHA*3+1 ];  /* alpha tag in zero-terminated utf-8      */
    char    number[ SIM_ADN_MAX_NUMBER+1 ];   /* dialing number in zero-terminated ASCII */
}
SimAdnRec, *SimAdn;

typedef struct {
    SimAdnRec       adn;
    byte_t          ext_record;  /* 0 or 0xFF means no extension */
}
SimAdnRecordRec, *SimAdnRecord;

extern int  sim_adn_record_from_bytes( SimAdnRecord  rec, cbytes_t  data, int  datalen );
extern int  sim_adn_record_to_bytes  ( SimAdnRecord  rec, bytes_t   data, int  datalen );


typedef struct {
    bytes_t         data;
    int             max;
    int             pos;
    int             error;
    unsigned char   data0[16];
} GsmRopeRec, *GsmRope;

extern void      gsm_rope_init( GsmRope  rope );
extern void      gsm_rope_init_alloc( GsmRope  rope, int  alloc );
extern int       gsm_rope_done( GsmRope  rope );
extern bytes_t   gsm_rope_done_acquire( GsmRope  rope, int  *psize );
extern void      gsm_rope_add_c( GsmRope  rope, char  c );
extern void      gsm_rope_add( GsmRope  rope, const void*  str, int  len );
extern void*     gsm_rope_reserve( GsmRope  rope, int  len );

#endif /* _android_gsm_h */
