
// Derived from:
// mozilla/netwerk/protocol/http/src/nsHttpChunkedDecoder.h

#ifndef NET_HTTP_HTTP_CHUNKED_DECODER_H_
#define NET_HTTP_HTTP_CHUNKED_DECODER_H_

#include <string>

namespace net {

// From RFC2617 section 3.6.1, the chunked transfer coding is defined as:
//
//   Chunked-Body    = *chunk
//                     last-chunk
//                     trailer
//                     CRLF
//   chunk           = chunk-size [ chunk-extension ] CRLF
//                     chunk-data CRLF
//   chunk-size      = 1*HEX
//   last-chunk      = 1*("0") [ chunk-extension ] CRLF
//
//   chunk-extension = *( ";" chunk-ext-name [ "=" chunk-ext-val ] )
//   chunk-ext-name  = token
//   chunk-ext-val   = token | quoted-string
//   chunk-data      = chunk-size(OCTET)
//   trailer         = *(entity-header CRLF)
//
// The chunk-size field is a string of hex digits indicating the size of the
// chunk.  The chunked encoding is ended by any chunk whose size is zero,
// followed by the trailer, which is terminated by an empty line.
//
// NOTE: This implementation does not bother to parse trailers since they are
// not used on the web.
//
class HttpChunkedDecoder {
 public:
  HttpChunkedDecoder();

  // Indicates that a previous call to FilterBuf encountered the final CRLF.
  bool reached_eof() const { return reached_eof_; }

  // Returns the number of bytes after the final CRLF.
  int bytes_after_eof() const { return bytes_after_eof_; }

  // Called to filter out the chunk markers from buf and to check for end-of-
  // file.  This method modifies |buf| inline if necessary to remove chunk
  // markers.  The return value indicates the final size of decoded data stored
  // in |buf|.  Call reached_eof() after this method to check if end-of-file
  // was encountered.
  int FilterBuf(char* buf, int buf_len);

 private:
  // Scans |buf| for the next chunk delimiter.  This method returns the number
  // of bytes consumed from |buf|.  If found, |chunk_remaining_| holds the
  // value for the next chunk size.
  int ScanForChunkRemaining(const char* buf, int buf_len);

  // Converts string |start| of length |len| to a numeric value.
  // |start| is a string of type "chunk-size" (hex string).
  // If the conversion succeeds, returns true and places the result in |out|.
  static bool ParseChunkSize(const char* start, int len, int* out);

  // Indicates the number of bytes remaining for the current chunk.
  int chunk_remaining_;

  // A small buffer used to store a partial chunk marker.
  std::string line_buf_;

  // True if waiting for the terminal CRLF of a chunk's data.
  bool chunk_terminator_remaining_;

  // Set to true when FilterBuf encounters the last-chunk.
  bool reached_last_chunk_;

  // Set to true when FilterBuf encounters the final CRLF.
  bool reached_eof_;

  // The number of unfiltered bytes after the final CRLF, either extraneous
  // data or the first part of the next response in a pipelined stream.
  int bytes_after_eof_;
};

}  // namespace net

#endif  // NET_HTTP_HTTP_CHUNKED_DECODER_H_
