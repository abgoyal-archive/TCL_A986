#!/bin/sh
if test ! -e src/google/protobuf/stubs/common.h; then
  cat >&2 << __EOF__
Could not find source code.  Make sure you are running this script from the
root of the distribution tree.
__EOF__
  exit 1
fi

if test ! -e src/Makefile; then
  cat >&2 << __EOF__
Could not find src/Makefile.  You must run ./configure (and perhaps
./autogen.sh) first.
__EOF__
  exit 1
fi

cd src
make $@ protoc &&
  ./protoc --cpp_out=dllexport_decl=LIBPROTOBUF_EXPORT:. google/protobuf/descriptor.proto && \
  ./protoc --cpp_out=dllexport_decl=LIBPROTOC_EXPORT:. google/protobuf/compiler/plugin.proto
cd ..
