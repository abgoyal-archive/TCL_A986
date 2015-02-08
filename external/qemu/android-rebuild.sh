#!/bin/bash
cd `dirname $0`
./android-configure.sh $* && \
make clean && \
make -j4 && \
echo "Done. !!"
