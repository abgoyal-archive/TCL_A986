#!/bin/sh -x
TESTFILE=Makefile

(./zcgzip < ${TESTFILE} | gunzip | cmp - ${TESTFILE}) && \
(gzip < ${TESTFILE} | ./zcgunzip | cmp - ${TESTFILE})

# Result of "(cmd) && (cmd)" implicitly becomes result of this script
# and thus the test.
