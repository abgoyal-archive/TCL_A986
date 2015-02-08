#!/bin/sh
PROGUARD_HOME=`dirname "$0"`
PROGUARD_HOME=`dirname "$PROGUARD_HOME"`

java -jar $PROGUARD_HOME/lib/retrace.jar "$@"
