#!/bin/sh
PROGUARD_HOME=`dirname "$0"`
PROGUARD_HOME=`dirname "$PROGUARD_HOME"`

java -jar -Xms128m -Xmx512m $PROGUARD_HOME/lib/proguard.jar "$@"
