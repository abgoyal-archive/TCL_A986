#!/bin/sh
OBJECT=`echo $1 | sed -e s/\\\\./\\\\\\\\./g`
#echo OBJECT=$OBJECT

OBJ_NAME=`basename $OBJECT`
#echo OBJ_NAME=$OBJ_NAME

# we replace $OBJ_NAME with $OBJECT only if $OBJ_NAME starts the line
# that's because some versions of GCC (e.g. 4.2.3) already produce
# a correct dependency line with the full path to the object file.
# In this case, we don't want to touch anything
#
cat $2 | sed -e s%^$OBJ_NAME%$OBJECT%g > $3 && rm -f $2



