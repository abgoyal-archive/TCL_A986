#!/bin/sh
output=$1
shift

if test -z "$output" || test -z "$1"; then
  echo "Usage: $0 OUTPUTFILE INPUTFILE..."
  exit 1
fi

if test -e "$output"; then
  echo "Output file \"$output\" already exists; refusing to overwrite."
  exit 1
fi

for input; do
  arrayname=xml_feature_`echo $input | sed 's,.*/,,; s/[-.]/_/g'`

  ${AWK:-awk} 'BEGIN { n = 0
      print "static const char '$arrayname'[] = {"
      for (i = 0; i < 255; i++)
        _ord_[sprintf("%c", i)] = i
    } {
      split($0, line, "");
      printf "  "
      for (i = 1; i <= length($0); i++) {
        c = line[i]
        if (c == "'\''") {
          printf "'\''\\'\'''\'', "
        } else if (c == "\\") {
          printf "'\''\\\\'\'', "
        } else if (_ord_[c] >= 32 && _ord_[c] < 127) {
	  printf "'\''%s'\'', ", c
        } else {
          printf "'\''\\%03o'\'', ", _ord_[c]
        }
        if (i % 10 == 0)
          printf "\n   "
      }
      printf "'\''\\n'\'', \n"
    } END {
      print "  0 };"
    }' < $input >> $output
done

echo >> $output
echo "extern const char *const xml_builtin[][2];" >> $output
echo "const char *const xml_builtin[][2] = {" >> $output

for input; do
  basename=`echo $input | sed 's,.*/,,'`
  arrayname=xml_feature_`echo $input | sed 's,.*/,,; s/[-.]/_/g'`
  echo "  { \"$basename\", $arrayname }," >> $output
done

echo "  { (char *)0, (char *)0 }" >> $output
echo "};" >> $output
