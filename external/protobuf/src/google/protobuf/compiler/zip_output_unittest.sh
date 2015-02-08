#!/bin/sh
fail() {
  echo "$@" >&2
  exit 1
}

echo '
  option java_multiple_files = true;
  option java_package = "test.jar";
  option java_outer_classname = "Outer";
  message Foo {}
  message Bar {}
' > testzip.proto

./protoc --cpp_out=testzip.zip --python_out=testzip.zip --java_out=testzip.jar \
    testzip.proto || fail 'protoc failed.'

echo "Testing output to zip..."
if unzip -h > /dev/null; then
  unzip -t testzip.zip > testzip.list || fail 'unzip failed.'

  grep 'testing: testzip\.pb\.cc *OK$' testzip.list > /dev/null \
    || fail 'testzip.pb.cc not found in output zip.'
  grep 'testing: testzip\.pb\.h *OK$' testzip.list > /dev/null \
    || fail 'testzip.pb.h not found in output zip.'
  grep 'testing: testzip_pb2\.py *OK$' testzip.list > /dev/null \
    || fail 'testzip_pb2.py not found in output zip.'
  grep -i 'manifest' testzip.list > /dev/null \
    && fail 'Zip file contained manifest.'
else
  echo "Warning:  'unzip' command not available.  Skipping test."
fi

echo "Testing output to jar..."
if jar c testzip.proto > /dev/null; then
  jar tf testzip.jar > testzip.list || fail 'jar failed.'

  grep '^test/jar/Foo\.java$' testzip.list > /dev/null \
    || fail 'Foo.java not found in output jar.'
  grep '^test/jar/Bar\.java$' testzip.list > /dev/null \
    || fail 'Bar.java not found in output jar.'
  grep '^test/jar/Outer\.java$' testzip.list > /dev/null \
    || fail 'Outer.java not found in output jar.'
  grep '^META-INF/MANIFEST\.MF$' testzip.list > /dev/null \
    || fail 'Manifest not ofund in output jar.'
else
  echo "Warning:  'jar' command not available.  Skipping test."
fi

echo PASS
