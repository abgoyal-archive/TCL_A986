#!/bin/sh
set -e

if [ $# -ne 3 ] ; then
  echo "usage: ${0} INPUT OUTPUT WORK_DIR" >& 2
  exit 1
fi

INPUT="${1}"
OUTPUT="${2}"
WORK_DIR="${3}"

# Start with a clean slate.
rm -f "${OUTPUT}"
rm -rf "${WORK_DIR}"
mkdir -p "${WORK_DIR}"

# ar doesn't operate on fat files.  Figure out what architectures are
# involved.
ARCHS=$(file "${INPUT}" | sed -Ene 's/^.*\(for architecture (.+)\):.*$/\1/p')
if [ -z "${ARCHS}" ] ; then
  ARCHS=self
fi

OUTPUT_NAME="output.a"

for ARCH in ${ARCHS} ; do
  # Get a thin version of fat input by running lipo.  If the input is already
  # thin, just copy it into place.  The extra copy isn't strictly necessary
  # but it simplifies the script.
  ARCH_DIR="${WORK_DIR}/${ARCH}"
  mkdir -p "${ARCH_DIR}"
  INPUT_NAME=input.a
  ARCH_INPUT="${ARCH_DIR}/${INPUT_NAME}"
  if [ "${ARCHS}" = "self" ] ; then
    cp "${INPUT}" "${ARCH_INPUT}"
  else
    lipo -thin "${ARCH}" "${INPUT}" -output "${ARCH_INPUT}"
  fi

  # Change directories to extract the archive to ensure correct pathnames.
  (cd "${ARCH_DIR}" && ar -x "${INPUT_NAME}")

  # Use ld -r to relink each object that was in the archive.  Providing an
  # empty -exported_symbols_list will transform all symbols to private_extern;
  # these symbols are retained with -keep_private_externs.
  for OBJECT in "${ARCH_DIR}/"*.o ; do
    NEW_OBJECT="${OBJECT}.new"
    ld -o "${NEW_OBJECT}" -r "${OBJECT}" \
        -exported_symbols_list /dev/null -keep_private_externs
    mv "${NEW_OBJECT}" "${OBJECT}"
  done

  # Build an architecture-specific archive from the modified object files.
  ARCH_OUTPUT="${ARCH_DIR}/${OUTPUT_NAME}"
  (cd "${ARCH_DIR}" && ar -rc "${OUTPUT_NAME}" *.o)
  ranlib "${ARCH_OUTPUT}"

  # Toss the object files out now that they're in the archive.
  rm -f "${ARCH_DIR}/"*.o
done

# Create a fat archive from the architecture-specific archives if needed.
# If the input was thin, leave the output thin by copying the only output
# archive to the destination.
if [ "${ARCHS}" = "self" ] ; then
  cp "${WORK_DIR}/self/${OUTPUT_NAME}" "${OUTPUT}"
else
  lipo -create -output "${OUTPUT}" "${WORK_DIR}/"*"/${OUTPUT_NAME}"
fi
