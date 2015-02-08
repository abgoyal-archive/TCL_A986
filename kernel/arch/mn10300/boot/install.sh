#!/bin/sh
rm -fr $4/../usr/include/linux $4/../usr/include/asm
install -c -m 0755 $2 $4/vmlinuz
install -c -m 0755 $5 $4/boot.rom
install -c -m 0755 -d $4/../usr/include/linux
cd ${srctree}/include/linux
for i in `find . -maxdepth 1 -name '*.h' -print`; do
  install -c -m 0644 $i $4/../usr/include/linux
done
install -c -m 0755 -d $4/../usr/include/linux/byteorder
cd ${srctree}/include/linux/byteorder
for i in `find . -name '*.h' -print`; do
  install -c -m 0644 $i $4/../usr/include/linux/byteorder
done
install -c -m 0755 -d $4/../usr/include/linux/lockd
cd ${srctree}/include/linux/lockd
for i in `find . -name '*.h' -print`; do
  install -c -m 0644 $i $4/../usr/include/linux/lockd
done
install -c -m 0755 -d $4/../usr/include/linux/netfilter_ipv4
cd ${srctree}/include/linux/netfilter_ipv4
for i in `find . -name '*.h' -print`; do
  install -c -m 0644 $i $4/../usr/include/linux/netfilter_ipv4
done
install -c -m 0755 -d $4/../usr/include/linux/nfsd
cd ${srctree}/include/linux/nfsd
for i in `find . -name '*.h' -print`; do
  install -c -m 0644 $i $4/../usr/include/linux/nfsd/$i
done
install -c -m 0755 -d $4/../usr/include/linux/raid
cd ${srctree}/include/linux/raid
for i in `find . -name '*.h' -print`; do
  install -c -m 0644 $i $4/../usr/include/linux/raid
done
install -c -m 0755 -d $4/../usr/include/linux/sunrpc
cd ${srctree}/include/linux/sunrpc
for i in `find . -name '*.h' -print`; do
  install -c -m 0644 $i $4/../usr/include/linux/sunrpc
done
install -c -m 0755 -d $4/../usr/include/asm
cd ${srctree}/include/asm
for i in `find . -name '*.h' -print`; do
  install -c -m 0644 $i $4/../usr/include/asm
done
