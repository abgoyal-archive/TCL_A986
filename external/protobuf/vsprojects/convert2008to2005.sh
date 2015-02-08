#! /bin/sh -e
for file in *.sln; do
  echo "downgrading $file..."
  sed -i -re 's/Format Version 10.00/Format Version 9.00/g;
              s/Visual Studio 2008/Visual Studio 2005/g;' $file
done

for file in *.vcproj; do
  echo "downgrading $file..."
  sed -i -re 's/Version="9.00"/Version="8.00"/g;' $file
done

# Yes, really, that's it.
