#! /bin/bash
#shell command to build a plugin.

rm -r build
mkdir build
cd build
cmake ../
make -j4
cd ..
