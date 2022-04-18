#! /bin/bash
#shell command to build a plugin.

SCRIPTPATH="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CURR_DIR=$(pwd)

rm -r $SCRIPTPATH/build
mkdir $SCRIPTPATH/build
cd $SCRIPTPATH/build
cmake ../
make -j4
cd $CURR_DIR
protoc -I=$SCRIPTPATH/msgs --python_out=$SCRIPTPATH/../env/msgs/ $SCRIPTPATH/msgs/*.proto
