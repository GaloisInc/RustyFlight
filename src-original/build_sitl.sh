#!/bin/bash
# run from /src directory
rm -rf sitl
cp -r ../transpiled/SITL sitl/
./replace.sh sitl

# include build config 
# modify if you are building on OS X or Windows,
# check https://forge.rust-lang.org/release/platform-support.html
TARGET=sitl
mkdir $TARGET/.cargo
cp _cargo_$TARGET/config $TARGET/.cargo/.
cp _cargo_$TARGET/*.ld $TARGET/.
cp _cargo_$TARGET/eeprom.bin $TARGET/.

cd sitl && cargo build && cd ..