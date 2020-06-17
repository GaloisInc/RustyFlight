#!/bin/bash
# run from /src directory
rm -rf spracingf3
cp -r ../transpiled/SPRACINGF3 spracingf3/

# include build config 
# modify if you are building on OS X or Windows,
# check https://forge.rust-lang.org/release/platform-support.html
TARGET=spracingf3
mkdir $TARGET/.cargo
cp _cargo_$TARGET/config $TARGET/.cargo/.
cp _cargo_$TARGET/*.ld $TARGET/.

cd spracingf3 && cargo build 2>&1 | tee build_log.txt
cd ..