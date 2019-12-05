#!/bin/bash
# run from /src directory
rm -rf sitl
cp -r ../transpiled/sitl .
./replace.sh sitl
cd sitl && cargo build && cd ..