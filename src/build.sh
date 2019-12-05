#!/bin/bash
rm -rf sitl
cp -r ../transpiled/sitl .
./replace.sh sitl
cd sitl && cargo build && cd ..