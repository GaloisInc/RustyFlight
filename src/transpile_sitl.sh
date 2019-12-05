#!/bin/bash
# Must be run from root of the directory and from inside the docker container
cd cleanflight
intercept-build make TARGET=SITL
sed -i -e '/save-temps=obj/d' compile_commands.json
sed -i -e '/fuse-linker-plugin/d' compile_commands.json
sed -i -e '/Wunsafe-loop-optimizations/d' compile_commands.json
sed -i -e 's/"-c"/"-c", "-fblocks"/g' compile_commands.json
cd ..

c2rust transpile --emit-no-std             \
                 --emit-build-files        \
                 --binary main             \
                 -o transpiled/sitl             \
                 cleanflight/compile_commands.json