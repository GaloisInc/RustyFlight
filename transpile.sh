# automated build script
#
# Kai Brooks
# github.com/kaibrooks

# builds c code, adjusts the compile_commands.json to prep for c2rust conversion,
# and translates the code through c2rust, for a specific targets
#
# use: RustyFlight% ./magic.sh TARGET
# TARGET is SITL, AFROMINI, SPRACINGF3, etc

# kill this thing if we don't have a target
if [ $# -lt 1 ]; then
  echo 1>&2 "$0: Please specify TARGET parameter"
  exit 2
fi

#remove previous build and transpiled artifacts
rm -rf cleanflight/obj/main/$1
rm -rf transpiled/$1

cd cleanflight

intercept-build make TARGET=$1

sed -i -e '/save-temps=obj/d' compile_commands.json
sed -i -e '/fuse-linker-plugin/d' compile_commands.json
sed -i -e '/Wunsafe-loop-optimizations/d' compile_commands.json
sed -i -e 's/"-c"/"-c", "-fblocks"/g' compile_commands.json

cd ..

c2rust transpile --emit-no-std \
                 --emit-build-files \
                 --binary main \
                 -o transpiled/$1 \
                 cleanflight/compile_commands.json 2>&1 | tee c2rust-logs/$1_log_$(date +"%F_%H%M%S").txt
