#!/bin/bash
KEYWORDS="SystemCoreClock colors SystemCoreClockbeeper inputSource_e pCurrentDisplay switchStates"

for keyword in ${KEYWORDS}; do
    echo "Removing ${keyword}"
    grep -lr "${keyword}" $1 | while read -r line ; do
        echo "Processing $line"
        new_file=`echo $line | perl -pe 's/\//_/igs' | perl -pe 's/\..//igs'`
        tmp=${new_file%.*}
        perl -0777 -i -pe 's/'${keyword}'/'${keyword}${tmp}'/igs' $line
    done
done

# speciality replacement for main
main_old="::std::process::exit(main_0() as i32)"
main_new="main_0\(\);"
main_file=`grep -lr "${main_old}" $1`
echo "MAIN $main_file"
perl -0777 -i -pe 's/::std::process::exit\(main_0\(\) as i32\)/'${main_new}'/igs' $main_file

main_file=`grep -lr "1 => { IOConfigGPIO" $1`
echo "GPIO $main_file"
perl -i -pe 's/1 => \{ IOConfigGPIO/\/\/1 => \{ IOConfigGPIO/igs' $main_file

main_file=`grep -rl "static mut serialPinConfig_System" $1`
echo "ConfigSystem $main_file"
perl -0777 -i -pe 's/#\[inline]\nunsafe extern \"C\" fn serialPinConfig\(\) -> \*const serialPinConfig_t \{\n    return &mut serialPinConfig_System;\n\}//igs' $main_file
perl -0777 -i -pe 's/uartPinConfigure\(serialPinConfig\(\)\);/\/\/uartPinConfigure\(serialPinConfig\(\)\);/igs' $main_file

# include build config 
# modify if you are building on OS X or Windows,
# check https://forge.rust-lang.org/release/platform-support.html
mkdir $1/.cargo
cp _cargo/config $1/.cargo/.
cp _cargo/pg.ld $1/.
cp _cargo/eeprom.bin $1/.