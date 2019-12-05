#!/bin/bash
KEYWORDS="SystemCoreClock colors SystemCoreClockbeeper"

for keyword in ${KEYWORDS}; do
    echo "Removing ${keyword}"
    grep -lr "${keyword}" $1 | while read -r line ; do
        file=`basename $line`

#        if [ $file == "cli.rs" ] || [ $file == "target.rs" ]
#        then
#        echo "Skipping $line"
        echo "Processing $file"
        tmp=${file%.*}
        perl -0777 -i -pe 's/^pub static mut '${keyword}'/\/\/pub static mut '${keyword}'/igs' $line
#        else
#        echo "Processing $line"
#        fi
    done
done

#echo "Removing colors"
#grep -lr "pub static mut colors" src/src/* | while read -r line ; do
#    file=`basename $line`
#    tmp=${file%.*}
#    echo "Processing $line"
#    perl -0777 -i -pe 's/pub static mut colors/pub static mut colors'${tmp}'/igs' $line
#done
