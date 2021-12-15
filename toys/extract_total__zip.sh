
#!/bin/bash

lib=~/work/libraries/Nangate_45nm/NangateOpenCellLibrary.lef

for layer in 8 6 3
do
	for file in `ls *.def`
	do
		bench=${file%.*}
		log=$bench"_M"$layer".log"
		opens=$bench"_M"$layer"_open_pins.log"

		echo "Total # of open pins: " > $opens
		cat $log | grep "Location" | wc -l >> $opens

		zip "M"$layer".zip" $log $opens
	done
done
