#!/bin/bash
make noguiexe
mkdir -p ./data
rm -f ./data/*
for i in {0..14}; do
	for j in {1..15}; do
		temp=$(./bin/cs296_exe_13 $j)
		echo -n "$i,$j," >> ./data/lab05_g13_data.csv
		num=0;
		while read -r line; do
			num=$(($num+1));
			if [[ "$num" -gt "1" && "$num" -lt "4" ]]
			then
				echo $line | cut -d' ' -f6 | tr -d '\n' >> ./data/lab05_g13_data.csv
				echo -n "," >> ./data/lab05_g13_data.csv
			elif [ $num -gt 3 ] && (( "$num" < "6" ))
			then
				echo $line | cut -d' ' -f7 | tr -d '\n' >> ./data/lab05_g13_data.csv
				echo -n "," >> ./data/lab05_g13_data.csv
			elif [ "$num" -eq 7 ]
			then
				echo $line | cut -d' ' -f6 >> ./data/lab05_g13_data.csv
			fi;
		done <<< "$temp"
	done
done
