#!/bin/bash

printnewprogress()
{
	cur=$1

	while :
	do
		a=`wc -l < estimateResults.log`
		b=`wc -l < params.log`
	
		p=`echo "scale=2; 100*$a/$b" | bc`
	
		if (( $(echo "$p > $cur" | bc -l) ))
		then
			cur=$p
			return 0
		fi
		sleep 0.001
	done
}

cur=0

while :
do
	printnewprogress $cur
	echo $cur
done

