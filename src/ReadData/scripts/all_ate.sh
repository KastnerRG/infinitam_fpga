#!/bin/bash

ALGO=$1
DATASET_ROOT=$2
NUM_POSES=$3 # optional

echo ID,ate
for i in `ls ${ALGO}*_poses.txt`
do
	BASE=${i##*/}
	NUM=$(echo $BASE | tr -dc '0-9')
	printf ${NUM},
	./calculate_ate.sh ${ALGO}${NUM} ${DATASET_ROOT} ${NUM_POSES}
done

