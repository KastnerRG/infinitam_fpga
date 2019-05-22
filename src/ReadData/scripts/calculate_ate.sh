#!/bin/sh

# $1 Design ID
# $2 Root path to dataset

./processPoses.py ${1}_poses.txt ${2}/depth ${1}_positions.txt
./evaluate_ate.py ${1}_positions.txt ${2}/groundtruth.txt --plot ${1}_ate.png

