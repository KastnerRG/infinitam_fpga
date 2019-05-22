#!/bin/bash

ALGO=/home/qkgautier/workspace/tango/ReadTangoRecords/scripts/de5_output_room_700/combined_icp
#FPGA_RESULTS=/media/qkgautier/db431c70-801e-48ae-8a1e-9b6c45d5c4a0/Quentin3DReconsDSE/depth_fusion_de5_240/depth_fusion/results/results_de5_16.1/fpga_results.csv
#FPGA_RESULTS=/home/qkgautier/workspace/tango/InfiniTAM/ITMLib/Engine/DeviceSpecific/OpenCL/de1/Raycasting/results/results_de5_240/fpga_results.csv
#FPGA_RESULTS=/home/qkgautier/workspace/tango/InfiniTAM/ITMLib/Engine/DeviceSpecific/OpenCL/de1/Combined/results/results_de5_240/fpga_results.csv
#FPGA_RESULTS=/media/qkgautier/db431c70-801e-48ae-8a1e-9b6c45d5c4a0/Quentin3DReconsDSE/icp_de5/ICP/scripts/fpga_results.csv
#FPGA_RESULTS=/media/qkgautier/db431c70-801e-48ae-8a1e-9b6c45d5c4a0/Quentin3DReconsDSE/all_algos_de5/all_algos_v2/scripts/fpga_results.csv
FPGA_RESULTS=/media/qkgautier/db431c70-801e-48ae-8a1e-9b6c45d5c4a0/Quentin3DReconsDSE/all_algos_de5/all_algos2/scripts/fpga_results.csv

#ALGO=/home/qkgautier/workspace/tango/ReadTangoRecords/scripts/de1_output_cave9_400/baseline
#FPGA_RESULTS=/home/qkgautier/workspace/tango/depth_fusion/results/results_de1_16.1_240/fpga_results.csv
#FPGA_RESULTS=/home/qkgautier/workspace/tango/InfiniTAM/ITMLib/Engine/DeviceSpecific/OpenCL/de1/Raycasting/results/results_240/fpga_results.csv
#FPGA_RESULTS=/home/qkgautier/workspace/tango/InfiniTAM/ITMLib/Engine/DeviceSpecific/OpenCL/de1/ICP/results/fpga_results.csv
#FPGA_RESULTS=/home/qkgautier/workspace/tango/InfiniTAM/ITMLib/Engine/DeviceSpecific/OpenCL/de1/Combined/results/results_240/fpga_results.csv

#FPGA_RESULTS=/home/qkgautier/workspace/tango/depth_fusion/results/results_de1_16.1/fpga_results.csv
#FPGA_RESULTS=/home/qkgautier/workspace/tango/InfiniTAM/ITMLib/Engine/DeviceSpecific/OpenCL/de1/Raycasting/results/results_180/fpga_results.csv
#FPGA_RESULTS=/home/qkgautier/workspace/tango/InfiniTAM/ITMLib/Engine/DeviceSpecific/OpenCL/de1/ICP/results/fpga_results.csv
#FPGA_RESULTS=/home/qkgautier/workspace/tango/InfiniTAM/ITMLib/Engine/DeviceSpecific/OpenCL/de1/Combined/results/results_180/fpga_results.csv

#FPGA_RESULTS=fake_fpga_results.csv

DATASET=~/data/rgbd_dataset_freiburg1_room
NUM_POSES=700

if [ ! -z "$1" ]
  then
    ALGO=$1
fi

if [ ! -z "$2" ]
  then
    FPGA_RESULTS=$2
fi

if [ ! -z "$3" ]
  then
    DATASET=$3
fi


ALGO_NAME=$(basename "$ALGO")


echo "Processing timing files..."
./processTiming.py ${ALGO}*.csv -a -n ${NUM_POSES} > /tmp/time.csv
echo "Processing pose files..."
./all_ate.sh ${ALGO} ${DATASET} ${NUM_POSES} > /tmp/ate.csv
echo "Merging files..."
./merge_data.py ${FPGA_RESULTS} /tmp/time.csv /tmp/ate.csv -o ${ALGO_NAME}.csv
echo "Done."


# Short version (without ATE)
#echo "Processing timing files..."
#./processTiming.py ${ALGO}*.csv -a > /tmp/time.csv
#echo "Merging files..."
#./merge_data.py ${FPGA_RESULTS} /tmp/time.csv -o ${ALGO_NAME}.csv
#echo "Done."

