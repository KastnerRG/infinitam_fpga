#!/usr/bin/env python3

from __future__ import division

import sys
import os
import argparse
import subprocess
import re


#----------------------------------------------------------------------
def main():
#----------------------------------------------------------------------
    parser = argparse.ArgumentParser(description="Generate InfiniTAM experiments")
    
    parser.add_argument("stdout_file", help='Path to the text file containing stdout from readData')
    
    args = parser.parse_args()
    
    in_filename = args.stdout_file


    with open(in_filename) as f:
        lines = f.readlines()

    #time_re = re.compile(r'(\S+): (\S+)ms \(Average: (\S+)ms')
    time_re = re.compile(r'(\S+): (\S+)ms')

    program_profiling = {}                

    for line in lines:
        m = time_re.search(line)
        if m:
            program_profiling.setdefault(m.group(1), []).append(float(m.group(2)))

    
    timer_ids = [
            "Tracking1", "Tracking2",
            "Raycast", "Raycast_CreateDepths", "Raycast_CreatePointCloud", "Raycast_CreateICPMaps", "Raycast_ForwardRender",
            "Fusion", "Fusion_allocate", "Fusion_integrate",
            "Total_time",
            "Tracking1_Kernel", "Raycast_CreateICPMaps_Kernel", "Fusion_integrate_Kernel"]

    text = ""
    for i in timer_ids:
        text += i + "," + ",".join(map(str, program_profiling.get(i, []))) + "\n"

    with open(in_filename + "_timing.csv", 'wt') as fout:
        fout.write(text)

    output = subprocess.check_output(
            "./processTiming.py " +
            in_filename + "_timing.csv",
            shell=True)

    time = float(output)

    print(time)




if __name__ == "__main__":
    main()



