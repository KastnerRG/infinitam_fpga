#!/usr/bin/python3

import os
import subprocess
import re
import sys


outfilename       = "fpga_results.csv" # Output filename for results

paramsfilename    = "small.txt" # List of design folders with their parameters (knob values)
compiledfilename  = "small.txt" # List of design folders

donefilename      = "fpga_run_done.csv"       # Designs already run

reportfilename    = "ITMSceneReconstructionEngine_OpenCL/acl_quartus_report.txt" # Primary report file to get info
reportfilename2   = "ITMSceneReconstructionEngine_OpenCL/top.fit.summary"        # Secondary report file in case the first one does not exist

kernel_filenames  = ["ITMSceneReconstructionEngine_OpenCL/Raycast_depth_s.attrib"] # Kernel report files

clBasename        = "ITMSceneReconstructionEngine_OpenCL"      # Basename for the OpenCL file



def main():

    # Get files to not run
    done = []
    if os.path.exists(donefilename):
        with open(donefilename, 'rt') as donefile:
            for line in donefile:
                done.append(line.split(",")[0].strip())

    # Get files to run
    dirs = []
    with open(compiledfilename, 'rt') as infile:
        for line in infile:
            value = line.split()[0]
            if not value in done:
                dirs.append(value)

    # Get knob values and other parameters for each design
    params = {}
    with open(paramsfilename, 'rt') as parfile:
        for line in parfile:
            data = line.split()
            params[data[0]] = data[1:]



    outfile = open(outfilename, 'wt')


    print(str(len(dirs)) + " directories")





    # Write CSV header
    outfile.write("ID," + ",".join(["param_"+str(i) for i in range(len(params[dirs[0]]))]) + "," + ",".join(["estim_throughput_" + str(i) for i in range(len(kernel_filenames))])
            + ",logic,mem,ram,dsp,fmax,fit\n")


    # for each directory
    #
    for d in dirs:

        reportfile_path  = os.path.join(d, reportfilename)
        reportfile_path2 = os.path.join(d, reportfilename2)


        fit = 'N'

        logicUse = -1
        memUse = -1 
        ramUse = -1 
        dspUse = -1 
        fmax   = -1 

        estim_throughput = []
        
        if os.path.isfile(os.path.join(d, clBasename + ".aocx")):
            fit = 'Y'


        # Read area info
        #
        if os.path.isfile(reportfile_path):

            fin = open(reportfile_path, 'r')

            logic_re = re.compile(r'Logic utilization:\s+(\S+)\s+')
            mem_re   = re.compile(r'Memory bits:\s+(\S+)\s+')
            ram_re   = re.compile(r'RAM blocks:\s+(\S+)\s+')
            dsp_re   = re.compile(r'DSP blocks:\s+(\S+)\s+')
            fmax_re  = re.compile(r'Kernel fmax:\s+(\S+)\s+')


            for line in fin:
                m = logic_re.search(line)
                m2 = mem_re.search(line)
                m3 = ram_re.search(line)
                m4 = dsp_re.search(line)
                m5 = fmax_re.search(line)

                if m:
                    logicUse = int(m.group(1).replace(",",""))

                if m2:
                    memUse = int(m2.group(1).replace(",",""))

                if m3:
                    ramUse = int(m3.group(1).replace(",",""))

                if m4:
                    dspUse = int(m4.group(1).replace(",",""))

                if m5:
                    fmax   = float(m5.group(1).replace(",",""))


        elif os.path.isfile(reportfile_path2):

            fin = open(reportfile_path2, 'r')

            logic_re = re.compile(r'Logic utilization.*:\s+(\S+)\s+')
            mem_re = re.compile(r'Total block memory bits :\s+(\S+)\s+')
            dsp_re = re.compile(r'Total DSP Blocks :\s+(\S+)\s+')


            for line in fin:
                m = logic_re.search(line)
                m2 = mem_re.search(line)
                m4 = dsp_re.search(line)

                if m:
                    logicUse = int(m.group(1).replace(",",""))

                if m2:
                    memUse = int(m2.group(1).replace(",",""))

                if m4:
                    dspUse = int(m4.group(1).replace(",",""))

                else:
                    pass

        # Read estimated throughput
        #
        through_re = re.compile(r'Throughput:\s+(\S+)\s+')
        for name in kernel_filenames:
            k_path = os.path.join(d, name)
            has_throughput = False
            if os.path.isfile(k_path):
                with open(k_path, 'rt') as f:
                    for line in f:
                        m = through_re.search(line)
                        if m:
                            estim_throughput.append(m.group(1))
                            has_throughput = True
                            break
            if not has_throughput:
                estim_throughput.append("0")


        # Write result to CSV
        #
        csv = d + "," + ",".join(params[d]) + "," + ",".join(estim_throughput) + "," + str(logicUse) + "," + str(memUse) + "," + str(ramUse) + "," + str(dspUse) + "," + str(fmax) + "," + fit

        outfile.write(csv + "\n")
        outfile.flush()

        print(csv)









if __name__=="__main__":
    main()


