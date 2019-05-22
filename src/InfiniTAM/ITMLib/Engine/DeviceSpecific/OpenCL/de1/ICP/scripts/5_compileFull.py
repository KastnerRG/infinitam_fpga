#!/usr/bin/python

import subprocess
import os
import multiprocessing as mp
import random
import signal
import sys
import re;


inFilename  = "small.txt"               # Design folder list to compile
outFilename = "compileFull.log"         # Logfile of compiled designs

clBasename  = "ITMSceneReconstructionEngine_OpenCL"               # Basename of OpenCL file


def launchCommand(path):
    
    date = subprocess.check_output("date", cwd=path, shell=True).strip()
    print(date + "  " + path)
   
    board_name = subprocess.check_output("aoc --list-boards | sed -n 2p", shell=True).strip()

    command = "time aoc " + clBasename + ".cl  --fp-relaxed --fpc --board " + board_name

    subprocess.call(command, cwd=path, shell=True)

    pathDelete = os.path.join(path, clBasename)
    #commandDelete = "rm -R -- */ ; find . -size +2M -delete"
    commandDelete = "rm -R -- */ ; find ! \( -name " + clBasename + ".log -o -name \*.attrib -o -name \*.area -o -name acl_quartus_report.txt -o -name top.fit.summary \) -type f -exec rm {} +"

    subprocess.call(commandDelete, cwd=pathDelete, shell=True)

    outFile = open(outFilename, 'at')
    outFile.write(path + '\n')
    outFile.close()




def main():

    # Get number of processors to use
    num_processes = 0
    if len(sys.argv) >= 2:
        num_processes = int(sys.argv[1])

    if num_processes > 0:
        print("Using " + str(num_processes) + " processes")
    else:
        print("Using all available processes")


    folders = []

    # Get folders from file
    inFile = open(inFilename, 'rt')
    for i, line in enumerate(inFile):
        values = line.split()
        folders.append(values[0])

    # Remove folders that we already compiled
    if(os.path.isfile(outFilename)):
        compiled = []
        logFile = open(outFilename, 'rt')
        for line in logFile:
            name = re.split(r'\s', line)
            compiled.append(name[0])
        logFile.close()

        for f in compiled:
            try:
                folders.remove(f)
            except:
                pass

    
    # Compile in random order
    # (can help to get overview of the design space for partial results)
    random.shuffle(folders)

    # Compile the folders on different processes
    if num_processes > 0:
        pool = mp.Pool(num_processes)
    else:
        pool = mp.Pool()
    result = pool.map_async(launchCommand, folders).get(31536000) # timeout of 365 days





if __name__=="__main__":
    main()
