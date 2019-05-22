#!/usr/bin/python

import subprocess
import os
import multiprocessing as mp
import random
import sys


logName      = "estimateResults.log" # Where to store the results

rootDir      = "../benchmarks"       # Directory containing all the designs
dir_basename = "combined"        # Basename of the design folders
cl_basename  = "ITMSceneReconstructionEngine_OpenCL"           # Basename of the OpenCL file


def launchCommand(path):

    board_name = subprocess.check_output("aoc --list-boards | sed -n 2p", shell=True).strip()

    command = "aoc " + cl_basename + ".cl -c -g --fp-relaxed --fpc --board " + board_name
    
    subprocess.call(command, cwd=path, shell=True)
    
    pathDelete = os.path.join(path, cl_basename)
    commandDelete = "rm -R -- */ ; find ! \( -name " + cl_basename + ".log \) -type f -exec rm {} + ; rm ../" + cl_basename + ".aoco"
    
    subprocess.call(commandDelete, cwd=pathDelete, shell=True)
    
    outFile = open(logName, 'at')
    outFile.write(path + '\n')
    outFile.close()


	

def main():
    
    # Get number of processors
    num_processes = 0
    if len(sys.argv) >= 2:
    	num_processes = int(sys.argv[1])
    
    if num_processes > 0:
    	print("Using " + str(num_processes) + " processes")
    else:
    	print("Using all available processes")
    
    
    # Folders to compile
    folders = [os.path.join(rootDir, d) for d in os.listdir(rootDir) if d.startswith(dir_basename)]
    

    # Get folders already compiled from previous log file
    compiled = []
    
    if os.path.isfile(logName):
    	print("Reading processed folders from log file")
    	logFile = open(logName, 'rt')
    	for line in logFile:
    		name = line.split('\n')
    		compiled.append(name[0])
    	logFile.close()
    
    
    for f in compiled:
    	folders.remove(f)
    
    
    
    # Start processes 
    print("Processing " + str(len(folders)) + " folders")

    if num_processes > 0:
    	pool = mp.Pool(num_processes)
    else:
    	pool = mp.Pool()
    pool.map(launchCommand, folders)



if __name__=="__main__":
	main()
