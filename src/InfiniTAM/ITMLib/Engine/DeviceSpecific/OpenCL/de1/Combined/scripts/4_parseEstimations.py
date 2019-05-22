#!/usr/bin/python

import os.path
import re
import sys


aocFilename     = "ITMSceneReconstructionEngine_OpenCL/ITMSceneReconstructionEngine_OpenCL.log" # Altera OpenCL log filename (relative to design folder)

smallFilename   = "small.txt"               # File to store small designs (likely to fit)
bigFilename     = "big.txt"                 # File to store large designs (not likely to fit)

foldersFilename = "params.log"              # Log file containing design folder list




smallFile = open(smallFilename, 'wt')
bigFile = open(bigFilename, 'wt')


numWarningFiles = 0


folderFile = open(foldersFilename, 'rt')

for line in folderFile:

	i = line.split()[0]
	filePath = os.path.join(i, aocFilename)

	if not os.path.isfile(filePath):
		print(filePath + " does not exist")
		#sys.exit(1)
		continue



	fin = open(filePath, 'r')

	logic_re = re.compile(r'Logic utilization\s+;\s*(\d+)%')
	reg_re   = re.compile(r'Dedicated logic registers\s+;\s*(\d+)%')
	mem_re   = re.compile(r'Memory blocks\s+;\s*(\d+)%')
	dsp_re   = re.compile(r'DSP blocks\s+;\s*(\d+)%')
	warn_re  = re.compile(r'Warning')


	logicUse = 0
	regUse = 0
	memUse = 0
	dspUse = 0

	hasWarning = False

	for line2 in fin:
		m = logic_re.search(line2)
		m2 = reg_re.search(line2)
		m3 = mem_re.search(line2)
		m4 = dsp_re.search(line2)

		if m:
			logicUse = int(m.group(1))

		if m2:
			regUse = int(m2.group(1))
			
		if m3:
			memUse = int(m3.group(1))

		if m4:
			dspUse = int(m4.group(1))

		if warn_re.search(line2):
			hasWarning = True

	if logicUse * regUse * memUse == 0: # * dspUse == 0:
		print(filePath)
		continue

	text = line.strip() + " " + ' '.join(map(str, [logicUse, regUse, memUse, dspUse])) + '\n'


	if logicUse > 90 or dspUse > 100 or regUse > 100 or memUse > 100:
		bigFile.write(text)
	else:	
		smallFile.write(text)
		if hasWarning: numWarningFiles += 1


print(str(numWarningFiles) + " warnings among smalls")

