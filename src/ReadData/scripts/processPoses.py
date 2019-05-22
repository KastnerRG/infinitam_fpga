#!/usr/bin/env python

from __future__ import division

import sys,os
import glob
import numpy as np


from scipy import interpolate

import subprocess
import tempfile


def main():

    if len(sys.argv) < 4:
        print("Usage: " + sys.argv[0] + " <poses.txt> <depth_dir> <output_positions.txt> [num_poses]")
        sys.exit(0)

    poseFilename = sys.argv[1]
    depthDir     = sys.argv[2]
    outFilename  = sys.argv[3]
    num_poses = -1
    if len(sys.argv) >= 5:
        num_poses = int(sys.argv[4])

    poseFile = open(poseFilename, 'rt')
    depthFilenames = glob.glob(os.path.join(depthDir, "*.png"))

    # Read output poses
    poses = []

    row = 0
    for line in poseFile:
        if row == 0:
            P = np.empty([4,4], dtype=np.float)
        P[row,:] = map(float,line.split(","))
        row += 1
        if row >= 4:
            poses.append(P)
            row = 0

    if num_poses <= 0: num_poses = len(poses)
    poses = poses[:num_poses]

    # Get trajectory from poses
    positions = np.empty([len(poses),3])
    for i,P in enumerate(poses):
        R = P[:3,:3]
        t = P[:3,3]
        positions[i,:] = np.dot(-R.T, t)



    # Read depth timestamps
    depthTimestamps = []
    for f in depthFilenames:
        timestamp = float(os.path.splitext(os.path.basename(f))[0])
        depthTimestamps.append(timestamp)
    depthTimestamps = np.sort(depthTimestamps)

    outFile = open(outFilename, 'wt')
    for i in range(min(len(depthTimestamps), len(positions))):
        outFile.write("{0:.5f} ".format(depthTimestamps[i]) + " ".join(map(str, positions[i])) + "\n")

    #print("./evaluate_ate.py " + gtFilename + " " + temp_name)
    #subprocess.call("python evaluate_ate.py " + gtFilename + " " + temp_name, shell=True, executable='/bin/bash')
    #subprocess.call(" ".join(["python", "evaluate_ate.py",  gtFilename, temp_name]), shell=True, cwd=".")





    #import matplotlib.pyplot as plt
    #from mpl_toolkits.mplot3d import Axes3D

    #fig = plt.figure()
    #ax = fig.add_subplot(111, projection='3d')

    #ax.plot(positions[:,0], positions[:,1], positions[:,2])
    #ax.plot(gtPositions[:,0], gtPositions[:,1], gtPositions[:,2])

    #plt.show()

    ##print("estX,estY,estZ,gtX,gtY,gtZ")
    ##for p in np.hstack((positions, gtPositions)):
    ##    print(",".join(map(str,p)))


if __name__ == "__main__":
    main()
