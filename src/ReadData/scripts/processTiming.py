#!/usr/bin/env python

from __future__ import division

import sys,os
import argparse
import numpy as np


def mad_based_outlier(data, thresh=3.5):
    data = data[:,None]
    median = np.median(data, axis=0)
    diff = np.sum((data - median)**2, axis=-1)
    diff = np.sqrt(diff)
    med_abs_deviation = np.median(diff)

    modified_z_score = 0.6745 * diff / med_abs_deviation

    return modified_z_score > thresh


def main():

    parser = argparse.ArgumentParser(description="Process timing output")
    
    parser.add_argument("timing_files", help='Path to the CSV file', nargs='+')
    parser.add_argument("-ro", "--remove-outliers", help='Remove outliers for each time data', action='store_true')
    parser.add_argument("-p", "--plot", help='Plot histogram', action='store_true')
    parser.add_argument("-a", "--all-timers", help='Extract all the timers data', action='store_true')
    parser.add_argument("-n", "--num-frames", type=int, default=-1, help='Stop extracting timing after n frames (negative to extract all - default)')
   
    args = parser.parse_args()

    removeOutliers  = args.remove_outliers
    plotHistogram   = args.plot
    multipleTimings = args.all_timers
    multiplePrintHeader = True
    num_frames = args.num_frames

    timingFilenames = args.timing_files

    for timingFilename in timingFilenames:

        timingFile = open(timingFilename, 'rt')


        alltimings = []

        
        for line in timingFile:
            values = line.strip().split(",")
            if not values[1]: values[1] = "0"
            if num_frames <= 0: num_frames = len(values) - 1
            alltimings.append((values[0],np.fromiter(map(float,values[1:num_frames+1]), dtype=float)))
            if values[0].startswith("Total_time"):
                total_timings = np.fromiter(map(float,values[1:]), dtype=float)


        if removeOutliers:
            total_timings = total_timings[np.logical_not(mad_based_outlier(total_timings))]
            for i,v in enumerate(alltimings):
                if v[1].mean() > 0:
                    alltimings[i] = (v[0],v[1][np.logical_not(mad_based_outlier(v[1]))])

        if multipleTimings:
            if multiplePrintHeader:
                header = "ID,"
                for name,_ in alltimings:
                    header += name + ","
                print(header)
                multiplePrintHeader = False
            csv = os.path.basename(timingFilename) + ","
            for _,values in alltimings:
                csv += str(values.mean()) + ","
            print(csv)
        else:
            print(str(total_timings.mean()))

        if plotHistogram:
            import matplotlib.pyplot as plt
            plt.hist(total_timings, 30)
            plt.show()



if __name__ == "__main__":
    main()
