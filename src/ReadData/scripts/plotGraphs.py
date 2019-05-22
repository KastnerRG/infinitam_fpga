#!/usr/bin/env python3

from __future__ import division

import sys
import argparse
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from matplotlib import cm
from matplotlib.ticker import LinearLocator, FormatStrFormatter, ScalarFormatter


def plot_subgraph(filenames, args, ax, colors, plotLegend=True):

    xLabel    = args.x_label
    yLabel    = args.y_label
    add_title = not args.no_title
    with_ate  = False #not args.no_ate
    #time_label= args.time
    log_scale = args.log
    scale_y   = args.scale_y
    axis      = args.axis

    title_location = 0.8 if axis == 1 else 0.7
    legend_location = 'best'

    ### Read designs
   
    time_factor  = 0.001
    logic_factor = 100/32070
    ate_factor   = 100

    has_ate = False
    has_baseline = False

    all_labels = []
    all_knobs  = []
    all_knob_names = []
    legend = []
    plot_proxies = []

    for ifile,filename in enumerate(filenames):

        if "depth" in filename.lower():
            time_label = "Fusion_integrate"
            title = "Depth Fusion"
        elif "ray" in filename.lower():
            time_label = "Raycast_EstimatedRaycast"
            title = "Ray Casting"
        elif "com" in filename.lower():
            time_label = "Fusion_integrate"
            title = "Combined Kernel"
            title_location = 0.0
            if axis == 0: legend_location = 'lower right'
        elif "icp" in filename.lower():
            time_label = "Tracking1"
            title = "ICP"
            title_location = 0.0

        csv = pd.read_csv(filename)
        csv["logic_inv"] = 1/(csv["logic"] * logic_factor)
        if "ate" in csv: has_ate = with_ate
        if has_ate: csv["ate_inv"]   = 1/(csv["ate"] * ate_factor)
        csv["time_inv"]  = 1/(csv[time_label] * time_factor)

        labels_list = [csv["time_inv"], csv["logic_inv"]]
        if has_ate: labels_list.append(csv["ate_inv"])

        labels = np.vstack(labels_list).T
        all_labels.append(labels)

        knob_names = [s for s in list(csv) if s.startswith("knob_") or s.startswith("param_")]
        knobs = np.vstack([csv[n] for n in knob_names]).T
        
        all_knob_names.append(knob_names)
        all_knobs.append(knobs)

        if "desk" in filename.lower():
            legend.append("Desk")
        elif "room" in filename.lower():
            legend.append("Room")
        elif "360" in filename.lower():
            legend.append("360")
        elif "cave" in filename.lower():
            legend.append("Cave")
        else:
            legend.append("")


    styles   = ['solid', 'dashed', 'dotted', 'dashdot']

    #ax2 = ax.twinx()
    
    for i,labels in enumerate(all_labels):
        
        Y = 1/labels[:,axis] * scale_y
        X = np.arange(len(Y))

        if i == 0:
            indexes = np.argsort(Y)
            #for j in [11]: #np.arange(all_knobs[0].shape[1]-4):
            #    knob_values = all_knobs[0][indexes,j].astype(np.float)
            #    knob_values -= np.min(knob_values)
            #    m = np.max(knob_values)
            #    if m > 0:
            #        knob_values /= m
            #    ax2.plot(X, knob_values)

        print(Y[indexes][0])
        ax.plot(X, Y[indexes], color=colors[i], alpha=1, label=legend[i], linewidth=2, linestyle=styles[i])
        
        plot_proxies.append(matplotlib.lines.Line2D([0],[0], c='black', linewidth=2, linestyle=styles[i]))

    if log_scale and title != "ICP":
        ax.set_yscale('log')
    ax.yaxis.set_major_formatter(ScalarFormatter(useOffset=False))

    ax.set_xlabel(xLabel, fontsize=14)
    ax.set_ylabel(yLabel, fontsize=14)
    if add_title: ax.set_title(title, y=title_location, fontsize=14, fontweight='bold')
    
    if plotLegend:
        #ax.legend(numpoints=1, fontsize=12, loc=legend_location)
        #ax.legend(plot_proxies, legend, numpoints=1, fontsize=12, loc=legend_location)
        ax.legend(plot_proxies, legend, bbox_to_anchor=(0., 1.02, 1., 1.02), loc=3,
                           ncol=len(legend), mode="expand", borderaxespad=0.)

    #ax.xaxis.set_ticks([])

    #ax2.set_ylim(ymax=10)


def main():

    #### Parse arguments ###
       
    parser = argparse.ArgumentParser(description="Plot design spaces")

    parser.add_argument("inputfile", help='Path to the CSV input file', nargs='+')
    
    parser.add_argument("-x", "--x-label", default="Designs", help='X axis label')
    parser.add_argument("-y", "--y-label", default="Running Time (ms)", help='Y axis label')
    parser.add_argument("-nt", "--no-title",  action="store_true", help='Disable titles')
    #parser.add_argument("-time", "--time", default="Total_time", help='Which timer to use from the input data')
    parser.add_argument("-log", "--log", action="store_true", help='Log scale')
    parser.add_argument("-ys", "--scale-y", default="1000", type=float, help='Scale factor for Y values. Default: %(default)s')
    parser.add_argument("-axis", "--axis", default="0", type=int, help='Axis (time, logic, ate). Default: %(default)s')
    
    args = parser.parse_args()

    plot_legend = True

    if args.axis != 1:
        filenames = list(filter(lambda l: len(l)>0, np.split(args.inputfile, [4,8,12,16])))
    else:
        plot_legend = False
        filenames = [[f] for f in args.inputfile]

    #colors   = ['#599ad3', '#f9a65a', '#9e66ab', '#3e9651', '#636363']
    colors_gs = ['#000000', '#252525', '#525252', '#737373', '#969696']
    colors_df = ['#084594', '#2171b5', '#4292c6', '#6baed6', '#9ecae1']
    colors_rc = ['#8c2d04', '#cc4c02', '#ec7014', '#fe9929', '#fec44f']
    colors_cb = ['#4a1486', '#6a51a3', '#807dba', '#9e9ac8', '#bcbddc']
    colors_ic = ['#006d2c', '#238b45', '#41ae76', '#66c2a4', '#99d8c9']

    colors = [colors_df, colors_rc, colors_cb, colors_ic]
    
    fig = plt.figure()

    xLabel = args.x_label
    yLabel = args.y_label

    for i,fn in enumerate(filenames):

        ax = fig.add_subplot(len(filenames), 1, i+1)

        args.x_label = "" if i < len(filenames)-1 else xLabel
        args.y_label = "" if i != int(len(filenames)/2) else yLabel

        do_plot_legend = plot_legend and (i == 0)

        plot_subgraph(fn, args, ax, colors[i], plotLegend=do_plot_legend)

    plt.show()
    


if __name__ == "__main__":
    main()
