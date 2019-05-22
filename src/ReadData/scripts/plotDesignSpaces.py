#!/usr/bin/env python3

from __future__ import division

import sys
import argparse
import matplotlib
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import pandas as pd

from matplotlib import cm
from matplotlib.ticker import LinearLocator, FormatStrFormatter

def getParetoOptimalDesigns(designs):
    """
    : designs: numDesigns x numObjectives
    : Return boolean mask of Pareto optimal designs
    """
    nperr = np.seterr(invalid='ignore') # (ignore NaN)
    isOptimal = np.ones(designs.shape[0], dtype = bool)
    for i, c in enumerate(designs):
        if isOptimal[i]:
            isOptimal[isOptimal] = np.any(designs[isOptimal]>c, axis=1)  # Remove dominated points
            isOptimal[i] = True
    np.seterr(**nperr)
    return isOptimal


def main():

    #### Parse arguments ###
       
    parser = argparse.ArgumentParser(description="Plot design spaces")

    parser.add_argument("inputfile", help='Path to the CSV input file', nargs='+')
    
    parser.add_argument("-x", "--x-label", default="", help='X axis label')
    parser.add_argument("-y", "--y-label", default="", help='Y axis label')
    parser.add_argument("-z", "--z-label", default="", help='Z axis label')
    parser.add_argument("-t", "--title",   default="", help='Plot title')
    parser.add_argument("-n", "--normalize",  action='store_true', help='Normalize to maximum')
    parser.add_argument("-nate", "--no-ate",  action='store_true', help='Do not read ATE')
    parser.add_argument("-nlog", "--no-logic",  action='store_true', help='Do not read Logic')
    parser.add_argument("-xmax", "--max-x", default="-1", type=float, help='Max X value to plot')
    parser.add_argument("-xmin", "--min-x", default="-1", type=float, help='Min X value to plot')
    parser.add_argument("-ymax", "--max-y", default="-1", type=float, help='Max Y value to plot')
    parser.add_argument("-ymin", "--min-y", default="-1", type=float, help='Min Y value to plot')
    parser.add_argument("-time", "--time", default="Total_time", help='Which timer to use from the input data')
    #parser.add_argument("-hk", "--highlight-knobs",    help='Highlight the given set of knobs', nargs="*")
    
    args = parser.parse_args()


    filenames = args.inputfile
    xLabel    = args.x_label
    yLabel    = args.y_label
    zLabel    = args.z_label
    title     = args.title
    do_normalize = args.normalize
    with_ate  = not args.no_ate
    with_logic = not args.no_logic
    max_x     = args.max_x
    min_x     = args.min_x
    max_y     = args.max_y
    min_y     = args.min_y
    time_label= args.time
    #knobsHigh = args.highlight_knobs

    if not xLabel: xLabel = "1 / Time" #("1 / " if do_normalize else "") + "Time"
    if not yLabel: yLabel = ("1 / " if do_normalize else "") + "Logic"
    if not zLabel: zLabel = ("1 / " if do_normalize else "") + "Error"
    
    ### Read designs
   
    time_factor  = 0.001
    logic_factor = 100/32070
    ate_factor   = 100

    has_ate = False
    has_baseline = False

    all_labels = []
    all_knobs  = []
    all_knob_names = []

    algos = ['Depth Fusion', 'Ray Casting', 'Combined', 'ICP', 'Baseline']
    axlabels = []

    for ifile,filename in enumerate(filenames):
        csv = pd.read_csv(filename)
        csv["logic_inv"] = 1/(csv["logic"] * logic_factor)
        if "ate" in csv: has_ate = with_ate
        if has_ate: csv["ate_inv"]   = 1/(csv["ate"] * ate_factor)
        csv["time_inv"]  = 1/(csv[time_label] * time_factor)

        labels_list = [csv["time_inv"]]
        if with_logic: labels_list.append(csv["logic_inv"])
        if has_ate: labels_list.append(csv["ate_inv"])

        labels = np.vstack(labels_list).T
        if do_normalize:
            labelsMax = np.max(labels, 0)
            labelsNorm = labels / labelsMax
            all_labels.append(labelsNorm)
        else:
            all_labels.append(labels)

        knob_names = [s for s in list(csv) if s.startswith("knob_") or s.startswith("param_")]
        knobs = np.vstack([csv[n] for n in knob_names]).T
        
        all_knob_names.append(knob_names)
        all_knobs.append(knobs)
        axlabels.append(algos[ifile])

        if ((ifile == 0 and "depth" not in filename.lower())
                or (ifile == 1 and "raycast" not in filename.lower())
                or (ifile == 2 and "combine" not in filename.lower())
                or (ifile == 3 and "icp" not in filename.lower())
                or (ifile == 4 and "baseline" not in filename.lower())):
            print("Files should be in order: Depth, Raycast, Combined, ICP, Baseline")
            sys.exit(1)

        if "baseline" in filename.lower():
            has_baseline = True
  

    axlabels.append('Pareto Designs')

    #highlightIndexes = np.empty(0, dtype=int)
    #if knobsHigh is None: knobsHigh = []
    #for k in knobsHigh:
    #    knobValues = np.fromiter(map(float, k.strip().split(",")), dtype=float)
    #    print(knobValues)
    #    print(knobValues==knobs)
    #    highlightIndexes = np.append(highlightIndexes, np.where(np.all(np.equal(knobs, knobValues), 1))[0])



    def compute_pareto(idx=None):
        
        if idx is None: idx = np.arange(all_labels[0].shape[1])
        
        if has_baseline:
            paretoMasks = getParetoOptimalDesigns(np.vstack(all_labels[:-1][:,idx])) # Remove baseline from pareto calculation
            paretoMasks = np.concatenate((paretoMasks, np.array([False] * all_labels[-1].shape[0])))
        else:
            paretoMasks = getParetoOptimalDesigns(np.vstack(all_labels)[:,idx])
        paretoMasks = np.split(paretoMasks, np.cumsum([l.shape[0] for l in all_labels])[:-1])

        return paretoMasks




    colors   = ['#599ad3', '#f9a65a', '#9e66ab', '#3e9651', '#636363']


    fig = plt.figure()
   

    def plot_2D(idx=None, num_plots=1, plot_idx=1, with_legend=True):

        ax = fig.add_subplot(1, num_plots, plot_idx)
        
        Xp = np.empty(0)
        Yp = np.empty(0)

        if idx is None: idx = np.arange(all_labels[0].shape[1])

        paretoMasks = compute_pareto(idx)

        for i,labelsNorm in enumerate(all_labels):
            paretoMask = paretoMasks[i]
            
            labelsNorm = labelsNorm[:,idx]
            
            Xp_ = labelsNorm[paretoMask,0]
            Yp_ = labelsNorm[paretoMask,1]

            if not do_normalize:
                #Xp_ = 1/Xp_
                Yp_ = 1/Yp_

            Xp  = np.append(Xp, Xp_)
            Yp  = np.append(Yp, Yp_)


            X = labelsNorm[:,0] #labelsNorm[np.logical_not(paretoMask),0]
            Y = labelsNorm[:,1] #labelsNorm[np.logical_not(paretoMask),1]

            if not do_normalize:
                #X = 1/X
                Y = 1/Y


            paretolabel = axlabels[-1] if i == 0 else ""
            ax.plot(X, Y, 'o', markeredgecolor='black', markeredgewidth=1, markersize=7, color=colors[i], label=axlabels[i])

            #ax.plot(labelsNorm[highlightIndexes,0], labelsNorm[highlightIndexes,1], 'ro', markeredgecolor='black', markeredgewidth=0.2)

        
        ax.plot(Xp, Yp, '', label=axlabels[-1], linestyle=':', linewidth=2, color='black') # markeredgecolor='black', markeredgewidth=1, markersize=16, markerfacecolor=(0, 0, 0, 0.3), 


        if do_normalize:
            ax.set_xlim([0,1.05])
            ax.set_ylim([0,1.05])

        if max_x > 0:
            ax.set_xlim(xmax=max_x)
        if min_x > 0:
            ax.set_xlim(xmin=min_x)
        if max_y > 0:
            ax.set_ylim(ymax=max_y)
        if min_y > 0:
            ax.set_ylim(ymin=min_y)

        axis_labels = np.array([xLabel, yLabel, zLabel])[idx]

        ax.set_xlabel(axis_labels[0], fontsize=12)
        ax.set_ylabel(axis_labels[1], fontsize=12)
        ax.set_title(title, fontsize=16)
        if with_legend: ax.legend(numpoints=1, fontsize=12)



    def plot_3D():
        ax = fig.gca(projection='3d')
        
        plot_proxies = []

        paretoMasks = compute_pareto()

        for i,labelsNorm in enumerate(all_labels):
            paretoMask = paretoMasks[i]
            knobs = all_knobs[i]
            knob_names = all_knob_names[i]

            Xp = labelsNorm[paretoMask,0]
            Yp = labelsNorm[paretoMask,1]
            Zp = labelsNorm[paretoMask,2]

            if not do_normalize:
                #Xp = 1/Xp
                Yp = 1/Yp
                Zp = 1/Zp

            pareto_params = np.hstack((knobs[paretoMask], np.reshape(Xp, (-1, 1)), np.reshape(Yp, (-1, 1)), np.reshape(Zp, (-1, 1))))
            print(",".join(knob_names))
            for p in pareto_params:
                print(",".join(map(str,p)))
            print("")



            X = labelsNorm[np.logical_not(paretoMask),0] #labelsNorm[np.logical_not(paretoMask),0]
            Y = labelsNorm[np.logical_not(paretoMask),1] #labelsNorm[np.logical_not(paretoMask),1]
            Z = labelsNorm[np.logical_not(paretoMask),2] #labelsNorm[np.logical_not(paretoMask),2]

            if not do_normalize:
                #X = 1/X
                Y = 1/Y
                Z = 1/Z
                ax.set_ylim(ymax=100)

            if max_x > 0:
                ax.set_xlim(xmax=max_x)
            if min_x > 0:
                ax.set_xlim(xmin=min_x)
            if max_y > 0:
                ax.set_ylim(ymax=max_y)
            if min_y > 0:
                ax.set_ylim(ymin=min_y)

            #X, Y = np.meshgrid(X, Y)
            

            # Scatter plot
            if i >= 4: # To plot baseline
                ax.scatter(X[0], Y[0], Z[0], marker='o', s=32, linewidth=0, antialiased=True, color=colors[i])
                ax.scatter(X[1], Y[1], Z[1], marker='s', s=32, linewidth=0, antialiased=True, color=colors[i])
                ax.scatter(X[2], Y[2], Z[2], marker='^', s=32, linewidth=0, antialiased=True, color=colors[i])
                ax.scatter(X[3], Y[3], Z[3], marker='D', s=32, linewidth=0, antialiased=True, color=colors[i])
            else:
                ax.scatter(X, Y, Z, s=32, linewidth=0, antialiased=True, color=colors[i])

    
            ax.scatter(Xp, Yp, Zp,
                    #cmap=cm.coolwarm,
                    marker='^', s=64,
                    linewidth=1, edgecolors='black', antialiased=True, color=colors[i], alpha=0.8) #color=colors[i%len(colors)])




            plot_proxies.append(matplotlib.lines.Line2D([0],[0], linestyle="none", c=colors[i], marker = 'o'))
       
        
        plot_proxies.append(matplotlib.lines.Line2D([0],[0], linestyle="none", mec='black', marker = '^', markersize=8, fillstyle='none'))


        # Customize the z axis.
        #ax.set_zlim(-1.01, 1.01)
        #ax.zaxis.set_major_locator(LinearLocator(10))
        #ax.zaxis.set_major_formatter(FormatStrFormatter('%.02f'))

        ax.set_xlabel(xLabel, fontsize=14)
        ax.set_ylabel(yLabel, fontsize=14)
        ax.set_zlabel(zLabel, fontsize=14)
        ax.set_title(title)
        ax.legend(plot_proxies, axlabels, numpoints=1, loc='upper center')





    if not has_ate or not with_logic:

        plot_2D()

    else:
        plot_2D([0,1], 2, 1, with_legend=False)
        plot_2D([0,2], 2, 2)

        #plot_3D()



    plt.show()
    


if __name__ == "__main__":
    main()
