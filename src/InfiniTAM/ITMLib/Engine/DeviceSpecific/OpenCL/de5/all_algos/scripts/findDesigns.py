#!/usr/bin/env python3

from __future__ import division

import pandas as pd
import sys
import argparse
import itertools

from constraint import Problem
from constraint import MaxSumConstraint


def readResults(filename):
    data = pd.read_csv(filename)
    data = data.drop(data[data.logic < 0].index)
    data.reset_index(drop=True, inplace=True)
    return data


def processResults(all_data):

    prob = Problem()
    for i,d in enumerate(all_data):
        prob.addVariable(i, d["logic"])

    prob.addConstraint(MaxSumConstraint(234720))

    solutions = prob.getSolutionIter()

    for i,s in enumerate(solutions):
        if i % 1000 == 0:
            print(i)





    #num_combs = 1
    #for d in all_data: num_combs *= len(d)

    #lengths = [range(len(d)) for d in all_data]
    #all_combinations = itertools.product(*lengths)

    #good_comb = []

    #progress_pc = 0

    #for progress,comb in enumerate(all_combinations):
    #    total_logic = 0
    #    for i,index in enumerate(comb):
    #        total_logic += all_data[i]["logic"][index]
    #    if total_logic <= 234720:
    #        good_comb.append(comb)

    #    progress_pc_new = (((progress+1) / num_combs) * 100)
    #    if progress_pc_new != progress_pc:
    #        progress_pc = progress_pc_new
    #    print(progress_pc_new)


def main():
    
    parser = argparse.ArgumentParser(description="Plot design spaces")

    parser.add_argument("inputfile", help='Path to the CSV input file', nargs='+')
    
    args = parser.parse_args()
    
    files = args.inputfile

    all_data = [readResults(f) for f in files]

    processResults(all_data)


if __name__ == '__main__':
    main()


