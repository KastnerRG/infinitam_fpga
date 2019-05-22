#!/usr/bin/env python

import sys,os
import pandas as pd
import argparse

#sys.path.append("../../../opencl-benchmarks/results_and_analysis/csv")
#from ResultsReader import ResultsReader


def make_ID_int(csv):
    ID_int = []
    id_header = [s for s in list(csv) if s.lower().startswith("id")][0]
    for id_str in csv[id_header]:
        id_str = str(id_str)
        if '/' in id_str: id_str = os.path.basename(id_str)
        ID_int.append(filter(str.isdigit, id_str))

    new_column = pd.DataFrame({'ID_int': ID_int})

    csv["ID_int"] = new_column["ID_int"]

    return csv


def main():

    #if len(sys.argv) < 4:
    #    print("Usage: " + sys.argv[0] + " <fpga_filename> <file_to_merge> <output_file>")
    #    sys.exit(1)

    #fpga_filename   = sys.argv[1]
    #merge_filename  = sys.argv[2]
    #output_filename = sys.argv[3]

    #merge_name = os.path.splitext(os.path.basename(merge_filename))[0]


    #reader = ResultsReader(fpga_filename, "fpga")
    #reader.addIdInt()

    #reader2 = ResultsReader(merge_filename, merge_name)
    #reader2.addIdInt()
    #
    #reader.mergeTimeFrom(reader2)
    #reader.exportCsv(output_filename)


    parser = argparse.ArgumentParser(description="Merge CSV data with ID")
    
    parser.add_argument("csv_files", help='Path to the CSV files', nargs='+')
    parser.add_argument("-o", "--output", help='Name of output CSV file (default: i%(default)s)', default='merged.csv')
   
    args = parser.parse_args()

    output = args.output


    csv_merged = None

    for filename in args.csv_files:

        csv  = pd.read_csv(filename)

        csv  = make_ID_int(csv)

        if csv_merged is not None:
            csv_merged = pd.merge(csv_merged, csv, left_on="ID_int", right_on="ID_int")
        else:
            csv_merged = csv

    
    csv_merged.to_csv(output, index=False)


if __name__ == "__main__":
    main()





