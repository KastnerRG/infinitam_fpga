#!/usr/bin/env python3

from __future__ import division

import sys
import os
import errno
import argparse
try: import configparser
except: import ConfigParser as configparser
import subprocess
import re
import shutil
import signal
import itertools
import psutil

sys.path.append("../../../opencl-benchmarks/results_and_analysis/csv")
from ResultsReader import ResultsReader


FULL_UNROLL = -1


#----------------------------------------------------------------------
def createTextFromTemplate(templateFilepath, knob_values, start=1):
#----------------------------------------------------------------------

    strValues = [' ' if str(v)==str(FULL_UNROLL) else str(v) for v in knob_values]

    # Read template file and set the knob values
    knobs_text = ""
    with open(templateFilepath, "rt") as fin:
        for line in fin:
    
            newline = line
    
            for (i, replace) in reversed(list(enumerate(strValues))):
                text = '%' + str(i+start)
                newline = newline.replace(text, replace)
    
            knobs_text += newline

    return knobs_text


#----------------------------------------------------------------------
def symlink_force(target, link_name):
#----------------------------------------------------------------------
    try:
        os.symlink(target, link_name)
    except OSError as e:
        if e.errno == errno.EEXIST and os.path.islink(link_name):
            os.remove(link_name)
            os.symlink(target, link_name)
        else:
            raise e

#----------------------------------------------------------------------
def modifyProgramParameters(working_dir, param_filename, param_ids, param_values):
#----------------------------------------------------------------------
    filename = os.path.join(working_dir, param_filename)
    params = []
    with open(filename, 'rt') as fin:
        for line in fin:
            l = line.strip()
            for i,param in enumerate(param_ids):
                if l.startswith(param):
                    l = param + " = " + str(param_values[i])
            params.append(l)
    with open(filename, 'wt') as fout:
        fout.write("\n".join(params))


#----------------------------------------------------------------------
def compileRunParseProgram(program_exe, program_options, compile_command, working_dir, isde1=False):
#----------------------------------------------------------------------
    # Compile the program
    subprocess.call(compile_command, cwd=working_dir, shell=True)

    # Run the program
    if True:
        process = None
        if isde1:
            process = subprocess.Popen("nice -n -20 " + program_exe + " " + program_options, cwd=working_dir, shell=True, stdout=subprocess.PIPE, preexec_fn=os.setsid)
        else:
            process = subprocess.Popen("" + program_exe + " " + program_options, cwd=working_dir, shell=True, stdout=subprocess.PIPE, preexec_fn=os.setsid)

        try:
            #cpu = 100
            #while process.returncode is None:
            #    if cpu < 2:
            #        raise Exception("Timeout")
            #    cpu = psutil.cpu_percent(interval=30)
            #    process.poll()
            program_output = process.communicate(timeout=86400)[0] # Timeout of 24 hours
            program_output = program_output.decode('UTF-8')
        #except subprocess.TimeoutExpired:
        except:
            print("*** Timeout: killing process")
            os.killpg(os.getpgid(process.pid), signal.SIGTERM) # send signal to the process group
            program_output = ""
    
    
    #program_output = subprocess.check_output("nice -n -20 " + program_exe + " " + program_options, cwd=working_dir, shell=True)

    # Parse output
    lines = program_output.split('\n')

    time_re = re.compile(r'(\S+): (\S+)ms') # \(Average: (\S+)ms')

    program_profiling = {}                

    for line in lines:
        m = time_re.search(line)
        if m:
            try:
                program_profiling.setdefault(m.group(1), []).append(float(m.group(2)))
            except:
                print("Problem in line: " + line)

    # Return data
    return program_profiling


#----------------------------------------------------------------------
def saveProgramOutputFiles(working_dir, dest_dir, design_id):
#----------------------------------------------------------------------
    pose_filename = os.path.join(working_dir, "poses.txt")
    world_filename = os.path.join(working_dir, "world.pcd")

    shutil.copyfile(pose_filename, os.path.join(dest_dir, str(design_id) + "_poses.txt"))
    #shutil.copyfile(world_filename, os.path.join(dest_dir, str(design_id) + "_world.pcd"))

    os.remove(pose_filename)
    #os.remove(world_filename)

#----------------------------------------------------------------------
def saveProgramOutput(program_output, dest_dir, design_id):
#----------------------------------------------------------------------
    
    timer_ids = [
            "Tracking1", "Tracking2", "Tracking1_Kernel",
            "Raycast", "Raycast_CreateDepths", "Raycast_CreatePointCloud", "Raycast_CreateICPMaps", "Raycast_ForwardRender", "Raycast_CreateICPMaps_Kernel",
            "Fusion", "Fusion_allocate", "Fusion_integrate", "Fusion_integrate_Kernel", 
            "Total_time"]

    text = ""
    for i in timer_ids:
        text += i + "," + ",".join(map(str, program_output.get(i, []))) + "\n"

    with open(os.path.join(dest_dir, str(design_id) + "_timing.csv"), 'wt') as fout:
        fout.write(text)


#----------------------------------------------------------------------
def processProgramOutputFiles(design_id, dest_dir, groundtruth_root):
#----------------------------------------------------------------------
    
    output = subprocess.check_output(
            "./calculate_ate.sh "
            + os.path.join(dest_dir, str(design_id)) + " "
            + groundtruth_root,
            shell=True)

    ate = float(output)

    output = subprocess.check_output(
            "./processTiming.py " +
            os.path.join(dest_dir, str(design_id) + "_timing.csv"),
            shell=True)

    time = float(output)

    return ate, time


#----------------------------------------------------------------------
def readAlgorithmParam(program_config, param_id, result):
#----------------------------------------------------------------------
    param_str = program_config.get('PARAMETERS', param_id)
    # TODO handle the case where the parameter does not exist (default)
    #param_str = ""
    #try: param_str = program_config.get('PARAMETERS', param_id)
    #except: pass
    result.append(list(map(lambda x: x.strip(), param_str.split(","))))


#----------------------------------------------------------------------
def startLogFile(log_filename, discard_log=False):
#----------------------------------------------------------------------
    previous_log = []
    if os.path.isfile(log_filename):
        with open(log_filename, 'rt') as fin:
            previous_log = [line.strip() for line in fin]

    logfile = open(log_filename, 'wt')

    if discard_log:
        previous_log = []
    else:
        logfile.write("\n".join(previous_log) + "\n")

    return (logfile, previous_log)



#----------------------------------------------------------------------
def main():
#----------------------------------------------------------------------

    parser = argparse.ArgumentParser(description="Generate InfiniTAM experiments")
    
    parser.add_argument("program", help='Path to the program config file')
    parser.add_argument("-cl", "--opencl", help='File describing the OpenCL experiments')
    parser.add_argument("-wd", "--working-dir", help='Path to the working directory (default: same dir as program binary)')
    parser.add_argument("-o", "--output-dir", help='Path to the directory where to copy the experiments output files (default: %(default)s)', default=".")

    args = parser.parse_args()

    # Read program options
    program_config = configparser.ConfigParser()
    program_config.read(args.program)

    program_exe         = program_config.get('PROGRAM', 'binary')
    program_options     = program_config.get('PROGRAM', 'options')
    parameters_filename = program_config.get('PROGRAM', 'parameters_file')
    compile_command     = program_config.get('PROGRAM', 'compile_command')

    try:
        groundtruth_root = program_config.get('PROGRAM', 'groundtruth_root')
    except:
        groundtruth_root = ""

    try:
        isde1 = (program_config.get('PROGRAM', 'de1') != "0")
    except:
        isde1 = False


    params_experiments  = (program_config.get('PARAMETERS', 'params_experiments') != "0")
    params_log          =  program_config.get('PARAMETERS', 'log_file')



    # Get command line options
    working_dir = os.path.dirname(program_exe)
    if args.working_dir:
        working_dir = args.working_dir

    output_dir = args.output_dir


    print("Running experiments with " + program_exe)
    print("In directory " + working_dir)




    ### Parameters experiments
    # Separate parameter experiment from OpenCL experiment for now
    
    # Read lists of parameters
    if params_experiments:
        all_parameters = []
        all_parameters_ids = [
                'device_type','opencl_device','opencl_algo','use_swapping','approximate_raycast',
                'use_bilateral_filter','tracker_type','no_hierarchy_levels','tracking_regime',
                'icp_quality','color_skip_points','icp_dist_threshold','icp_error_threshold',
                'voxel_size','mu','use_max_w','max_w'
                ]
        for p in all_parameters_ids:
            readAlgorithmParam(program_config, p, all_parameters)
        
        all_combinations = itertools.product(*all_parameters)

        final_combinations = []
        for c in all_combinations:

            device_type   = c[0]
            opencl_device = c[1]
            opencl_algo   = c[2]
            no_hierarchy_levels = c[7]
            tracking_regime = c[8]
            voxel_size = c[13]
            mu = c[14]

            keepCombi = True

            if (device_type != "opencl" and
                    (opencl_device != "cpu" or opencl_algo != "0")):
                if opencl_device != "fpga":# or not isde1:
                    keepCombi = False

            if tracking_regime == "default" and int(no_hierarchy_levels) >= 1:
                keepCombi = False

            if float(mu) <= float(voxel_size):
                keepCombi = False

            if keepCombi: final_combinations.append(c)

        print(str(len(final_combinations)) + " combination" + ("s" if len(final_combinations) != 1 else ""))

        # Run experiments
        if len(final_combinations) > 0:

            # Log file
            logfile, previousLog = startLogFile(params_log, discard_log=False)

            if len(previousLog) == 0:
                logfile.write("ID," + ",".join(all_parameters_ids))
                if groundtruth_root:
                    logfile.write(",ate,time")
                logfile.write("\n")

            previousID = [x.split(",")[0] for x in previousLog]

            for design_id, params in enumerate(final_combinations):

                if str(design_id) in previousID: continue

                # Modify the parameter file
                modifyProgramParameters(working_dir, parameters_filename, all_parameters_ids, list(params))

                errorLog = open("error.log", "at")
                try:
                    # Compile and run the program
                    program_output = compileRunParseProgram(program_exe, program_options, compile_command, working_dir, isde1)

                    # Export the results
                    saveProgramOutput(program_output, output_dir, design_id)
                    saveProgramOutputFiles(working_dir, output_dir, design_id)

                    logfile.write(str(design_id) + "," + ",".join(params))
                    if groundtruth_root:
                        results = processProgramOutputFiles(design_id, output_dir, groundtruth_root)
                        logfile.write("," + ",".join(map(str, results)))
                    logfile.write("\n")
                    logfile.flush()
                except:
                    logfile.write("\n")
                    errorLog.write(str(design_id) + "\n")



    ### Read options for OpenCL experiments
    if args.opencl:
        config = configparser.ConfigParser()
        config.read(args.opencl)
        
        template_filename = config.get('FPGA', 'template_filename')
        knob_filename     = config.get('FPGA', 'knob_filename')
        num_kernels       = int(config.get('FPGA', 'num_kernels'))
        log_filename      = config.get('FPGA', 'log_file')
        discard_log       = (config.get('FPGA', 'discard_log') != "0")
        fpga_filenames = []
        benchmark_dirs = []
        aocx_filenames = []
        aocx_out_names = []
        algorithm_ids  = []
        for i in range(num_kernels):
            fpga_filenames.append(config.get('FPGA', 'fpga_filename' + str(i+1)))
            benchmark_dirs.append(config.get('FPGA', 'benchmarks_path' + str(i+1)))
            aocx_filenames.append(config.get('FPGA', 'aocx_filename' + str(i+1)))
            aocx_out_names.append(config.get('FPGA', 'aocx_link_filename' + str(i+1)))
            algorithm_ids.append(config.get('FPGA', 'algorithm_id' + str(i+1)))



    ### OpenCL experiments
    if args.opencl:

        # Log file
        logfile, previous_log = startLogFile(log_filename, discard_log)
        logfile.flush()

        readers = []
        # Read the CSV files for each kernel
        for k in range(num_kernels):
            reader = ResultsReader(fpga_filenames[k], "fpga")
            reader.removeBadDesigns()
            reader.addIdInt()
            readers.append(reader)

        cl_experiments_kernel_id = list(range(num_kernels))

        # For each kernel that we want to try
        for k in cl_experiments_kernel_id:
            # For each design of that kernel
            for i in range(readers[k].getNumDesigns()):

                knob_values = []
                for k2 in range(num_kernels):
                    if k != k2:
                        knob_values += readers[k2].getDesignKnobs(0) # Get default knob values for other kernels
                    else:
                        knob_values += readers[k].getDesignKnobs(i) # Get knob values for this kernel
                
                
                design_id = os.path.basename(readers[k].getDesignID(i))

                # Skip if we already tested this design
                if design_id in previous_log: continue

                print('Testing "' + design_id + '"')
                
                # Create a link to the AOCX file
                aocx_path = os.path.join(benchmark_dirs[k], design_id, aocx_filenames[k])
                aocx_link = os.path.join(working_dir, aocx_out_names[k])


                symlink_force(aocx_path, aocx_link)
                
                # Write the generated knob file
                knobfile_text = createTextFromTemplate(template_filename, knob_values, 1)
                with open(knob_filename, 'wt') as fout:
                    fout.write(knobfile_text)

                # Modify the parameter file
                modifyProgramParameters(working_dir, parameters_filename, ["opencl_algo"], [algorithm_ids[k]])

                # Compile and run the program
                program_output = compileRunParseProgram(program_exe, program_options, compile_command, working_dir, isde1)

                # Export the results
                try:
                    saveProgramOutputFiles(working_dir, output_dir, design_id)
                    saveProgramOutput(program_output, output_dir, design_id)

                    # Log result
                    logfile.write(design_id + "\n")
                except:
                    pass
                logfile.flush()


if __name__ == "__main__":
    main()



