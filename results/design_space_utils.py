import pandas as pd
import numpy as np
import os


def normalize(values):
    return (values - values.min()) / (values.max() - values.min())



def read_design_space(platform=None, version=None, benchmark=None, algorithm=None, time='Total_time', filename=None):
    
    if filename is None:
        dirname = os.path.join(platform, version, benchmark)
        basename = algorithm + '.csv'
        filename = os.path.join(dirname, basename)
        
    df = pd.read_csv(filename)
    
    max_logic = 234720 # Stratix V on DE5
    if 'de1' in filename.lower(): max_logic = 32070 # Cyclone V on DE1
    
    df['logic_utilization'] = df['logic'] / max_logic
    df['logic_utilization_percent'] = df['logic_utilization'] * 100
    
    if 'ate' in df:
        df['ate_cm'] = df['ate'] * 100
    
    df['FPS'] = 1000/df[time]
    
    if 'Total_time' in df:
        df['Total FPS'] = 1000/df['Total_time']
    
    return df
