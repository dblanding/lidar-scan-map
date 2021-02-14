"""
Development tool for remapping scan data
"""
import math
from pathlib import Path
import pickle
import pprint
import proscan

def remap(nmbr, verbose=False, display=True):
    """
    Load saved datafile by file 'nmbr'

    if verbose: pretty print data
    if display: display interactive plot
    """
    filename = f'Data/scan_data{nmbr}.pkl'
    with open(filename, 'rb') as file:
        data = pickle.load(file)
    if verbose:
        pprint.pprint(data)
    pscan = proscan.ProcessScan(data, lev=5000, gap=10, fit=4)
    if verbose:
        print(f"Regions: {pscan.regions}")
    pscan.map(seq_nmbr=nmbr, display_all_points=True, show=display)

def plot_all():
    """
    Load each .pkl data file in Data/ folder, generate plot,
    save as correspondingly numbered .png image in Maps/ folder.
    """
    p = Path('Data')
    pathlist = list(p.glob('*.pkl'))
    for f in pathlist:
        fname = f.stem
        nmbr_str = fname.rpartition('data')[-1]
        remap(nmbr_str, verbose=False, display=False)

def polar_coords():
    """
    Get polar coords of all the points in scan.
    """
    nmbr = input("Enter 'all' or integer number of data to load: ")
    if nmbr.isnumeric():
        filename = f'Data/scan_data{nmbr}.pkl'
        with open(filename, 'rb') as file:
            data = pickle.load(file)
        pscan = proscan.ProcessScan(data)
        #for point in pscan.points:
            #print(f"{point.get('theta')*180/math.pi}, {point.get('dist')}")
        rvals = [point.get('dist')
                 for point in pscan.points
                 if point.get('dist') != 3]
        print(sum(rvals)/len(rvals))
    
def function_name(arguments):
    """
    1. Description of what the function does.
    2. Description of the arguments, if any.
    3. Description of the return value(s), if any.
    4. Description of errors, if any.
    5. Optional extra notes or examples of usage.
    """
    return None

if __name__ == '__main__':
    '''
    nmbr = input("Enter 'all' or integer number of data to load: ")
    if nmbr == 'all':
        # plot all datafiles
        plot_all()  # can take about 30 seconds to run
    elif nmbr.isnumeric():
        # generate individual interactive plot
        remap(nmbr)
    '''
    polar_coords()
