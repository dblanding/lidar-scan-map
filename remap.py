"""
Development tool for remapping scan data
"""
from pathlib import Path
import pickle
import pprint
import proscan

def remap(nmbr, verbose=True, display=True):
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
    pscan = proscan.ProcessScan(data, gap=5, fit=2)
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
    # plot all datafiles
    plot_all()  # can take about 30 seconds to run

    # for individual interactive plots:
    #nmbr = input("Enter number of data to load: ")
    #remap(nmbr)
