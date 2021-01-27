"""
Development tool for remapping scan data
"""

import pickle
import pprint
import proscan

def remap(nmbr):
    """
    Read saved datafile 'nmbr' and replot map
    """
    filename = f'Data/scan_data{nmbr}.pkl'
    with open(filename, 'rb') as file:
        data = pickle.load(file)
    pprint.pprint(data)
    pscan = proscan.ProcessScan(data, gap=5, fit=2)
    #pprint.pprint(pscan.get_line_parameters())
    pscan.map(seq_nmbr=nmbr, display_all_points=True, show=True)

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
    nmbr = input("Enter number of data to load: ")
    remap(nmbr)
