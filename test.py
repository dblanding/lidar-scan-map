"""Development tool for feeding scan data to processScan"""

import pickle
import pprint
import proscan

with open('scan_data.pkl', 'rb') as file:
    data = pickle.load(file)

pscan = proscan.ProcessScan(data, fit=3)
pprint.pprint(pscan.get_line_parameters())
pscan.map(nmbr=1, display_all_points=True)

def function_name(arguments):
    """
    1. Description of what the function does.
    2. Description of the arguments, if any.
    3. Description of the return value(s), if any.
    4. Description of errors, if any.
    5. Optional extra notes or examples of usage.
    """
    return None

