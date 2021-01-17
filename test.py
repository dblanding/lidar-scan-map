"""Development tool for feeding scan data to processScan"""

import pickle
import pprint
import proscan

with open('scan_data.pkl', 'rb') as file:
    data = pickle.load(file)

pscan = proscan.ProcessScan(data, fit=3)
pprint.pprint(pscan.get_line_parameters())
pscan.map(nmbr=1, display_all_points=True)
