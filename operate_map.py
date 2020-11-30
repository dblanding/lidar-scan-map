import pickle
import map_scan_data

with open('scan_data.pkl', 'rb') as f:
    data_list = pickle.load(f)

for n, scan_data in enumerate(data_list):
    map_scan_data.show_map(scan_data, n+1)
