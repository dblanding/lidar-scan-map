import map_scan_data
import omnicar
import time
import pickle

car = omnicar.OmniCar()
data_list = []
# Collect 4 scans
for n in range(4):
    scan_data = car.scan()
    print(scan_data[0])
    print("Number of data points: ", len(scan_data))
    scan_data.pop(0)
    print(scan_data[0])
    print("Number of data points: ", len(scan_data))
    data_list.append(scan_data)
    time.sleep(3)
with open('scan_data.pkl', 'wb') as f:
    pickle.dump(data_list, f)

# map the scans
for n, scan_data in enumerate(data_list):
    map_scan_data.show_map(scan_data, n+1)
'''
with open('scan_data.pkl', 'rb') as f:
    data_list = pickle.load(f)
'''
