import map_scan_data
import ztcar
import time

zt = ztcar.ZTCar()
data_list = []
for n in range(4):
    scan_data = zt.scan()
    print(scan_data[0])
    print("Number of data points: ", len(scan_data))
    scan_data.pop(0)
    print(scan_data[0])
    print("Number of data points: ", len(scan_data))
    data_list.append(scan_data)
    time.sleep(2)
for n, scan_data in enumerate(data_list):
    map_scan_data.show_map(scan_data, n+1)
