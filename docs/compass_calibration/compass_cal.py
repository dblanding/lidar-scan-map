
cd = {}
with open('compass-data.csv') as file:
    lines = [line.rstrip().split(',') for line in file]
    for v, k in lines:
        cd[int(k)] = int(v)
print(cd)
