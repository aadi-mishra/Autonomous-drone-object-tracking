import gmplot
import csv
import numpy as np

# Reading the CSV File containing GPS coordinates
f = open('C:\\Users\\AADI\\Documents\\Mission Planner\\logs\\QUADROTOR\\1\\gps_raw.csv')
csv_f = csv.reader(f)

lat = []
lon = []
coords = []
for row in csv_f:
    coord = {}
    coord["lat"] = row[2]
    coord["lng"] = row[3]
    # lat.append(row[2])
    # lon.append(row[3])
    coords.append(coord)

print(coords)

# print(len(lat))
# print(len(lon))

# lat = list(map(float,lat))
# lon = list(map(float,lon))

# print(len(lat))
# print(len(lon))
# '''
# def batch(iterable, n = 1):
#    current_batch = []
  
#    for item in iterable:
#        current_batch.append(item)
#        if len(current_batch) == n:
#            yield current_batch
#            current_batch = []
#    if current_batch:
#        yield current_batch


# '''
# gmap = gmplot.GoogleMapPlotter(lat[0], lon[0], 30)


# gmap.plot(lat,lon, 'cornflowerblue', edge_width=2)

# gmap.draw("my_map.html")

# f.close()

