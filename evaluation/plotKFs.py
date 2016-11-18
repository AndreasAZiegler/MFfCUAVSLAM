import csv
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

list_timestamp_1 = []
list_x_1 = []
list_y_1 = []
list_z_1 = []

list_timestamp_2 = []
list_x_2 = []
list_y_2 = []
list_z_2 = []

with open('export_0.txt', 'rt') as csvfile:
  reader = csv.reader(csvfile, delimiter=';')
  for row in reader:
    if int(row[1]) == 0:
      list_timestamp_1.append(row[2])
      list_x_1.append(float(row[3]))
      list_y_1.append(float(row[4]))
      list_z_1.append(float(row[5]))
      #print(row[3] + ", " + row[4] + ", " + row[5])
    elif int(row[1]) == 1:
      list_timestamp_2.append(row[2])
      list_x_2.append(float(row[3]))
      list_y_2.append(float(row[4]))
      list_z_2.append(float(row[5]))
      #print(row[3] + ", " + row[4] + ", " + row[5])

#array_x = np.array(list_x_1)
#array_y = np.array(list_y_1)
#array_z = np.array(list_z_1)

print("map 1 nr of data points: " + str(len(list_x_1)))
print("map 2 nr of data points: " + str(len(list_x_2)))

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(list_x_1, list_y_1, list_z_1, c='blue')
ax.scatter(list_x_2, list_y_2, list_z_2, c='red')
plt.show()
