# Imports
import csv
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from align import align_sim3

# Global variables
list_slam_timestamp_1 = []
list_slam_x_1 = []
list_slam_y_1 = []
list_slam_z_1 = []

list_leica_timestamp_1 = []
list_leica_x_1 = []
list_leica_y_1 = []
list_leica_z_1 = []

list_slam_timestamp_2 = []
list_slam_x_2 = []
list_slam_y_2 = []
list_slam_z_2 = []

list_leica_timestamp_2 = []
list_leica_x_2 = []
list_leica_y_2 = []
list_leica_z_2 = []

# Add data points to the corresponding lists
with open('export_0.txt', 'rt') as csvfile:
  reader = csv.reader(csvfile, delimiter=';')
  for row in reader:
    if int(row[1]) == 0:
      list_slam_timestamp_1.append(1000*float(row[2]))
      list_slam_x_1.append(float(row[3]))
      list_slam_y_1.append(float(row[4]))
      list_slam_z_1.append(float(row[5]))
      #print(row[3] + ", " + row[4] + ", " + row[5])
    elif int(row[1]) == 1:
      list_slam_timestamp_2.append(1000*float(row[2]))
      list_slam_x_2.append(float(row[3]))
      list_slam_y_2.append(float(row[4]))
      list_slam_z_2.append(float(row[5]))
      #print(row[3] + ", " + row[4] + ", " + row[5])

with open('export_leica.csv', 'rt') as csvfile:
  reader = csv.reader(csvfile, delimiter=';')
  for row in reader:
    if int(row[0]) == 0:
      list_leica_timestamp_1.append(1000*(float(row[1])) + 32.8)
      list_leica_x_1.append(float(row[2]))
      list_leica_y_1.append(float(row[3]))
      list_leica_z_1.append(float(row[4]))
      #print(row[3] + ", " + row[4] + ", " + row[5])
    elif int(row[0]) == 1:
      list_leica_timestamp_2.append(1000*(float(row[1])) + 32.8)
      list_leica_x_2.append(float(row[2]))
      list_leica_y_2.append(float(row[3]))
      list_leica_z_2.append(float(row[4]))

#array_x = np.array(list_slam_x_1)
#array_y = np.array(list_slam_y_1)
#array_z = np.array(list_slam_z_1)

# Print nr. of data points
print("map 1 nr of data points in slam_1: " + str(len(list_slam_x_1)))
print("map 2 nr of data points in slam_2: " + str(len(list_slam_x_2)))

print("map 1 nr of data points in leica_1: " + str(len(list_leica_x_1)))
print("map 2 nr of data points in leica_2: " + str(len(list_leica_x_2)))

array_leica_1_timestamp = np.array(list_leica_timestamp_1)
array_leica_1_x = np.array(list_leica_x_1)
array_leica_1_y = np.array(list_leica_y_1)
array_leica_1_z = np.array(list_leica_z_1)

leica_coordinates_1 = np.vstack((array_leica_1_timestamp, array_leica_1_x))
leica_coordinates_1 = np.vstack((leica_coordinates_1, array_leica_1_y))
leica_coordinates_1 = np.vstack((leica_coordinates_1, array_leica_1_z))
leica_coordinates_1 = leica_coordinates_1.transpose()

array_slam_1_timestamp = np.array(list_slam_timestamp_1)
array_slam_1_x = np.array(list_slam_x_1)
array_slam_1_y = np.array(list_slam_y_1)
array_slam_1_z = np.array(list_slam_z_1)

slam_coordinates_1 = np.vstack((array_slam_1_timestamp, array_slam_1_x))
slam_coordinates_1 = np.vstack((slam_coordinates_1, array_slam_1_y))
slam_coordinates_1 = np.vstack((slam_coordinates_1, array_slam_1_z))
slam_coordinates_1 = slam_coordinates_1.transpose()

# Transform SLAM points according to calibration transformation
R_cam_marker = np.array([[0.99858239, -0.00736774, -0.05271548],
                         [0.00682177, 0.99992129, -0.01052936],
                         [0.05278891, 0.01015482, 0.99855406]])
t_cam_marker = np.array([-0.00604897, 0.05317244, 0.04223419])
R_marker_cam = R_cam_marker.transpose()
t_marker_cam = -t_cam_marker
slam_coordinates_1[:,1:4] = np.transpose(np.dot(R_marker_cam, np.transpose(slam_coordinates_1[:,1:4]))) + t_marker_cam


# Sort timings
slam_coordinates_1 = slam_coordinates_1[slam_coordinates_1[:, 0].argsort()]
#slam_coordinates_1 = slam_coordinates_1[np.argsort(slam_coordinates_1[:, 0])]
#slam_coordinates_1 = slam_coordinates_1[np.lexsort(slam_coordinates_1[:, 0])]

# Add Gaussian noise to first trajectory
#print("mean slam 1 map: " + str(np.mean(slam_coordinates_1[:,1:4], axis=0)) + ", std: " + str(np.std(slam_coordinates_1[:,1:4], axis=0)))
#slam_coordinates_1 = slam_coordinates_1 + np.hstack((np.zeros(slam_coordinates_1[:,0:1].shape), np.random.normal(loc=0.0, scale=0.15, size=slam_coordinates_1[:, 1:4].shape)))
#print("mean slam 1 map w. noise: " + str(np.mean(slam_coordinates_1[:,1:4], axis=0)) + ", std: " + str(np.std(slam_coordinates_1[:,1:4], axis=0)))

array_leica_2_timestamp = np.array([list_leica_timestamp_2])
array_leica_2_x = np.array(list_leica_x_2)
array_leica_2_y = np.array(list_leica_y_2)
array_leica_2_z = np.array(list_leica_z_2)

leica_coordinates_2 = np.vstack((array_leica_2_timestamp, array_leica_2_x))
leica_coordinates_2 = np.vstack((leica_coordinates_2, array_leica_2_y))
leica_coordinates_2 = np.vstack((leica_coordinates_2, array_leica_2_z))
leica_coordinates_2 = leica_coordinates_2.transpose()

array_slam_2_timestamp = np.array([list_slam_timestamp_2])
array_slam_2_x = np.array(list_slam_x_2)
array_slam_2_y = np.array(list_slam_y_2)
array_slam_2_z = np.array(list_slam_z_2)

slam_coordinates_2 = np.vstack((array_slam_2_timestamp, array_slam_2_x))
slam_coordinates_2 = np.vstack((slam_coordinates_2, array_slam_2_y))
slam_coordinates_2 = np.vstack((slam_coordinates_2, array_slam_2_z))
slam_coordinates_2 = slam_coordinates_2.transpose()

# Transform SLAM points according to calibration transformation
slam_coordinates_2[:,1:4] = np.transpose(np.dot(R_marker_cam, np.transpose(slam_coordinates_2[:,1:4]))) + t_marker_cam

# Sort timings
slam_coordinates_2 = slam_coordinates_2[slam_coordinates_2[:, 0].argsort()]

# Matching of the datasets according to their time stamps
leica_1 = np.zeros((len(list_leica_x_1), 4))
slam_1 = np.zeros((len(list_slam_x_1), 4))

leica_2 = np.zeros((len(list_leica_x_2), 4))
slam_2 = np.zeros((len(list_slam_x_2), 4))

pos = 0
offset = 1
break_flag = False
for i in range(len(list_leica_x_1)):
  for j in range(offset, len(list_slam_x_1)):
    # value where ekf time > vicon time
    if leica_coordinates_1[i,0] < slam_coordinates_1[j,0]:
      # check the one before
      if abs(leica_coordinates_1[i,0] - slam_coordinates_1[j-1,0]) < 5:
        leica_1[pos] = leica_coordinates_1[i]
        slam_1[pos] = slam_coordinates_1[j - 1]
        pos = pos + 1
        if pos >= len(slam_coordinates_1):
          break_flag = True
          break
        offset = j
        continue
  if break_flag==True:
    break

pos = 0
offset = 1
break_flag = False
for i in range(len(list_leica_x_2)):
  for j in range(offset, len(list_slam_x_2)):
    # value where ekf time > vicon time
    if leica_coordinates_2[i,0] < slam_coordinates_2[j,0]:
      # check the one before
      if abs(leica_coordinates_2[i,0] - slam_coordinates_2[j-1,0]) < 5:
        leica_2[pos] = leica_coordinates_2[i]
        slam_2[pos] = slam_coordinates_2[j - 1]
        pos = pos + 1
        if pos >= len(slam_coordinates_2):
          break_flag = True
          break
        offset = j
        continue
  if break_flag==True:
    break

# Remove empty rows
slam_1 = slam_1[~(slam_1 == 0).all(1)]
leica_1 = leica_1[~(leica_1 == 0).all(1)]

slam_2 = slam_2[~(slam_2 == 0).all(1)]
leica_2 = leica_2[~(leica_2 == 0).all(1)]

# Get transformation for alignement
s_1, R_gt_es_1, gt_t_gt_es_1 = align_sim3(leica_1[0:50,1:4], slam_1[0:50,1:4])
# Transform slam output
slam_1_transformed = s_1 * np.transpose(np.dot(R_gt_es_1, np.transpose(slam_1[:,1:4]))) + gt_t_gt_es_1
slam_1_orig_transformed = s_1 * np.transpose(np.dot(R_gt_es_1, np.transpose(slam_coordinates_1[:,1:4]))) + gt_t_gt_es_1

# Get transformation for alignement
#s_2, R_gt_es_2, gt_t_gt_es_2 = align_sim3(leica_2[0:80,1:4], slam_2[0:80,1:4])
# Transform slam output
slam_2_transformed = s_1 * np.transpose(np.dot(R_gt_es_1, np.transpose(slam_2[:,1:4]))) + gt_t_gt_es_1
slam_2_orig_transformed = s_1 * np.transpose(np.dot(R_gt_es_1, np.transpose(slam_coordinates_2[:,1:4]))) + gt_t_gt_es_1

# Calculate rmse
var_slam_1 = []

for i in range(len(slam_1_transformed)):
  var_slam_1.append(((leica_1[i, 1] - slam_1_transformed[i, 0])**2 + (leica_1[i, 2] - slam_1_transformed[i, 1])**2 + (leica_1[i, 3] - slam_1_transformed[i, 2])**2))

var_slam_2 = []

for i in range(len(slam_2_transformed)):
  var_slam_2.append(((leica_2[i, 1] - slam_2_transformed[i, 0])**2 + (leica_2[i, 2] - slam_2_transformed[i, 1])**2 + (leica_2[i, 3] - slam_2_transformed[i, 2])**2))

print("SLAM Map 1 rmse = {0}".format(np.sqrt(1 / (len(var_slam_1) - 1) * sum(var_slam_1))))
print("SLAM Map 2 rmse = {0}".format(np.sqrt(1 / (len(var_slam_2) - 1) * sum(var_slam_2))))

# Plot data points
fig = plt.figure()
fig.suptitle('Choosen SLAM points Map 1')
ax1 = fig.add_subplot(111, projection='3d')
#ax1 = fig.gca(projection='3d')
ax1.set_xlabel('X')
ax1.set_ylabel('Y')
ax1.set_zlabel('Z')
legend_all = ax1.scatter(slam_1_orig_transformed[:,0], slam_1_orig_transformed[:,1], slam_1_orig_transformed[:,2], c='orange', label='All SLAM points')
#legend_all = ax1.scatter(slam_coordinates_1[:,1], slam_coordinates_1[:,2], slam_coordinates_1[:,3], c='orange', label='All SLAM points')
for i in range(0, 1):
  #ax1.annotate('%s' % slam_coordinates_1[i,0], xy=(slam_coordinates_1[i,1], slam_coordinates_1[i,2]), textcoords='data')
  ax1.text(slam_1_orig_transformed[i,0], slam_1_orig_transformed[i,1], slam_1_orig_transformed[i,2], '%.2f' % (slam_coordinates_1[i,0]), size=20, zorder=1)
#legend_all = ax1.plot(slam_1_orig_transformed[0:50,0], slam_1_orig_transformed[0:50,1], slam_1_orig_transformed[0:50,2], c='orange', label='All SLAM points')
legend_matched = ax1.scatter(slam_1_transformed[:,0], slam_1_transformed[:,1], slam_1_transformed[:,2], c='blue', s=100, label='Matched SLAM points')
#legend_matched = ax1.scatter(slam_1[:,1], slam_1[:,2], slam_1[:,3], c='blue', s=100, label='Matched SLAM points')
#for i in range(len(slam_1_transformed[:,0])):
for i in range(len(slam_1[:,1])):
  if i%4 == 0:
    ax1.text(slam_1_transformed[i,0], slam_1_transformed[i,1], slam_1_transformed[i,2], '%s' % (i), size=20, zorder=1)
#legend_matched = ax1.plot(slam_1_transformed[0:50,0], slam_1_transformed[0:50,1], slam_1_transformed[0:50,2], c='blue', label='Matched SLAM points')
#ax1.plot(slam_coordinates_1[:,1], slam_coordinates_1[:,2], slam_coordinates_1[:,3], c='blue')
# Make scaling equal
max_range = np.array([slam_1_transformed[:,0].max()-slam_1_transformed[:,0].min(), slam_1_transformed[:,1].max()-slam_1_transformed[:,1].min(), slam_1_transformed[:,2].max()-slam_1_transformed[:,2].min()]).max() / 2.0
mid_x = (slam_1_transformed[:,0].max()+slam_1_transformed[:,0].min()) * 0.5
mid_y = (slam_1_transformed[:,1].max()+slam_1_transformed[:,1].min()) * 0.5
mid_z = (slam_1_transformed[:,2].max()+slam_1_transformed[:,2].min()) * 0.5
ax1.set_xlim(mid_x - max_range, mid_x + max_range)
ax1.set_ylim(mid_y - max_range, mid_y + max_range)
ax1.set_zlim(mid_z - max_range, mid_z + max_range)
plt.legend(handles=[legend_all, legend_matched])

fig = plt.figure()
fig.suptitle('Choosen SLAM points Map 2')
ax1 = fig.add_subplot(111, projection='3d')
#ax1 = fig.gca(projection='3d')
ax1.set_xlabel('X')
ax1.set_ylabel('Y')
ax1.set_zlabel('Z')
ax1.scatter(slam_2_orig_transformed[:,0], slam_2_orig_transformed[:,1], slam_2_orig_transformed[:,2], c='orange')
for i in range(1):
  ax1.text(slam_2_orig_transformed[i,0], slam_2_orig_transformed[i,1], slam_2_orig_transformed[i,2], '%.2f' % (slam_coordinates_2[i,0]), size=20, zorder=1)
ax1.scatter(slam_2_transformed[:,0], slam_2_transformed[:,1], slam_2_transformed[:,2], c='blue', s=100)
#ax1.plot(slam_coordinates_2[:,1], slam_coordinates_2[:,2], slam_coordinates_2[:,3], c='blue')
# Make scaling equal
max_range = np.array([slam_2_transformed[:,0].max()-slam_2_transformed[:,0].min(), slam_2_transformed[:,1].max()-slam_2_transformed[:,1].min(), slam_2_transformed[:,2].max()-slam_2_transformed[:,2].min()]).max() / 2.0
mid_x = (slam_2_transformed[:,0].max()+slam_2_transformed[:,0].min()) * 0.5
mid_y = (slam_2_transformed[:,1].max()+slam_2_transformed[:,1].min()) * 0.5
mid_z = (slam_2_transformed[:,2].max()+slam_2_transformed[:,2].min()) * 0.5
ax1.set_xlim(mid_x - max_range, mid_x + max_range)
ax1.set_ylim(mid_y - max_range, mid_y + max_range)
ax1.set_zlim(mid_z - max_range, mid_z + max_range)

fig = plt.figure()
fig.suptitle('Choosen Leica points Map 1')
ax2 = fig.add_subplot(111, projection='3d')
ax2.set_xlabel('X')
ax2.set_ylabel('Y')
ax2.set_zlabel('Z')
legend_all = ax2.scatter(list_leica_x_1[:], list_leica_y_1[:], list_leica_z_1[:], c='green', label='All leica points')
for i in range(0, 1):
  ax2.text(list_leica_x_1[i], list_leica_y_1[i], list_leica_z_1[i], '%s' % (list_leica_timestamp_1[i]), size=20, zorder=1)
legend_matched = ax2.scatter(leica_1[:,1], leica_1[:,2], leica_1[:,3], c='red', s=100, label='Matched leica points')
for i in range(len(slam_1_transformed[:,0])):
  if i%4 == 0:
    ax2.text(leica_1[i,1], leica_1[i,2], leica_1[i,3], '%s' % (i), size=20, zorder=1)
#legend_all = ax2.plot(list_leica_x_1[0:5*50], list_leica_y_1[0:5*50], list_leica_z_1[0:5*50], c='green', label='All leica points')
#legend_matched = ax2.plot(leica_1[0:50,1], leica_1[0:50,2], leica_1[0:50,3], c='red', label='Matched leica points')
# Make scaling equal
max_range = np.array([leica_1[:,1].max()-leica_1[:,1].min(), leica_1[:,2].max()-leica_1[:,2].min(), leica_1[:,3].max()-leica_1[:,3].min()]).max() / 2.0
mid_x = (leica_1[:,1].max()+leica_1[:,1].min()) * 0.5
mid_y = (leica_1[:,2].max()+leica_1[:,2].min()) * 0.5
mid_z = (leica_1[:,3].max()+leica_1[:,3].min()) * 0.5
ax2.set_xlim(mid_x - max_range, mid_x + max_range)
ax2.set_ylim(mid_y - max_range, mid_y + max_range)
ax2.set_zlim(mid_z - max_range, mid_z + max_range)
plt.legend(handles=[legend_all, legend_matched])

fig = plt.figure()
fig.suptitle('Choosen Leica points Map 2')
ax2 = fig.add_subplot(111, projection='3d')
ax2.set_xlabel('X')
ax2.set_ylabel('Y')
ax2.set_zlabel('Z')
legend_all = ax2.scatter(list_leica_x_2, list_leica_y_2, list_leica_z_2, c='green', label='All leica points')
legend_matched = ax2.scatter(leica_2[:,1], leica_2[:,2], leica_2[:,3], c='red', s=100, label='Matched leica points')
# Make scaling equal
max_range = np.array([leica_2[:,1].max()-leica_2[:,1].min(), leica_2[:,2].max()-leica_2[:,2].min(), leica_2[:,3].max()-leica_2[:,3].min()]).max() / 2.0
mid_x = (leica_2[:,1].max()+leica_2[:,1].min()) * 0.5
mid_y = (leica_2[:,2].max()+leica_2[:,2].min()) * 0.5
mid_z = (leica_2[:,3].max()+leica_2[:,3].min()) * 0.5
ax2.set_xlim(mid_x - max_range, mid_x + max_range)
ax2.set_ylim(mid_y - max_range, mid_y + max_range)
ax2.set_zlim(mid_z - max_range, mid_z + max_range)
plt.legend(handles=[legend_all, legend_matched])

fig = plt.figure()
fig.suptitle('Leica and SLAM transformed Map 1')
ax3 = fig.add_subplot(111, projection='3d')
ax3.set_xlabel('X')
ax3.set_ylabel('Y')
ax3.set_zlabel('Z')
legend_slam = ax3.scatter(slam_1_transformed[:,0], slam_1_transformed[:,1], slam_1_transformed[:,2], c='blue', label='SLAM transformed')
"""
for i in range(len(slam_1_transformed[:,1])):
  if i%4 == 0:
    ax3.text(slam_1_transformed[i,0], slam_1_transformed[i,1], slam_1_transformed[i,2], '%s' % (i), size=20, zorder=1)
"""
#legend_slam = ax3.plot(slam_1_transformed[:100,0], slam_1_transformed[0:100,1], slam_1_transformed[0:100,2], c='blue', label='SLAM transformed')
#legend_slam = ax3.scatter(slam_1[:,0], slam_1[:,1], slam_1[:,2], c='blue', label='SLAM transformed')
legend_leica = ax3.scatter(leica_1[:,1], leica_1[:,2], leica_1[:,3], c='red', label='Leica')
"""
for i in range(len(leica_1[:,1])):
  if i%4 == 0:
    ax3.text(leica_1[i,1], leica_1[i,2], leica_1[i,3], '%s' % (i), size=20, zorder=1)
"""
#legend_leica = ax3.plot(leica_1[0:100,1], leica_1[0:100,2], leica_1[0:100,3], c='red', label='Leica')

# Make scaling equal
max_range = np.array([leica_1[:,1].max()-leica_1[:,1].min(), leica_1[:,2].max()-leica_1[:,2].min(), leica_1[:,3].max()-leica_1[:,3].min()]).max() / 2.0
mid_x = (leica_1[:,1].max()+leica_1[:,1].min()) * 0.5
mid_y = (leica_1[:,2].max()+leica_1[:,2].min()) * 0.5
mid_z = (leica_1[:,3].max()+leica_1[:,3].min()) * 0.5
ax3.set_xlim(mid_x - max_range, mid_x + max_range)
ax3.set_ylim(mid_y - max_range, mid_y + max_range)
ax3.set_zlim(mid_z - max_range, mid_z + max_range)

plt.legend(handles=[legend_slam, legend_leica])


fig = plt.figure()
fig.suptitle('Leica and SLAM transformed Map 2')
ax4 = fig.add_subplot(111, projection='3d')
ax4.scatter(slam_2_transformed[:,0], slam_2_transformed[:,1], slam_2_transformed[:,2], c='blue')
#ax4.plot(slam_2_transformed[0:100,0], slam_2_transformed[0:100,1], slam_2_transformed[0:100,2], c='blue')
#ax4.scatter(slam_2[:,0], slam_2[:,1], slam_2[:,2], c='blue')
ax4.scatter(leica_2[:,1], leica_2[:,2], leica_2[:,3], c='red')
#ax4.plot(leica_2[0:100,1], leica_2[0:100,2], leica_2[0:100,3], c='red')
# Make scaling equal
max_range = np.array([leica_2[:,1].max()-leica_2[:,1].min(), leica_2[:,2].max()-leica_2[:,2].min(), leica_2[:,3].max()-leica_2[:,3].min()]).max() / 2.0
mid_x = (leica_2[:,1].max()+leica_2[:,1].min()) * 0.5
mid_y = (leica_2[:,2].max()+leica_2[:,2].min()) * 0.5
mid_z = (leica_2[:,3].max()+leica_2[:,3].min()) * 0.5
ax4.set_xlim(mid_x - max_range, mid_x + max_range)
ax4.set_ylim(mid_y - max_range, mid_y + max_range)
ax4.set_zlim(mid_z - max_range, mid_z + max_range)

plt.show()

test = 5

# Plot variance of ekf
"""
plt.figure(2)
plt.title('Variance of SLAM for Map 1')
plt.plot(slam_1[:, 0], var_slam_1)

plt.figure(3)
plt.title('Variance of SLAM for Map 2')
plt.plot(slam_2[:, 0], var_slam_2)

plt.show()
"""
