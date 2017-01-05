# Imports
import csv
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from align import align_sim3

def open_slam():
  list_slam_timestamp_1 = []
  list_slam_x_1 = []
  list_slam_y_1 = []
  list_slam_z_1 = []

  list_slam_timestamp_2 = []
  list_slam_x_2 = []
  list_slam_y_2 = []
  list_slam_z_2 = []

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
  return list_slam_timestamp_1, list_slam_x_1, list_slam_y_1, list_slam_z_1, list_slam_timestamp_2, list_slam_x_2, list_slam_y_2, list_slam_z_2

def open_ground_truth():
  list_gt_timestamp_1 = []
  list_gt_x_1 = []
  list_gt_y_1 = []
  list_gt_z_1 = []

  list_gt_timestamp_2 = []
  list_gt_x_2 = []
  list_gt_y_2 = []
  list_gt_z_2 = []

  with open('export_leica.csv', 'rt') as csvfile:
    reader = csv.reader(csvfile, delimiter=';')
    for row in reader:
      if int(row[0]) == 0:
        # Rack
        #list_gt_timestamp_1.append(1000*(float(row[1])) + 32.8)
        # UAV
        #list_gt_timestamp_1.append(1000*(float(row[1])) + 41.8)
        list_gt_timestamp_1.append(1000 * (float(row[1])))
        list_gt_x_1.append(float(row[2]))
        list_gt_y_1.append(float(row[3]))
        list_gt_z_1.append(float(row[4]))
        #print(row[3] + ", " + row[4] + ", " + row[5])
      elif int(row[0]) == 1:
        # Rack
        #list_gt_timestamp_2.append(1000*(float(row[1])) + 32.8)
        # UAV
        #list_gt_timestamp_2.append(1000*(float(row[1])) + 41.8)
        list_gt_timestamp_2.append(1000 * (float(row[1])))
        list_gt_x_2.append(float(row[2]))
        list_gt_y_2.append(float(row[3]))
        list_gt_z_2.append(float(row[4]))
  return list_gt_timestamp_1, list_gt_x_1, list_gt_y_1, list_gt_z_1, list_gt_timestamp_2, list_gt_x_2, list_gt_y_2, list_gt_z_2

def time_offset(list_gt_timestamp, offset):
  for i in range(len(list_gt_timestamp)):
    list_gt_timestamp[i] += offset
    #print("list_gt_timestamp[i] = {0}".format(list_gt_timestamp[i]))
  return list_gt_timestamp

def create_array(list_timestamp, list_x, list_y, list_z):
  array_timestamp = np.array(list_timestamp)
  array_x = np.array(list_x)
  array_y = np.array(list_y)
  array_z = np.array(list_z)

  array = np.vstack((array_timestamp, array_x))
  array = np.vstack((array, array_y))
  array = np.vstack((array, array_z))
  array = array.transpose()
  return array

def transform_cam_marker(slam_coordinates_1, slam_coordinates_2):
  # Transform SLAM points according to calibration transformation
  """
  # Rack
  R_cam_marker = np.array([[0.99858239, -0.00736774, -0.05271548],
                           [0.00682177, 0.99992129, -0.01052936],
                           [0.05278891, 0.01015482, 0.99855406]])
  t_cam_marker = np.array([-0.00604897, 0.05317244, 0.04223419])
  """
  # UAV
  R_cam_marker = np.array([[-0.02680569, -0.38553653, 0.92230312],
                           [-0.99927546, -0.01460280, -0.03514699],
                           [0.02701865, -0.92257701, -0.38486576]])
  t_cam_marker = np.array([0.08020492, -0.05702338, -0.12186974])
  R_marker_cam = R_cam_marker.transpose()
  t_marker_cam = -t_cam_marker
  slam_coordinates_1[:,1:4] = np.transpose(np.dot(R_marker_cam, np.transpose(slam_coordinates_1[:,1:4]))) + t_marker_cam

  # Transform SLAM points according to calibration transformation
  slam_coordinates_2[:,1:4] = np.transpose(np.dot(R_marker_cam, np.transpose(slam_coordinates_2[:,1:4]))) + t_marker_cam

  return slam_coordinates_1[:,1:4], slam_coordinates_2[:,1:4]

def time_matching(list_gt_x, list_slam_x, gt_coordinates, slam_coordinates, accuracy):
  # Matching of the datasets according to their time stamps
  gt = np.zeros((len(list_gt_x), 4))
  slam = np.zeros((len(list_slam_x), 4))

  pos = 0
  offset = 1
  break_flag = False
  for i in range(len(list_gt_x)):
    for j in range(offset, len(list_slam_x)):
      # value where ekf time > vicon time
      if gt_coordinates[i, 0] < slam_coordinates[j, 0]:
        # check the one before
        if abs(gt_coordinates[i, 0] - slam_coordinates[j-1, 0]) < accuracy:
          gt[pos] = gt_coordinates[i]
          slam[pos] = slam_coordinates[j - 1]
          pos = pos + 1
          if pos >= len(slam_coordinates):
            break_flag = True
            break
          offset = j
          continue
    if break_flag==True:
      break

  # Remove empty rows
  slam = slam[~(slam == 0).all(1)]
  gt = gt[~(gt == 0).all(1)]

  return slam, gt

def plot(slam_1_orig_transformed, slam_1_transformed, slam_2_orig_transformed, slam_2_transformed, list_gt_x_1, list_gt_y_1, list_gt_z_1, list_gt_x_2, list_gt_y_2, list_gt_z_2, gt_1, gt_2):
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
  legend_all = ax2.scatter(list_gt_x_1[:], list_gt_y_1[:], list_gt_z_1[:], c='green', label='All leica points')
  for i in range(0, 1):
    ax2.text(list_gt_x_1[i], list_gt_y_1[i], list_gt_z_1[i], '%s' % (list_gt_timestamp_1[i]), size=20, zorder=1)
  legend_matched = ax2.scatter(gt_1[:, 1], gt_1[:, 2], gt_1[:, 3], c='red', s=100, label='Matched leica points')
  for i in range(len(slam_1_transformed[:,0])):
    if i%4 == 0:
      ax2.text(gt_1[i, 1], gt_1[i, 2], gt_1[i, 3], '%s' % (i), size=20, zorder=1)
  #legend_all = ax2.plot(list_gt_x_1[0:5*50], list_gt_y_1[0:5*50], list_gt_z_1[0:5*50], c='green', label='All leica points')
  #legend_matched = ax2.plot(gt_1[0:50,1], gt_1[0:50,2], gt_1[0:50,3], c='red', label='Matched leica points')
  # Make scaling equal
  max_range = np.array([gt_1[:, 1].max() - gt_1[:, 1].min(), gt_1[:, 2].max() - gt_1[:, 2].min(), gt_1[:, 3].max() - gt_1[:, 3].min()]).max() / 2.0
  mid_x = (gt_1[:, 1].max() + gt_1[:, 1].min()) * 0.5
  mid_y = (gt_1[:, 2].max() + gt_1[:, 2].min()) * 0.5
  mid_z = (gt_1[:, 3].max() + gt_1[:, 3].min()) * 0.5
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
  legend_all = ax2.scatter(list_gt_x_2, list_gt_y_2, list_gt_z_2, c='green', label='All leica points')
  legend_matched = ax2.scatter(gt_2[:, 1], gt_2[:, 2], gt_2[:, 3], c='red', s=100, label='Matched leica points')
  # Make scaling equal
  max_range = np.array([gt_2[:, 1].max() - gt_2[:, 1].min(), gt_2[:, 2].max() - gt_2[:, 2].min(), gt_2[:, 3].max() - gt_2[:, 3].min()]).max() / 2.0
  mid_x = (gt_2[:, 1].max() + gt_2[:, 1].min()) * 0.5
  mid_y = (gt_2[:, 2].max() + gt_2[:, 2].min()) * 0.5
  mid_z = (gt_2[:, 3].max() + gt_2[:, 3].min()) * 0.5
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
  legend_leica = ax3.scatter(gt_1[:, 1], gt_1[:, 2], gt_1[:, 3], c='red', label='Leica')
  """
  for i in range(len(gt_1[:,1])):
    if i%4 == 0:
      ax3.text(gt_1[i,1], gt_1[i,2], gt_1[i,3], '%s' % (i), size=20, zorder=1)
  """
  #legend_leica = ax3.plot(gt_1[0:100,1], gt_1[0:100,2], gt_1[0:100,3], c='red', label='Leica')

  # Make scaling equal
  max_range = np.array([gt_1[:, 1].max() - gt_1[:, 1].min(), gt_1[:, 2].max() - gt_1[:, 2].min(), gt_1[:, 3].max() - gt_1[:, 3].min()]).max() / 2.0
  mid_x = (gt_1[:, 1].max() + gt_1[:, 1].min()) * 0.5
  mid_y = (gt_1[:, 2].max() + gt_1[:, 2].min()) * 0.5
  mid_z = (gt_1[:, 3].max() + gt_1[:, 3].min()) * 0.5
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
  ax4.scatter(gt_2[:, 1], gt_2[:, 2], gt_2[:, 3], c='red')
  #ax4.plot(gt_2[0:100,1], gt_2[0:100,2], gt_2[0:100,3], c='red')
  # Make scaling equal
  max_range = np.array([gt_2[:, 1].max() - gt_2[:, 1].min(), gt_2[:, 2].max() - gt_2[:, 2].min(), gt_2[:, 3].max() - gt_2[:, 3].min()]).max() / 2.0
  mid_x = (gt_2[:, 1].max() + gt_2[:, 1].min()) * 0.5
  mid_y = (gt_2[:, 2].max() + gt_2[:, 2].min()) * 0.5
  mid_z = (gt_2[:, 3].max() + gt_2[:, 3].min()) * 0.5
  ax4.set_xlim(mid_x - max_range, mid_x + max_range)
  ax4.set_ylim(mid_y - max_range, mid_y + max_range)
  ax4.set_zlim(mid_z - max_range, mid_z + max_range)

  plt.show()

def calculate_rmse(slam_transformed, gt):
  # Calculate rmse
  var_slam = []

  for i in range(len(slam_transformed)):
    var_slam.append(((gt[i, 1] - slam_transformed[i, 0]) ** 2 + (gt[i, 2] - slam_transformed[i, 1]) ** 2 + (gt[i, 3] - slam_transformed[i, 2]) ** 2))

  return var_slam


if __name__ == '__main__':
  list_slam_timestamp_1 = []
  list_slam_x_1 = []
  list_slam_y_1 = []
  list_slam_z_1 = []

  list_gt_timestamp_1 = []
  list_gt_x_1 = []
  list_gt_y_1 = []
  list_gt_z_1 = []

  list_slam_timestamp_2 = []
  list_slam_x_2 = []
  list_slam_y_2 = []
  list_slam_z_2 = []

  list_gt_timestamp_2 = []
  list_gt_x_2 = []
  list_gt_y_2 = []
  list_gt_z_2 = []

  list_slam_timestamp_1, list_slam_x_1, list_slam_y_1, list_slam_z_1, list_slam_timestamp_2, list_slam_x_2, list_slam_y_2, list_slam_z_2 = open_slam()

  list_gt_timestamp_1, list_gt_x_1, list_gt_y_1, list_gt_z_1, list_gt_timestamp_2, list_gt_x_2, list_gt_y_2, list_gt_z_2 = open_ground_truth()

  offset = 42.4
  list_gt_timestamp_1 = time_offset(list_gt_timestamp_1, offset)
  list_gt_timestamp_2 = time_offset(list_gt_timestamp_2, offset)

  #array_x = np.array(list_slam_x_1)
  #array_y = np.array(list_slam_y_1)
  #array_z = np.array(list_slam_z_1)

  # Print nr. of data points
  print("map 1 nr of data points in slam_1: " + str(len(list_slam_x_1)))
  print("map 2 nr of data points in slam_2: " + str(len(list_slam_x_2)))

  print("map 1 nr of data points in gt_1: " + str(len(list_gt_x_1)))
  print("map 2 nr of data points in gt_2: " + str(len(list_gt_x_2)))

  gt_coordinates_1 = create_array(list_gt_timestamp_1, list_gt_x_1, list_gt_y_1, list_gt_z_1)

  slam_coordinates_1 = create_array(list_slam_timestamp_1, list_slam_x_1, list_slam_y_1, list_slam_z_1)

  # Add Gaussian noise to first trajectory
  #print("mean slam 1 map: " + str(np.mean(slam_coordinates_1[:,1:4], axis=0)) + ", std: " + str(np.std(slam_coordinates_1[:,1:4], axis=0)))
  #slam_coordinates_1 = slam_coordinates_1 + np.hstack((np.zeros(slam_coordinates_1[:,0:1].shape), np.random.normal(loc=0.0, scale=0.15, size=slam_coordinates_1[:, 1:4].shape)))
  #print("mean slam 1 map w. noise: " + str(np.mean(slam_coordinates_1[:,1:4], axis=0)) + ", std: " + str(np.std(slam_coordinates_1[:,1:4], axis=0)))

  gt_coordinates_2 = create_array(list_gt_timestamp_2, list_gt_x_2, list_gt_y_2, list_gt_z_2)

  slam_coordinates_2 = create_array(list_slam_timestamp_2, list_slam_x_2, list_slam_y_2, list_slam_z_2)

  # Transform SLAM points according to calibration transformation
  slam_coordinates_1[:,1:4], slam_coordinates_2[:,1:4] = transform_cam_marker(slam_coordinates_1, slam_coordinates_2)


  # Sort timings
  slam_coordinates_1 = slam_coordinates_1[slam_coordinates_1[:, 0].argsort()]
  #slam_coordinates_1 = slam_coordinates_1[np.argsort(slam_coordinates_1[:, 0])]
  #slam_coordinates_1 = slam_coordinates_1[np.lexsort(slam_coordinates_1[:, 0])]

  slam_coordinates_2 = slam_coordinates_2[slam_coordinates_2[:, 0].argsort()]

  accuracy = 4
  slam_1, gt_1 = time_matching(list_gt_x_1, list_slam_x_1, gt_coordinates_1, slam_coordinates_1, accuracy)
  slam_2, gt_2 = time_matching(list_gt_x_2, list_slam_x_2, gt_coordinates_2, slam_coordinates_2, accuracy)

  # Get transformation for alignement
  s_1, R_gt_es_1, gt_t_gt_es_1 = align_sim3(gt_1[0:50, 1:4], slam_1[0:50, 1:4])
  # Transform slam output
  slam_1_transformed = s_1 * np.transpose(np.dot(R_gt_es_1, np.transpose(slam_1[:,1:4]))) + gt_t_gt_es_1
  slam_1_orig_transformed = s_1 * np.transpose(np.dot(R_gt_es_1, np.transpose(slam_coordinates_1[:,1:4]))) + gt_t_gt_es_1

  # Transform slam output
  slam_2_transformed = s_1 * np.transpose(np.dot(R_gt_es_1, np.transpose(slam_2[:,1:4]))) + gt_t_gt_es_1
  slam_2_orig_transformed = s_1 * np.transpose(np.dot(R_gt_es_1, np.transpose(slam_coordinates_2[:,1:4]))) + gt_t_gt_es_1

  # Calculate rmse
  var_slam_1 = calculate_rmse(slam_1_transformed, gt_1)
  var_slam_2 = calculate_rmse(slam_2_transformed, gt_2)

  print("SLAM Map 1 rmse = {0}".format(np.sqrt(1 / (len(var_slam_1) - 1) * sum(var_slam_1))))
  print("SLAM Map 2 rmse = {0}".format(np.sqrt(1 / (len(var_slam_2) - 1) * sum(var_slam_2))))


  plot(slam_1_orig_transformed, slam_1_transformed, slam_2_orig_transformed, slam_2_transformed, list_gt_x_1, list_gt_y_1, list_gt_z_1, list_gt_x_2, list_gt_y_2, list_gt_z_2, gt_1, gt_2)
