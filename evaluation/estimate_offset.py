from evaluation import open_slam, open_ground_truth, time_offset, create_array, time_matching, transform_cam_marker, calculate_rmse
from align import align_sim3
import numpy as np
import math
import matplotlib.pyplot as plt
from joblib import Parallel, delayed
import multiprocessing as mp

# Global variables
min_pos = 0
min_value = 10
accuracy = 4

slam = 0
slam_transformed = 0
gt = 0

def iteration(gt_coordinates_1, gt_coordinates_2, slam_coordinates_1, slam_coordinates_2, list_gt_x_1, list_slam_x_1, list_gt_x_2, list_slam_x_2, i, ns, lock):
  """
  global min_value
  global min_pos
  global accuracy
  global slam
  global slam_transformed
  global gt
  global lock
  """

  gt_coordinates_1_local = np.copy(gt_coordinates_1)
  gt_coordinates_2_local = np.copy(gt_coordinates_2)

  offset = i * 0.1
  gt_coordinates_1_local[:,0] = time_offset(gt_coordinates_1_local[:,0], offset)
  gt_coordinates_2_local[:,0] = time_offset(gt_coordinates_2_local[:,0], offset)

  #for j in range(3):
  #  accuracy_j = 5 + j
  accuracy_j = 4
  slam_1, gt_1 = time_matching(list_gt_x_1, list_slam_x_1, gt_coordinates_1_local, slam_coordinates_1, accuracy_j)
  slam_2, gt_2 = time_matching(list_gt_x_2, list_slam_x_2, gt_coordinates_2_local, slam_coordinates_2, accuracy_j)

  # Get transformation for alignement
  s_1, R_gt_es_1, gt_t_gt_es_1 = align_sim3(gt_1[0:50, 1:4], slam_1[0:50, 1:4])
  # Transform slam output
  slam_1_transformed = s_1 * np.transpose(np.dot(R_gt_es_1, np.transpose(slam_1[:,1:4]))) + gt_t_gt_es_1
  slam_1_orig_transformed = s_1 * np.transpose(np.dot(R_gt_es_1, np.transpose(slam_coordinates_1[:,1:4]))) + gt_t_gt_es_1

  # Transform slam output
  #slam_2_transformed = s_1 * np.transpose(np.dot(R_gt_es_1, np.transpose(slam_2[:,1:4]))) + gt_t_gt_es_1
  #slam_2_orig_transformed = s_1 * np.transpose(np.dot(R_gt_es_1, np.transpose(slam_coordinates_2[:,1:4]))) + gt_t_gt_es_1

  # Calculate rmse
  var_slam_1 = calculate_rmse(slam_1_transformed, gt_1)
  #var_slam_2 = calculate_rmse(slam_2_transformed, gt_2)

  rmse = math.sqrt(1 / (len(var_slam_1) - 1) * sum(var_slam_1))
  #print("Accuracy = {0}, Offset {1}, SLAM Map rmse = {2}".format(accuracy_j, i, rmse))
  #print("SLAM Map 2 rmse = {0}".format(np.sqrt(1 / (len(var_slam_2) - 1) * sum(var_slam_2))))

  lock.acquire()
  if rmse < ns.min_value:
    print("Accuracy = {0}, Offset {1}, SLAM Map rmse = {2}".format(accuracy_j, i, rmse))
    ns.min_value = rmse
    ns.min_pos = i
    ns.accuracy = accuracy_j
    ns.slam = slam_1
    ns.slam_transformed = slam_1_transformed
    ns.gt = gt_1
    print("min_value: {0}".format(ns.min_value))
  lock.release()

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

  gt_coordinates_1 = create_array(list_gt_timestamp_1, list_gt_x_1, list_gt_y_1, list_gt_z_1)

  slam_coordinates_1 = create_array(list_slam_timestamp_1, list_slam_x_1, list_slam_y_1, list_slam_z_1)

  gt_coordinates_2 = create_array(list_gt_timestamp_2, list_gt_x_2, list_gt_y_2, list_gt_z_2)

  slam_coordinates_2 = create_array(list_slam_timestamp_2, list_slam_x_2, list_slam_y_2, list_slam_z_2)

  # Transform SLAM points according to calibration transformation
  slam_coordinates_1[:,1:4], slam_coordinates_2[:,1:4] = transform_cam_marker(slam_coordinates_1, slam_coordinates_2)


  # Sort timings
  slam_coordinates_1 = slam_coordinates_1[slam_coordinates_1[:, 0].argsort()]

  slam_coordinates_2 = slam_coordinates_2[slam_coordinates_2[:, 0].argsort()]


  offset = 30
  gt_coordinates_1[:,0] = time_offset(gt_coordinates_1[:,0], offset)
  gt_coordinates_2[:,0] = time_offset(gt_coordinates_2[:,0], offset)

  manager = mp.Manager()
  ns = manager.Namespace()

  lock = manager.Lock()

  ns.min_pos = 0
  ns.min_value = 10
  ns.ccuracy = 4

  ns.slam = 0
  ns.slam_transformed = 0
  ns.gt = 0

  pool = mp.Pool(processes=8)
  for i in range(200):
    pool.apply_async(iteration, (gt_coordinates_1, gt_coordinates_2, slam_coordinates_1, slam_coordinates_2, list_gt_x_1, list_slam_x_1, list_gt_x_2, list_slam_x_2, i, ns, lock))
  pool.close()
  pool.join()

  accuracy = ns.accuracy
  min_pos = ns.min_pos
  min_value = ns.min_value
  slam = ns.slam
  slam_transformed = ns.slam_transformed
  gt = ns.gt

  print("Accuracy: {0}, Min pos: {1}, Min value: {2}".format(accuracy, min_pos, min_value))


  fig = plt.figure()
  fig.suptitle('Leica and SLAM transformed Map 1')
  ax3 = fig.add_subplot(111, projection='3d')
  ax3.set_xlabel('X')
  ax3.set_ylabel('Y')
  ax3.set_zlabel('Z')
  legend_slam = ax3.scatter(slam_transformed[:,0], slam_transformed[:,1], slam_transformed[:,2], c='blue', label='SLAM transformed')
  """
  for i in range(len(slam_1_transformed[:,1])):
    if i%4 == 0:
      ax3.text(slam_1_transformed[i,0], slam_1_transformed[i,1], slam_1_transformed[i,2], '%s' % (i), size=20, zorder=1)
  """
  #legend_slam = ax3.plot(slam_1_transformed[:100,0], slam_1_transformed[0:100,1], slam_1_transformed[0:100,2], c='blue', label='SLAM transformed')
  #legend_slam = ax3.scatter(slam_1[:,0], slam_1[:,1], slam_1[:,2], c='blue', label='SLAM transformed')
  legend_leica = ax3.scatter(gt[:, 1], gt[:, 2], gt[:, 3], c='red', label='Leica')
  """
  for i in range(len(gt_1[:,1])):
    if i%4 == 0:
      ax3.text(gt_1[i,1], gt_1[i,2], gt_1[i,3], '%s' % (i), size=20, zorder=1)
  """
  #legend_leica = ax3.plot(gt_1[0:100,1], gt_1[0:100,2], gt_1[0:100,3], c='red', label='Leica')

  # Make scaling equal
  max_range = np.array([gt[:, 1].max() - gt[:, 1].min(), gt[:, 2].max() - gt[:, 2].min(), gt[:, 3].max() - gt[:, 3].min()]).max() / 2.0
  mid_x = (gt[:, 1].max() + gt[:, 1].min()) * 0.5
  mid_y = (gt[:, 2].max() + gt[:, 2].min()) * 0.5
  mid_z = (gt[:, 3].max() + gt[:, 3].min()) * 0.5
  ax3.set_xlim(mid_x - max_range, mid_x + max_range)
  ax3.set_ylim(mid_y - max_range, mid_y + max_range)
  ax3.set_zlim(mid_z - max_range, mid_z + max_range)

  plt.legend(handles=[legend_slam, legend_leica])

  plt.show()


