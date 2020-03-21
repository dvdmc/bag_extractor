import sys
import os
import numpy as np

def load_bin_file(filename):
  """Load 3D point cloud from KITTI file format
     (adapted from https://github.com/hunse/kitti)
  """
  points = np.fromfile(filename, dtype=np.float32).reshape(-1,4)
  #points[:, 3] = 1.0
  print(points)
  return points


if __name__ == "__main__":

  filename = sys.argv[1]

  print("File loaded from: %s",filename)

  load_bin_file(filename)
