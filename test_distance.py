import open3d as o3d
import numpy as np

points = [[0,0,0],
          [5,0,0],
          [3,4,0],
          [0,3,4],
          [4,0,3]]

pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(np.asanyarray(points))

### FIND WEIRD CHANGE WITHIN THEESE COMMENTS ###

# pcd.scale(0.2, center = (0,0,0))


################################################

PCD_points = np.asanyarray(pcd.points)
point_c = PCD_points[0]

distances = []
for point in PCD_points:
    distances.append(np.linalg.norm(point_c - point))

print(f'\nPoints: \n{PCD_points}')
print(f'\nDistances: {distances}')