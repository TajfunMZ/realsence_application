import open3d as o3d
from cameraF import plotGeometriesWithOriginVectors, createBox, createBoundingBox, removeOutliers
from basic import selectAndRotate
from mathF import preformVolumeCalculations
import copy, math


def plotWithPlane(pcd_in):
    pcd = pcd_in[0] if hasattr(pcd_in, "__len__") else pcd_in

    floor_width = pcd.get_max_bound()[0] - pcd.get_min_bound()[0]
    floor_height = pcd.get_max_bound()[1] - pcd.get_min_bound()[1]
    floor_center_x = (pcd.get_max_bound()[0] + pcd.get_min_bound()[0])/2
    floor_center_y = (pcd.get_max_bound()[1] + pcd.get_min_bound()[1])/2
    floor_plane = createBox(width = floor_width*2, height = floor_height*2, depth = 0.01)
    floor_plane.translate((-floor_center_x/4, -floor_center_y/4, 0))
    floor_plane.paint_uniform_color([0.83, 0.83, 0.83])
    
    plot_pcd = pcd_in if hasattr(pcd_in, "__len__") else [pcd_in]
    plotGeometriesWithOriginVectors(plot_pcd)


file_empty = 'assetsget_pcd.ply'
file_full = 'assetsget_pcd_kup.ply'

pcd_empty = o3d.io.read_point_cloud("./assets/" + file_empty, print_progress = True)
pcd_full = o3d.io.read_point_cloud("./assets/" + file_full, print_progress = True)

# Show the pointclouds before any transformationa
# pcd_empty.paint_uniform_color([0.9,0.9,0.9])
# pcd_full.paint_uniform_color([0,0,0])
# plotWithPlane([pcd_full, pcd_empty])


# process the pcd-s
pcd_full, cropArea, rotationMatrix = selectAndRotate(pcd_full, True, [])
pcd_empty.rotate(rotationMatrix, center=(0,0,0))
save_cropArea = copy.deepcopy(cropArea)
cropBox = createBoundingBox(cropArea)

pcd_empty = pcd_empty.crop(cropBox)
pcd_empty = removeOutliers(pcd_empty, 30, 4.0)
pcd_full = pcd_full.crop(cropBox)
pcd_full = removeOutliers(pcd_full, 30, 4.0)

# This moves the entire PCD above the 0 on the 'z' axis.
lift_pcd = math.ceil(pcd_empty.get_max_bound()[2] - 2*pcd_empty.get_min_bound()[2])
pcd_empty.translate((0, 0, lift_pcd))
pcd_full.translate((0, 0, lift_pcd))

# Show the pointclouds after any transformationa
pcd_empty.paint_uniform_color([0.9,0.9,0.9])
pcd_full.paint_uniform_color([0,0,0])
plotWithPlane([pcd_full, pcd_empty])

volume_empty = preformVolumeCalculations(pcd_empty, 0, 10)
volume_full = preformVolumeCalculations(pcd_full, 0, 0)

print(f'First part finished successfully, with a calculated volume difference of {round(volume_full-volume_empty, 3)}m^3 and an actual one of 0.0976m^3!')


floor, inliers = pcd_empty.segment_plane(distance_threshold=0.02, ransac_n=3, num_iterations=1000)
inlier_cloud = pcd_empty.select_by_index(inliers)
inlier_cloud.paint_uniform_color([1.0, 1.0, 0])
plotGeometriesWithOriginVectors([inlier_cloud, pcd_empty])

floor, inliers = pcd_full.segment_plane(distance_threshold=0.02, ransac_n=3, num_iterations=1000)
inlier_cloud = pcd_full.select_by_index(inliers)
inlier_cloud.paint_uniform_color([1.0, 1.0, 0])
plotGeometriesWithOriginVectors([inlier_cloud, pcd_full])