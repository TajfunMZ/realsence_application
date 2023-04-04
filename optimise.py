import copy, math
import numpy as np
from functools import reduce
import open3d as o3d
import matplotlib.pyplot as plt
from cameraF import createBoundingBox, plotGeometriesWithOriginVectors, getPointCoords, createBox, getPCD
from mathF import moveByDistanceFromClosestTwoPoints, preformVolumeCalculations, average

NO_OF_TAKES = 10

# find seperated areas of same color
def getCenters(filterd_colors_ind, pcd, x, points_no = 10):
    points = getPointCoords(filterd_colors_ind, pcd)

    # Create pcd and cluster points
    pcd_filterd = o3d.geometry.PointCloud()
    pcd_filterd.points = o3d.utility.Vector3dVector(points)
    point_class_vector = pcd_filterd.cluster_dbscan(0.2 * x, points_no)

    clusters = []
    if len(point_class_vector) == 0:
        return []

    for i in range(max(point_class_vector)+1):
        clusters.append([])

    for ind, classified in enumerate(point_class_vector):
        if classified == -1:
            continue        

        clusters[classified].append(getPointCoords([ind], pcd_filterd))

    # Get centers of clusters
    centers = []
    for cluster in clusters:
        x=[]
        y=[]
        z=[]

        for point in cluster:
            x.append(point[0][0])
            y.append(point[0][1])
            z.append(point[0][2])

        centers.append([reduce(lambda a, b: a + b, x) / len(x), reduce(lambda a, b: a + b, y) / len(y), reduce(lambda a, b: a + b, z) / len(z)])
    
    return centers


# Change this at need. It is to optimise the hardcoded parameters used in code
def findOptimalParameters(pipe, iterr, real_distance1 = 1, real_distance2 = 1):
    run_distances1 = []
    run_distances2 = []
    result = []
    
    getPCD(pipe, True)

    for x in iterr:
        print(f'\nScale factor: {x}')
        sucessful_takes = 0

        ## get points and distances
        for i in range(10): # while sucessful_takes < NO_OF_TAKES: # 
            pcd = getPCD(pipe, True)
            # pcd.scale(x, center=(0, 0, 0))

            # Find 'red ish' colour
            filterd_colors_ind = []
            # print('RED')
            for inx, rgb in enumerate(np.asarray(pcd.colors)):
                if rgb[0] > 0.4 and rgb[1] < 0.4 and rgb[2] < 0.4:
                    filterd_colors_ind.append(inx)
                    # print(rgb)

            centers1 = getCenters(filterd_colors_ind, pcd, x, 30)
            if len(centers1) == 2:
                run_distances1.append(np.linalg.norm(np.asarray(centers1[0]) - np.asarray(centers1[1])))

            ## get points and distances
            # Find 'green ish' colour
            filterd_colors_ind = []
            # print('GREEN')
            for inx, rgb in enumerate(np.asarray(pcd.colors)):
                if rgb[0] < 0.5 and rgb[1] > 0.3 and rgb[2] < 0.4:
                    filterd_colors_ind.append(inx)
                    # print(rgb)

            centers2 = getCenters(filterd_colors_ind, pcd, x, 50)
            
            if len(centers2) == 2:
                run_distances2.append(np.linalg.norm(np.asarray(centers2[0]) - np.asarray(centers2[1])))

            # print(f'\nColor 1 points:\n{centers1}\nColor 2 points:\n{centers2}')
            if len(centers1) != 2 or len(centers2) != 2:
                continue
            
            sucessful_takes += 1

        if x == iterr[0]:
            box_array = []
            box_size = 0.01 * x
            for center in (centers1 + centers2):
                found_points_box = createBox(width = box_size, height = box_size, depth = box_size)
                found_points_box.translate(center)
                box_array.append(found_points_box)

            plotGeometriesWithOriginVectors([pcd] + box_array)
        
        avrg_d_1 = average(run_distances1)
        avrg_d_2 = average(run_distances2)

        distance_error1 = abs(avrg_d_1 - real_distance1)
        distance_error2 = abs(avrg_d_2 - real_distance2)
        print(f'Error 1: {distance_error1}')
        print(f'Error 2: {distance_error2}')

        result.append([x, avrg_d_1, distance_error1, avrg_d_2, distance_error2])
    return result


# floor height is decided automatically, position is still manual. I depends on the soroundings
def optimiseWithAutofloorSize(pcd_o, rotation_matrix, cropArea_o, iterr):
    measured_volumes = []
    volume_errors = []
    for x in iterr:
        pcd = copy.deepcopy(pcd_o)
        cropArea = copy.deepcopy(cropArea_o)
        pcd.scale(x, center=(0, 0, 0))
        pcd.rotate(rotation_matrix, center=(0,0,0))

        ## Get volume
        # get object crop
        pcd_b = copy.deepcopy(pcd)

        cropBox = createBoundingBox(cropArea)
        cropBox.scale(x, center=(0,0,0))
        cropArea = cropBox.get_box_points()
        pcd_b = pcd_b.crop(cropBox)
        
        # Get bounding box of the object and set it to get floor
        object_bound_box = pcd_b.get_axis_aligned_bounding_box()

        ## CALIBRATE
        pcd_t = copy.deepcopy(pcd)

        # get floor height
        floor_box = copy.deepcopy(cropBox)
        min_distance = min(moveByDistanceFromClosestTwoPoints(copy.deepcopy(cropArea[0:3]), 1)[1][0])
        floor_box.translate((-20*min_distance/23, 0, 0))
        pcd_t = pcd_t.crop(floor_box)
        floor_center = pcd_t.get_center()
        
        # move oriented crop box to the selected area
        pcd_c = copy.deepcopy(pcd)
        box_center = object_bound_box.get_center()
        object_bound_box.translate(floor_center - box_center)
        pcd_c = pcd_c.crop(object_bound_box)

        if(x == 0.7 or x == 0.72):
            object_bound_box.color = (1, 0, 0)
            plotGeometriesWithOriginVectors([pcd, object_bound_box])
        
        # set floor distance for this specific scale
        lift_pcd = math.ceil(pcd_c.get_max_bound()[2] - 2*pcd_c.get_min_bound()[2])
        pcd_c.translate((0, 0, lift_pcd))
        pcd_b.translate((0, 0, lift_pcd))

        zeroVolume = preformVolumeCalculations(pcd_c, 0, 10)
        measured_volume = preformVolumeCalculations(pcd_b, zeroVolume, 10)

        volume_error = abs(measured_volume - 0.33638)  # 0.33638 it the phisical volume of the box in m^3
        measured_volumes.append(measured_volume)
        volume_errors.append(volume_error)

    return [measured_volumes, volume_errors]