import json, copy
import numpy as np
from functools import reduce
from mathF import preformVolumeCalculations, getRotationMatrix, rectangleFromPoints, createPlaneParametersFromPoints
from cameraF import getPCD, getPointCoords, selectAreaWithPoints, createBoundingBox, plotGeometriesWithOriginVectors, createBox

CROP_HEIGHT = 4             # If the cropbox cuts of the pile in the z coordinate, increase this
CROP_EDGE = 1               # It is the added distance between points devided by crop_edge_factor

# get rotated pcd and new rotated points
def rotateImage(pcd, pointIndex, automaticAlignment):
    if(automaticAlignment):
        floor, inliers = pcd.segment_plane(distance_threshold=0.02, ransac_n=3, num_iterations=1000)
        # inlier_cloud = pcd.select_by_index(inliers)
        # inlier_cloud.paint_uniform_color([1.0, 1.0, 0])
        # o3d.visualization.draw_geometries([inlier_cloud])
    else:
        points = getPointCoords(pointIndex, pcd)
        floor = createPlaneParametersFromPoints(points)
    normal = tuple(floor[:3])/np.linalg.norm(tuple(floor[:3]))

    rotation_matrix = getRotationMatrix(normal, (0,0,1))
    pcd.rotate(rotation_matrix, center=(0,0,0))

    return [pcd, rotation_matrix]


# Save to json file
def save2json(save_dict, name):
    json_object = json.dumps(save_dict, indent=4)
    
    with open("./configuration_files/" + name + ".json", "w") as outfile:
        outfile.write(json_object)


# Load calibration data
def loadCalibration(calibrationFileName):
    # Opening and reading JSON file
    with open('./configuration_files/' + calibrationFileName + '.json', 'r') as openfile:
        json_object = json.load(openfile)

    return json_object


# select points and rotate the pointcloud
def selectAndRotate(pcd_o, automaticAlignment, point_class_vector = [], targets_pcd = 0):
    auto_select = False if targets_pcd == 0 else True
    # Select points to define crop area and floor plane angle
    if not auto_select:
        pointIndex = selectAreaWithPoints(pcd_o)
    else:
        automaticAlignment = True
        pointIndex = []
    
    while(True):
        pcd = copy.deepcopy(pcd_o)
        
        # Calculate the alignment and rotate the pcd and redefine the point coordinates
        pcd, rotationMatrix = rotateImage(pcd, pointIndex, automaticAlignment)

        if auto_select:
            targets_pcd.rotate(rotationMatrix, center = (0,0,0))
            clusters = []
            for i in range(max(point_class_vector)+1):
                clusters.append([])

            for ind, classified in enumerate(point_class_vector):
                if classified == -1:
                    continue

                clusters[classified].append(getPointCoords([ind], targets_pcd))
            
            points = []
            for cluster in clusters:
                x=[]
                y=[]
                z=[]

                for point in cluster:
                    x.append(point[0][0])
                    y.append(point[0][1])
                    z.append(point[0][2])

                points.append(np.asarray([reduce(lambda a, b: a + b, x) / len(x), reduce(lambda a, b: a + b, y) / len(y), reduce(lambda a, b: a + b, z) / len(z)]))
            if len(points) != 4:
                auto_select = False
                print('Could not detect 4 different markers, please choose points manualy.')
                continue

            # box_array = []
            # for center in points:
            #     found_points_box = createBox(width = 0.01, height = 0.01, depth = 0.01)
            #     found_points_box.translate(center)
            #     box_array.append(found_points_box)

            # targets_pcd.paint_uniform_color([1,1,1])
            # plotGeometriesWithOriginVectors([pcd, targets_pcd] + box_array)
        else:
            # get new coordinates of points
            points = getPointCoords(pointIndex, pcd)

        cropArea = rectangleFromPoints(points, CROP_HEIGHT, CROP_EDGE)

        # Create a crop box and show it
        cropBox = createBoundingBox(cropArea)

        cropBox.color = (1, 0, 0)
        plotGeometriesWithOriginVectors([pcd, cropBox])

        # If the box inclues the right area crop the image
        selection = input('\nIs the cropping rectangle selected correctly? \nTo confirm enter "y", to diss/enable automatic plain selection enter "d" or "e" respectively, to try again enter "t", to exit enter "q": ')

        if(selection == 'y'):
            return [pcd.crop(cropBox), cropArea, rotationMatrix]
        elif(selection == 'd'):
            automaticAlignment = False
            continue
        elif(selection == 'e'):
            automaticAlignment = True
            continue
        elif(selection == 't'):
            pointIndex = selectAreaWithPoints(pcd_o)
            auto_select = False
            continue
        else:
            exit("\nThank you for flying air Tajfun, we hope you've had a pleasant journey and we wish to see you again soon. :)\n")

# use calibrated data and camera feed to measure volume
def startMeasurment(calibration_json, pipe, measurment):
    # Extract saved data
    rotationMatrix = np.array(calibration_json['rotation'])
    cropArea = np.array(calibration_json['crop area'])
    zeroVolume = calibration_json['volume']
    lift_pcd = calibration_json['liftPcd']

    # Take a snapshot (pcd)
    pcd = getPCD(pipe)
    pcd.rotate(rotationMatrix, center=(0,0,0))

    # crop image to only include the container
    cropBox = createBoundingBox(cropArea)
    if(measurment <= 0):
        cropBox.color = (1, 0, 0)
        plotGeometriesWithOriginVectors([pcd, cropBox])
        if(input('If the area is selected correctly, enter "y", otherwise enter "q": ') != 'y'):
            exit('If the area was not selected correctly, try running the calibration again.')

    pcd = pcd.crop(cropBox)

    # Remove noise from cropped PCD
    cl, ind = pcd.remove_statistical_outlier(nb_neighbors=100,std_ratio=2.0)
    pcd = pcd.select_by_index(ind)

    pcd.translate((0, 0, lift_pcd))
    return [preformVolumeCalculations(pcd, zeroVolume, measurment), zeroVolume]