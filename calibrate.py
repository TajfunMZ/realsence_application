import copy, math
from functools import reduce
from cameraF import createBoundingBox, removeOutliers, savePCD, getPCD, getMarkerPoints
from mathF import preformVolumeCalculations
from basic import selectAndRotate, save2json

OUTLIER_NEIGBOURS = 40

# Capture the empty container and save selected point coordinates to calibrate for furure use
def captureReference(calibrationFileName, pipe, zero_volume, automaticAlignment, get_rgb, no_of_ref_measurments = 10, save_pcd = False):
    volume_measurements = []

    # Get image
    pcd = getPCD(pipe, get_rgb)
    
    if(save_pcd):
        savePCD(pcd, calibrationFileName)

    # Select area and rotate to align it straight, draw crop box and if it is fine, crop it
    if not get_rgb:
        pcd, cropArea, rotationMatrix = selectAndRotate(pcd, automaticAlignment, [])
    else:
        [point_class_vector, targets_pcd] = getMarkerPoints(pcd)
        pcd, cropArea, rotationMatrix = selectAndRotate(pcd, automaticAlignment, point_class_vector, targets_pcd)
    
    save_cropArea = copy.deepcopy(cropArea)
    
    lift_pcd = math.ceil(pcd.get_max_bound()[2] - 2*pcd.get_min_bound()[2])

    if zero_volume == -1:
        for i in range(no_of_ref_measurments):
            pcd = getPCD(pipe, False)
            cropArea = copy.deepcopy(save_cropArea)

            pcd.rotate(rotationMatrix, center=(0,0,0))
            cropBox = createBoundingBox(cropArea)
            pcd = pcd.crop(cropBox)

            pcd = removeOutliers(pcd, OUTLIER_NEIGBOURS)

            # move above the camera floor, from which we calculate the volume
            pcd.translate((0, 0, lift_pcd))    
            volume_measurements.append(preformVolumeCalculations(pcd, 0, i))
        save_volume = reduce(lambda a, b: a + b, volume_measurements) / len(volume_measurements)    # or use median np.median(volume_measurements) 
    else:
        save_volume = zero_volume

    ## Save rotated point coordinates, pcd rotation and box volume 
    save_dict = {
        'crop area': save_cropArea.tolist(),
        'rotation': rotationMatrix.tolist(),
        'volume': save_volume,
        'liftPcd': lift_pcd
    }
    save2json(save_dict, calibrationFileName)