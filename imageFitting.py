import open3d as o3d
from cameraF import plotGeometriesWithOriginVectors, selectAreaWithPoints, getPointCoords, createBoundingBox, removeOutliers
from mathF import rectangleFromPoints
import os, copy, time
import numpy as np
from scipy.spatial.transform import Rotation as R


OUTLIER_NEIGBOURS = 10
STD_RATIO = 5


def GetCrop(pcd):
    r = 'n'
    while(r != 'y'):
        pointIndex = selectAreaWithPoints(pcd)
        points = getPointCoords(pointIndex, pcd)
        cropArea = rectangleFromPoints(points, 5, 1)
        
        # Create a crop box and show it
        cropBox = createBoundingBox(cropArea)
        plotGeometriesWithOriginVectors([pcd, cropBox])
        
        r = input('\nWas it good? [y/n]: ')
    
    pcd_c = pcd.crop(cropBox)
    pcd_c = removeOutliers(pcd_c, OUTLIER_NEIGBOURS, STD_RATIO)

    return pcd_c


def cropTransformedPCD(target, source, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    target_temp.transform(transformation)
    bbpx = source_temp.get_axis_aligned_bounding_box()
    target_croped = target_temp.crop(bbpx)

    return [source_temp, target_croped]


def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    # target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(np.linalg.inv(transformation))
    o3d.visualization.draw_geometries([source_temp, target_temp],
                                      zoom=0.4459,
                                      front=[0.9288, -0.2951, -0.2242],
                                      lookat=[1.6784, 2.0612, 1.4451],
                                      up=[-0.3402, -0.9189, -0.1996])


def preprocess_point_cloud(pcd, voxel_size):
    print(":: Downsample with a voxel size %.3f." % voxel_size)
    pcd_down = pcd.voxel_down_sample(voxel_size)

    radius_normal = voxel_size * 2
    print(":: Estimate normal with search radius %.3f." % radius_normal)
    pcd_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

    radius_feature = voxel_size * 5
    print(":: Compute FPFH feature with search radius %.3f." % radius_feature)
    pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        pcd_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
    return pcd_down, pcd_fpfh


def execute_global_registration(source_down, target_down, source_fpfh,
                                target_fpfh, voxel_size):
    distance_threshold = voxel_size * 1.5
    print(":: RANSAC registration on downsampled point clouds.")
    print("   Since the downsampling voxel size is %.3f," % voxel_size)
    print("   we use a liberal distance threshold %.3f." % distance_threshold)
    result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh, True,
        distance_threshold,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
        3, [
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(
                0.9),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(
                distance_threshold)
        ], o3d.pipelines.registration.RANSACConvergenceCriteria(100000, 0.999))

    distance_threshold = voxel_size * 0.4
    print(":: Point-to-plane ICP registration is applied on original point")
    print("   clouds to refine the alignment. This time we use a strict")
    print("   distance threshold %.3f." % distance_threshold)
    result = o3d.pipelines.registration.registration_icp(
        source_down, target_down, distance_threshold, result.transformation,
        o3d.pipelines.registration.TransformationEstimationPointToPlane())
    return result


def matchAndJoin(get_aligned, voxel_size, end_voxel_size):
    while len(get_aligned) > 1:
        pcd_c = GetCrop(get_aligned[0])
        pcd_down1, pcd_fpfh1 = preprocess_point_cloud(pcd_c, voxel_size)

        if(input('Discard pcd? [y/n]: ') == 'y'):
            get_aligned.pop(0)
            continue

        for pcd_a in get_aligned:
            # feature matching
            pcd1_d = copy.deepcopy(pcd_down1)
            p_fpfh1 = copy.deepcopy(pcd_fpfh1)
            # plotGeometriesWithOriginVectors([pcds[i], pcds[len(get_aligned) - i - 1]])
            
            pcd_down2, pcd_fpfh2 = preprocess_point_cloud(pcd_a, voxel_size)
            result_ransac = execute_global_registration(pcd_down2, pcd1_d,
                                                        pcd_fpfh2, p_fpfh1,
                                                        voxel_size)
            
            print(result_ransac)
            draw_registration_result(pcd_down1, pcd_down2, result_ransac.transformation)
            [source, target] = cropTransformedPCD(pcd_down2, pcd_down1, result_ransac.transformation)
            
            get_aligned.pop(0)
            if(input('Matching succesful? [y/n]: ') == 'y'):
                source += target
                get_aligned.append(source.voxel_down_sample(voxel_size = end_voxel_size))

    return get_aligned


def CreateTransMatrix(positions, a, b):
    trans_matrixes = []
    cam2tool = np.array([0.2, 0.1, -1.0])
    # cam2tool[b] += a
    print(cam2tool)

    for position in positions:
        
        position[b] += a
        
        x = position[0]
        y = position[1]
        z = position[2]

        xyz = [x, y, z]

        rot_x = -position[3]
        rot_y = -position[4]
        rot_z = -position[5]
        
        rot = np.array([rot_x, rot_y, rot_z])
        r = R.from_rotvec([rot], degrees=True)

        T = np.c_[r.as_matrix()[0], xyz]
        T = np.r_[T, np.matrix([0, 0, 0, 1])]

        c2t_m = np.c_[np.identity(3), cam2tool]
        c2t_m = np.r_[c2t_m, np.matrix([0, 0, 0, 1])]
        T2 = np.dot(T, c2t_m)

        trans_matrixes.append(T2)

    return trans_matrixes


def FormatMatrix(matrixes):
    # return [np.linalg.inv(matrix) for matrix in matrixes]

    # decompose to R and T
    temp_matrixes = []
    for matrix in matrixes:
        xx = matrix[0][0]
        xy = matrix[0][1]
        xz = matrix[0][2]

        yx = matrix[1][0]
        yy = matrix[1][1]
        yz = matrix[1][2]

        zx = matrix[2][0]
        zy = matrix[2][1]
        zz = matrix[2][2]

        x = matrix[0][3]
        y = matrix[1][3]
        z = matrix[2][3]

        temp_matrixes.append([[xx, xy, xz, x], [yx, yy, yz, y], [zx, zy, zz, z], [0, 0, 0, 1]])
    
    return temp_matrixes
    # return [np.linalg.inv(matrix) for matrix in temp_matrixes]


def LoadPCD(file):
    return o3d.io.read_point_cloud("./assets/transformed_pcd/" + file, print_progress = False)


def SavePCD(name, pcd):
    o3d.io.write_point_cloud("./assets/transformed_pcd2/" + name, pcd, write_ascii=False, compressed=False, print_progress=False)


def TransformPCD(pcd, transform_matrix):
    return pcd.transform(transform_matrix)


if __name__ == '__main__':
    files = os.listdir('./assets/transformed_pcd')
    pcds = []
    pcds_o = []
    positions = []
    matrixes = []

    with open('./assets/positions.txt', 'r') as file:
        my_text = file.read()
        for i, line in enumerate(my_text.splitlines()):
            if 'Position' in line:
                if len(positions) != 0:
                    matrixes.append(matrix)

                positions.append([float(x) for x in line.split(':')[-1][2:-3].split(',')])
            
            elif 'Matrix' in line:
                matrix = [x for x in line.split(':')[-1][4:-3].split(' ') if x != '']
                matrix = [[float(x.split('e')[0]) for x in matrix]]

            else:
                temp_matrix = [x for x in line.split(':')[-1][2:-3].split(' ') if x != '']
                matrix.append([float(x.split('e')[0]) for x in temp_matrix])

                if i+1 == len(my_text.splitlines()):
                    matrixes.append(matrix)


    # matrixes.pop(3)
    # matrixes.pop(3)
    # transformed_matrixes = FormatMatrix(matrixes)
    
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    ctr = vis.get_view_control()
    
    for b in range(0,3):
        for a in [round(-10 + x * 1, 2) for x in range(0, 20)]: # from -1 to 1 by 0.1
            pcds = []
            transformed_matrixes = CreateTransMatrix(positions, a, b)

            for i, file in enumerate(files):

                pcds_o.append(LoadPCD(file))
                pcd_c = copy.deepcopy(pcds_o[-1])
                pcds.append(TransformPCD(pcd_c, transformed_matrixes[i]))
                
                SavePCD(file, pcds[-1])
            
            vis.clear_geometries()
            for pcd in pcds:
                vis.add_geometry(pcd)

            ctr.rotate(600.0, 0.0)
            vis.poll_events()
            vis.update_renderer()
    
    pcds = []
    transformed_matrixes = CreateTransMatrix(positions, 0, 0)

    for i, file in enumerate(files):

        pcds_o.append(LoadPCD(file))
        pcd_c = copy.deepcopy(pcds_o[-1])
        pcds.append(TransformPCD(pcd_c, transformed_matrixes[i]))

    # pcds_o[0].paint_uniform_color([1,1,1])
    # pcds[0].paint_uniform_color([0.25,0.25,0.25])
    # print('Showing original and transformed pairs:')
    # for i in range(len(pcds)-1):
    #     print(f'Pair number: {i} & {i}')
        # plotGeometriesWithOriginVectors([pcds[i]])
        # plotGeometriesWithOriginVectors([pcds_o[i]])
        # plotGeometriesWithOriginVectors([pcds[i], pcds[i+1]])
    
    # print('Showing all clouds of the same run:')
    plotGeometriesWithOriginVectors(pcds)

    # plotGeometriesWithOriginVectors(pcds_o)

    # end_voxel_size = 0.01
    # voxel_size = 0.025
    # threshold = 0.02
    # trans_init = np.asarray([[1,0,0,  -0.85],
    #                         [0,1,0,   0.35],
    #                         [0,0,1,   -0.6],
    #                         [0,0,0,   1]])
    
    # get_aligned = copy.deepcopy(pcds)

    # while(len(get_aligned) > 2):
    #     get_aligned = matchAndJoin(get_aligned, voxel_size, end_voxel_size)


    # plotGeometriesWithOriginVectors(get_aligned)
    # SavePCD('joinedPCD', get_aligned)

    

        # ICP registration
        # reg_p2p = o3d.pipelines.registration.registration_icp(
        #     source, target, threshold, trans_init,
        #     o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        #     o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=2000))
        
        # previous_pcd = to_align

        # print(reg_p2p)
        # print("Transformation is:")
        # print(reg_p2p.transformation)

        # draw_registration_result(to_align, previous_pcd, reg_p2p.transformation)
