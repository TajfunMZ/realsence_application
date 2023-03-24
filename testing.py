import pyrealsense2 as rs
import numpy as np
import open3d as o3d
from cameraF import plotGeometriesWithOriginVectors, getPointCoords, createBox
from basic import selectAndRotate
from functools import reduce
import matplotlib.pyplot as plt

SCALE = 1

if __name__ == '__main__':
    # init pipeline, pointcloud and configure camera settings
    pipe = rs.pipeline()
    cfg = rs.config()
    ctx = rs.context()
    align = rs.align(rs.stream.color)

    pipeline_wrapper = rs.pipeline_wrapper(pipe)
    pipeline_profile = cfg.resolve(pipeline_wrapper)
    device = pipeline_profile.get_device()
    dev = ctx.query_devices()
    advncd_mode_cfg = rs.rs400_advanced_mode(dev[0])
    # print(advncd_mode_cfg)

    # Declare wanted filters
    dec_filter = rs.decimation_filter()
    tresh_filter = rs.threshold_filter()
    disparity_filter = rs.disparity_transform()    
    temp_filter = rs.temporal_filter()

    found_rgb = False
    for s in device.sensors:
        if s.get_info(rs.camera_info.name) == 'RGB Camera':
            found_rgb = True
            break
    if not found_rgb:
        print("The program can use a color sensor and color markings for automatic detection. Will enable manual edge point selection.")

    cfg.enable_stream(rs.stream.depth, 424, 240, rs.format.z16, 30) # Enable depth stream
    if found_rgb:
        cfg.enable_stream(rs.stream.color, 424, 240, rs.format.rgb8, 30) # Enable color stream

    # Start capturing and apply some additional settings
    profile = pipe.start(cfg)
    if found_rgb:
        color_sensor = profile.get_device().first_color_sensor()
        color_sensor.set_option(rs.option.enable_auto_exposure, 1)
        color_sensor.set_option(rs.option.auto_exposure_priority, 1)
        color_sensor.set_option(rs.option.enable_auto_white_balance, 0)
        color_sensor.set_option(rs.option.brightness, -64)
        color_sensor.set_option(rs.option.contrast, 0)
        color_sensor.set_option(rs.option.gamma, 100)
        color_sensor.set_option(rs.option.hue, 0)
        color_sensor.set_option(rs.option.saturation, 100)
        color_sensor.set_option(rs.option.sharpness, 100)
        
        # Find options
        # print('')
        # for option in color_sensor.get_supported_options():
        #     print(f'Possible option: {option}. Possible values: {color_sensor.get_option_range(option)}. Currently at: {color_sensor.get_option(option)}')
    
    depth_sensor = profile.get_device().first_depth_sensor()
    depth_sensor.set_option(rs.option.enable_auto_exposure, 1); # Enable auto exposure
    depth_sensor.set_option(rs.option.depth_units, 0.001) # Set depth units from 1mm to 1mm. This decreases camera range but increases depth accuracy
    depth_table = advncd_mode_cfg.get_depth_table()
    depth_table.depthClampMax = 65535   # Set max range to max possible value at current depth units (16 bit)
    advncd_mode_cfg.set_depth_table(depth_table)

    # Align images
    for x in range(20):
        frame = pipe.wait_for_frames()
        frame = align.process(frame)
        
        # Filter frame
        dec_filter.process(frame)
        tresh_filter.process(frame)
        disparity_filter.process(frame)
        temp_filter.process(frame)

    # Grab camera data
    if found_rgb:
        color_frame = frame.get_color_frame()
        color_image = np.asanyarray(color_frame.get_data())
        color_image_o3d = o3d.geometry.Image(color_image)
        
    depth_frame = frame.get_depth_frame()
    depth_image = np.asanyarray(depth_frame.get_data())
    depth_image_o3d = o3d.geometry.Image(depth_image)

    intrinsics = frame.profile.as_video_stream_profile().intrinsics
    pinhole_camera_intrinsic = o3d.camera.PinholeCameraIntrinsic(intrinsics.width, intrinsics.height, intrinsics.fx, intrinsics.fy, intrinsics.ppx, intrinsics.ppy)
    
    # print(intrinsics)
    # print(pinhole_camera_intrinsic.intrinsic_matrix)

    # Convert to o3d
    if found_rgb:
        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(color_image_o3d, depth_image_o3d, depth_scale=1000.0, depth_trunc=1000.0, convert_rgb_to_intensity=False)
        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, pinhole_camera_intrinsic)
    else:
        #option1
        pcd = o3d.geometry.PointCloud.create_from_depth_image(depth_image_o3d, pinhole_camera_intrinsic)

    #Flip it, otherwise the pointcloud will be upside down
    pcd.transform([[1, 0, 0, 0 ], 
                   [0, -1, 0, 0],
                   [0, 0, -1, 0], 
                   [0, 0, 0, 1 ]])
    # pcd.scale(SCALE, center = (0,0,0))

    # [pcd_cropped, cropArea, rotationMatrix] = selectAndRotate(pcd, True)
    # mean_color = reduce(lambda a, b: a + b, np.asarray(pcd_cropped.colors) / len(np.asarray(pcd_cropped.colors)))
    # print(mean_color)

    # Find colour
    r = []
    g = []
    b = []

    if found_rgb:
        filterd_colors_ind = []
        for inx, rgb in enumerate(np.asarray(pcd.colors)):
            if rgb[0] > 0.5 and rgb[1] < 0.4 and rgb[2] < 0.4:
                filterd_colors_ind.append(inx)
                # print(rgb)

        # print(filterd_colors_ind)
        points = getPointCoords(filterd_colors_ind, pcd)

        # Create pcd and cluster points
        pcd_filterd = o3d.geometry.PointCloud()
        pcd_filterd.points = o3d.utility.Vector3dVector(points)
        point_class_vector = pcd_filterd.cluster_dbscan(0.1 * SCALE, 4, False)

        clusters = []
        box_array = []
        if len(point_class_vector) != 0:
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
            
            if len(centers) == 2:
                distance = np.linalg.norm(np.asarray(centers[0]) - np.asarray(centers[1]))
                print(f'\nDistance: {distance} m')
            else:
                print(f'\nWanted 2 centers but got {len(centers)}')

            for center in centers:
                # print(center)
                # TODO: najdi točko najbližjo centru v PCD-ju glej kako se razdalja med centroma spreminja preko indexa teh dveh točk po raznih operacijah
                # TODO: če to ne dela piši na njihov blog da ti povejo zakaj se razdalja ne ohrani v našem primeru
                found_points_box = createBox(width = 0.01 * SCALE, height = 0.01 * SCALE, depth = 0.01 * SCALE)
                found_points_box.translate(center)
                box_array.append(found_points_box)

        pcd_filterd.paint_uniform_color([1,1,1])
        plotGeometriesWithOriginVectors([pcd, pcd_filterd] + box_array)

    # Stop streaming
    pipe.stop()