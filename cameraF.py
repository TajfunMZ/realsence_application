## Camera and image related functionsDOWN
import open3d as o3d
import numpy as np
import pyrealsense2 as rs
import copy

# Initialize D435 depth camera, capture only depth data, not RGB
def initCamera(capture_rgb = True):
    pipe = rs.pipeline()
    cfg = rs.config()
    ctx = rs.context()

    pipeline_wrapper = rs.pipeline_wrapper(pipe)
    pipeline_profile = cfg.resolve(pipeline_wrapper)
    device = pipeline_profile.get_device()
    dev = ctx.query_devices()
    advncd_mode_cfg = rs.rs400_advanced_mode(dev[0])

    found_rgb = False
    for s in device.sensors:
        if s.get_info(rs.camera_info.name) == 'RGB Camera':
            found_rgb = True
            break
    if not found_rgb or not capture_rgb:
        print("Manual selection confirmed, the program can use a color sensor and color markings for automatic detection.")
    mode = found_rgb and capture_rgb
    
    cfg.enable_stream(rs.stream.depth, 424, 240, rs.format.z16, 30) # Enable depth stream
    if mode:
        cfg.enable_stream(rs.stream.color, 424, 240, rs.format.rgb8, 30) # Enable color stream  

    # Start capturing and apply some additional settings
    profile = pipe.start(cfg)
    if mode:
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

    depth_sensor = profile.get_device().first_depth_sensor()
    depth_sensor.set_option(rs.option.enable_auto_exposure, 1); # Enable auto exposure
    depth_sensor.set_option(rs.option.depth_units, 0.001) # Set depth units from 1mm to 1mm. This decreases camera range but increases depth accuracy
    depth_table = advncd_mode_cfg.get_depth_table()
    depth_table.depthClampMax = 65535   # Set max range to max possible value at current depth units (16 bit)
    advncd_mode_cfg.set_depth_table(depth_table)

    # Calibrate autoexposure
    for x in range(20):
        pipe.wait_for_frames()

    return pipe


# Capture a pointcloud using a connected D435 camera (camera has to be initialized beforehand)
def capture_pcd(pipe, mode):
    if mode:
        align = rs.align(rs.stream.color)
    
    # declare filters
    dec_filter = rs.decimation_filter()
    tresh_filter = rs.threshold_filter()
    disparity_filter = rs.disparity_transform()    
    temp_filter = rs.temporal_filter()
    
    # Align images
    for i in range(20):
        frame = pipe.wait_for_frames()

        # Filter frame
        dec_filter.process(frame)
        tresh_filter.process(frame)
        disparity_filter.process(frame)
        temp_filter.process(frame)
        
        if mode:
            frame = align.process(frame)

    if mode:
        color_frame = frame.get_color_frame()
        color_image = np.asanyarray(color_frame.get_data())
        color_image_o3d = o3d.geometry.Image(color_image)

    depth_frame = frame.get_depth_frame()
    depth_image = np.asanyarray(depth_frame.get_data())
    depth_image_o3d = o3d.geometry.Image(depth_image)

    # From the depth image create a Point cloud, taking into account the camera intrinsics
    intrinsics = frame.profile.as_video_stream_profile().intrinsics
    pinhole_camera_intrinsic = o3d.camera.PinholeCameraIntrinsic(intrinsics.width, intrinsics.height, intrinsics.fx, intrinsics.fy, intrinsics.ppx, intrinsics.ppy)
    
    # Convert to o3d
    if mode:
        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(color_image_o3d, depth_image_o3d, depth_scale=1000.0, depth_trunc=100.0, convert_rgb_to_intensity=False)
        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, pinhole_camera_intrinsic)  
    else:
        pcd = o3d.geometry.PointCloud.create_from_depth_image(o3d.geometry.Image(depth_image), pinhole_camera_intrinsic)
    
    # because the photo is taken from above, we flip the image
    pcd.transform([[1, 0, 0, 0],
                    [0, -1, 0, 0],
                    [0, 0, -1, 0],
                    [0, 0, 0, 1]])

    return pcd


# get 3d image
def getPCD(pipe, mode = False):     # The camera initially has offset and the transformation to meters is not 1:1. Previous scale = 0.225
    pcd = capture_pcd(pipe, mode)   # second parameter is downgrade koeficient in meters
    return pcd


# filter by color   
# TODO: later this can be upgraded to detect actual marker tracking like how it is done here: https://vision.soe.ucsc.edu/ColorMarkerSoftware
def getMarkerPoints(pcd):
    # Find colour
    filterd_colors_ind = []
    for inx, rgb in enumerate(np.asarray(pcd.colors)):
        if rgb[0] < 0.5 and rgb[1] > 0.3 and rgb[2] < 0.4:  # green
            filterd_colors_ind.append(inx)

    points = getPointCoords(filterd_colors_ind, pcd)

    # Create pcd and cluster points
    pcd_filterd = o3d.geometry.PointCloud()
    pcd_filterd.points = o3d.utility.Vector3dVector(points)
    point_class_vector = pcd_filterd.cluster_dbscan(0.2, 50, False)

    return [point_class_vector, pcd_filterd]


# plot open3d object with axis arrows
def plotGeometriesWithOriginVectors(geometries, mesh_show_wireframe=False):
    viewer = o3d.visualization.Visualizer()
    viewer.create_window()
    for geometry in geometries:
        viewer.add_geometry(geometry)
    opt = viewer.get_render_option()
    opt.show_coordinate_frame = True
    opt.mesh_show_wireframe = mesh_show_wireframe
    opt.background_color = np.asarray([0.5, 0.5, 0.5])
    viewer.run()
    viewer.destroy_window()


# Select area to crop
def selectAreaWithPoints(pcd):
    print("\n1) Please pick at least four correspondences using [shift + left click]")
    print("   Press [shift + right click] to undo point picking")
    print("2) Afther picking points, press q for close the window\n")
    
    # User picks points
    picked_points = []
    
    # Load the image ona plot and enable editing
    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window()
    vis.clear_geometries()
    vis.add_geometry(pcd)

    opt = vis.get_render_option()
    opt.show_coordinate_frame = True
    opt.background_color = np.asarray([0.5, 0.5, 0.5])

    vis.run()
    vis.destroy_window()
    picked_points = vis.get_picked_points()

    if(len(picked_points) != 4):
        if(input('You have not selected 4 points, do you want to quit? Anwser with "y" for yes or "n" for no: ') == 'y'):
            exit('Please try your best to select exactly 4 points next time. I belive in you!!')
        else:
            exit('Please restart the program to select the correct number of points.')
    else:        
        return picked_points


# convert point sequence number to coordinates
def getPointCoords(picked_points, pcd):
    point_coords = []
    for i in range(len(picked_points)):
        point_pcd = pcd.select_by_index([picked_points[i]])
        point_coord = np.asarray(point_pcd.points)[0]
        point_coords.append(point_coord)
    
    return point_coords


# define triangulation mesh structure
def defineTriangulation(xyz, tri):
    surface = o3d.geometry.TriangleMesh()
    surface.vertices = o3d.utility.Vector3dVector(xyz)
    surface.triangles = o3d.utility.Vector3iVector(tri.simplices)
    surface.paint_uniform_color([1,0.7,0])

    return surface


# Create open3d object from points
def createBoundingBox(cropArea):
    cropBox = o3d.utility.Vector3dVector(cropArea)
    return o3d.geometry.OrientedBoundingBox.create_from_points(cropBox, True)


# Remove noise from cropped PCD
def removeOutliers(pcd, outlier_neigbours):
    cl, ind = pcd.remove_statistical_outlier(nb_neighbors=outlier_neigbours, std_ratio=2.0)
    return pcd.select_by_index(ind)


# Save PCD
def savePCD(pcd, name):
    o3d.io.write_point_cloud("/realsence_application/assets/" + name + '.ply', pcd, write_ascii=False, compressed=False, print_progress=False)


# define open3d box
def createBox(width, height, depth):
    return o3d.geometry.TriangleMesh.create_box(width, height, depth)


# open point cloud from file
def loadPCD(file):
    return o3d.io.read_point_cloud("/realsence_application/assets/" + file, print_progress = True)


# Draww with real colours
def draw_registration_result_original_color(source, target, transformation):
    source_temp = copy.deepcopy(source)
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target],
                                      zoom=0.5,
                                      front=[-0.2458, -0.8088, 0.5342],
                                      lookat=[1.7745, 2.2305, 0.9787],
                                      up=[0.3109, -0.5878, -0.7468])