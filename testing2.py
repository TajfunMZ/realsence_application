import pyrealsense2 as rs
import numpy as np


def get_aligned_images():
    frames = pipeline.wait_for_frames()     
  
    #Apply filters
    pc_filtered = decimation.process(frames)
    pc_filtered = depth_to_disparity.process(pc_filtered)
    pc_filtered = spatial.process(pc_filtered)
    pc_filtered = temporal.process(pc_filtered)
    pc_filtered = disparity_to_depth.process(pc_filtered).as_frameset()
    
    #Align the depth frame to color frame
    aligned_frames = align.process(pc_filtered)      
    aligned_depth_frame = aligned_frames.get_depth_frame()     
    aligned_color_frame = aligned_frames.get_color_frame()     

    img_color = np.asanyarray(aligned_color_frame.get_data())       
    img_depth = np.asanyarray(aligned_depth_frame.get_data())     

    aligned_depth_color_frame = colorizer.colorize(aligned_depth_frame)
    img_depth_mapped = np.asanyarray(aligned_depth_color_frame.get_data())

    return img_color, img_depth, img_depth_mapped, aligned_color_frame, aligned_depth_frame, aligned_frames


def get_3d_camera_coordinate(depth_pixel, aligned_color_frame, aligned_depth_frame, aligned_frames):
    x = np.round(depth_pixel[1]).astype(np.int64)
    y = np.round(depth_pixel[0]).astype(np.int64)

    #pointcloud
    pc.map_to(aligned_color_frame)
    points = pc.calculate(aligned_depth_frame)
    points.export_to_ply("../frame_test.ply", aligned_color_frame)  

    vtx = np.asanyarray(points.get_vertices())
    #print('vtx_before_reshape: ', vtx.shape)  
    vtx = np.reshape(vtx, (1080, 1920, -1))
    #print('vtx_after_reshape: ', vtx.shape) 

    camera_coordinate = vtx[y][x][0]
    #print ('camera_coordinate: ',camera_coordinate)
    dis = camera_coordinate[2]

    return dis, camera_coordinate


''' camera setting '''
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 1280,720, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 1920,1080, rs.format.bgr8, 30)

profile = pipeline.start(config)

pc = rs.pointcloud()
points = rs.points()

#Define filters
#Decimation:
decimation = rs.decimation_filter()
#Depth to disparity
depth_to_disparity = rs.disparity_transform(True)
disparity_to_depth = rs.disparity_transform(False)
#Spatial:
spatial = rs.spatial_filter()
spatial.set_option(rs.option.holes_fill, 0) # between 0 and 5 def = 0
spatial.set_option(rs.option.filter_magnitude, 2) # between 1 and 5 def=2
spatial.set_option(rs.option.filter_smooth_alpha, 0.5) # between 0.25 and 1 def=0.5
spatial.set_option(rs.option.filter_smooth_delta, 20) # between 1 and 50 def=20
#Temporal:
temporal = rs.temporal_filter()
temporal.set_option(rs.option.filter_smooth_alpha, 0.4)
temporal.set_option(rs.option.filter_smooth_delta, 20)

colorizer = rs.colorizer()

#Get info about depth scaling of the device
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()
print("Depth Scale is: " , depth_scale)

#align to color
align_to = rs.stream.color
align = rs.align(align_to)