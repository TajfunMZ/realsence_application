import open3d as o3d
import pyrealsense2 as rs
import numpy as np
from scipy.spatial import Delaunay
from functools import reduce
import copy

dataset_path = "C:/Users/Student/Desktop/"

# Create a floor plane virtual object, which is helpful when visually inspecting alignment of trailer pointcloud with XY plane
floor_plane = o3d.geometry.TriangleMesh.create_box(width=1.0, height=1.0, depth=0.01)
floor_plane.translate((-0.5, -0.5, -0.5))
floor_plane.paint_uniform_color([0.4, 0.7, 0.8])

# D435 pipeline, used for streaming depth data from camera to python
pipe = rs.pipeline()

# Initialize D435 depth camera, capture only depth data, not RGB
def init_camera():

    cfg = rs.config()
    ctx = rs.context()
    cfg.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 30) # Enable depth stream
    dev = ctx.query_devices()
    advncd_mode_cfg = rs.rs400_advanced_mode(dev[0])
    
    # jsonObj = json.load(open(dataset_path + "Resources/D435_params.json"))
    # json_string = str(jsonObj).replace("'", '\"')
    # advncd_mode_cfg.load_json(json_string) # Use HIGH ACCURACY preset on D435

    # Start capturing and apply some additional settings
    profile = pipe.start(cfg)
    depth_sensor = profile.get_device().first_depth_sensor()
    depth_sensor.set_option(rs.option.enable_auto_exposure, 1); #Disble auto exposure
    depth_sensor.set_option(rs.option.depth_units, 0.0002) # Set depth units from 1mm to 0.2mm. This decreases camera range but increases depth accuracy
    depth_table = advncd_mode_cfg.get_depth_table()
    depth_table.depthClampMax = 65535   # Set max range to max possible value at current depth units (16 bit)
    advncd_mode_cfg.set_depth_table(depth_table)

# Capture a pointcloud using a connected D435 camera (camera has to be initialized beforehand)
def capture_pcd():
    # Capture frame from camera and convert it into an Open3D depth image
    frame = pipe.wait_for_frames()
    depth_frame = frame.get_depth_frame()
    depth_image = np.asanyarray(depth_frame.get_data())
    depth_image_o3d = o3d.geometry.Image(depth_image)

    # From the depth image create a Point cloud, taking into account the camera intrinsics
    intrinsics = frame.profile.as_video_stream_profile().intrinsics
    pinhole_camera_intrinsic = o3d.camera.PinholeCameraIntrinsic(
        intrinsics.width, intrinsics.height, intrinsics.fx, intrinsics.fy, intrinsics.ppx, intrinsics.ppy)
    pcd = o3d.geometry.PointCloud.create_from_depth_image(depth_image_o3d,pinhole_camera_intrinsic)
    pcd.transform([[1, 0, 0, 0],
                    [0, -1, 0, 0],
                    [0, 0, -1, 0],
                    [0, 0, 0, 1]])
    pcd = copy.deepcopy(pcd)
    return pcd

# Create a 3D bounding box for cropping a pointcloud
def rectangle_crop(start_position,X_WIDTH,Y_WIDTH,Z_WIDTH):
    points_raw = np.array([
    #Vertices Polygon1
    [start_position[0] + (X_WIDTH / 2), start_position[1] + (Y_WIDTH / 2), start_position[2] + (Z_WIDTH / 2)], # face-topright
    [start_position[0] - (X_WIDTH / 2), start_position[1] + (Y_WIDTH / 2), start_position[2] + (Z_WIDTH / 2)], # face-topleft
    [start_position[0] - (X_WIDTH / 2), start_position[1] - (Y_WIDTH / 2), start_position[2] + (Z_WIDTH / 2)], # rear-topleft
    [start_position[0] + (X_WIDTH / 2), start_position[1] - (Y_WIDTH / 2), start_position[2] + (Z_WIDTH / 2)], # rear-topright
    # Vertices Polygon2
    [start_position[0] + (X_WIDTH / 2), start_position[1] + (Y_WIDTH / 2), start_position[2] - (Z_WIDTH / 2)],
    [start_position[0] - (X_WIDTH / 2), start_position[1] + (Y_WIDTH / 2), start_position[2] - (Z_WIDTH / 2)],
    [start_position[0] - (X_WIDTH / 2), start_position[1] - (Y_WIDTH / 2), start_position[2] - (Z_WIDTH / 2)],
    [start_position[0] + (X_WIDTH / 2), start_position[1] - (Y_WIDTH / 2), start_position[2] - (Z_WIDTH / 2)],
    ]).astype("float64")

    points = o3d.utility.Vector3dVector(points_raw)
    return o3d.geometry.OrientedBoundingBox.create_from_points(points)

def get_triangles_vertices(triangles, vertices):
    # Code explained at https://jose-llorens-ripolles.medium.com/stockpile-volume-with-open3d-fa9d32099b6f
    triangles_vertices = []
    for triangle in triangles:
        new_triangles_vertices = [vertices[triangle[0]], vertices[triangle[1]], vertices[triangle[2]]]
        triangles_vertices.append(new_triangles_vertices)
    return np.array(triangles_vertices)

def volume_under_triangle(triangle):
    # Code explained at https://jose-llorens-ripolles.medium.com/stockpile-volume-with-open3d-fa9d32099b6f
    p1, p2, p3 = triangle
    x1, y1, z1 = p1
    x2, y2, z2 = p2
    x3, y3, z3 = p3
    return abs((z1+z2+z3)*(x1*y2-x2*y1+x2*y3-x3*y2+x3*y1-x1*y3)/6)

def calculate_volume(zero_vol):
    # Code explained at https://jose-llorens-ripolles.medium.com/stockpile-volume-with-open3d-fa9d32099b6f
    #downpdc = copy.deepcopy(cropped_pcd)
    downpdc = cropped_pcd.voxel_down_sample(voxel_size=0.01)    #pohitrim delovanje, izgubim nekaj detajlov
    xyz = np.asarray(downpdc.points)
    xy_catalog = []
    for point in xyz:
        xy_catalog.append([point[0], point[1]])
    tri = Delaunay(np.array(xy_catalog))

    surface = o3d.geometry.TriangleMesh()
    surface.vertices = o3d.utility.Vector3dVector(xyz)
    surface.triangles = o3d.utility.Vector3iVector(tri.simplices)
    surface.paint_uniform_color([1,0.7,0])

    o3d.visualization.draw_geometries([surface,floor_plane], mesh_show_wireframe=True)

    volume = reduce(lambda a, b: a + volume_under_triangle(b),
                    get_triangles_vertices(surface.triangles, surface.vertices), 0)

    # Example of zero_vol is 0.2467
    print(f"Celotni volumen znaša: {round(volume, 3)} m3, volumen kupa pa {round(volume-zero_vol, 3)} m3")

if __name__ == '__main__':
    print("Running...")

    init_camera()
    frame=pipe.wait_for_frames()

    # Capture a new pointcloud
    pcd = capture_pcd()
    # o3d.visualization.draw_geometries([pcd, floor_plane])
    # Scale PCD to correct size (1 Open3D unit = 1 meter)
    pcd.scale(0.225, center=(0, 0, 0))

    # Rotate and translate PCD so that the bottom of the trailer is aligned with the horizontal (XY) plane
    pcd.rotate(pcd.get_rotation_matrix_from_xyz((0.16, 0, 0)), center=(0, 0, 0))
    pcd.translate((0, 0, 0.6))

    # Crop PCD so that only points inside trailer remain
    oriented_bbox = rectangle_crop([0.0, 0, 0], 0.5, 0.5, 0.45)
    # oriented_bbox_visualize = rectangle_crop([0.0, -0.44, 0.25], 2.0, 1.15, 0.35)
    
    oriented_bbox.color = (1, 0, 0)

    cropped_pcd = pcd.crop(oriented_bbox)

    # Remove noise from cropped PCD
    #cropped_pcd = cropped_pcd.voxel_down_sample(voxel_size=0.02) # Vzorcenje oblaka
    #cl, ind = cropped_pcd.remove_statistical_outlier(nb_neighbors=100,std_ratio=2.0)

    #cl, ind = cropped_pcd.remove_radius_outlier(nb_points=3000, radius=0.10)
    #cropped_pcd = cropped_pcd.select_by_index(ind)

    # Draw the PCD
    #o3d.visualization.draw_geometries([pcd])
    o3d.visualization.draw_geometries([pcd, oriented_bbox, floor_plane])

    # Calculate volume
    # calculate_volume(float(sys.argv[1]))
    calculate_volume(0.025)
