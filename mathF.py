## Math functions
import numpy as np
import math
from scipy.spatial import Delaunay
from functools import reduce
from cameraF import defineTriangulation, createBox, plotGeometriesWithOriginVectors

# define a cube from the given points, with a specific height from the first selected point
def rectangleFromPoints(crop_points, height, distance_to_edge):
    if(distance_to_edge >= 0 and distance_to_edge != 1):
        crop_points = moveByDistanceFromClosestTwoPoints(crop_points, distance_to_edge)[0]

    points_raw = np.array([
    #Vertices Polygon1
    [crop_points[0][0], crop_points[0][1], crop_points[0][2] + height], # face-topright
    [crop_points[1][0], crop_points[1][1], crop_points[0][2] + height], # face-topleft
    [crop_points[2][0], crop_points[2][1], crop_points[0][2] + height], # rear-topleft
    [crop_points[3][0], crop_points[3][1], crop_points[0][2] + height], # rear-topright
    # Vertices Polygon2
    [crop_points[0][0], crop_points[0][1], crop_points[0][2] - height], # face-bottomright
    [crop_points[1][0], crop_points[1][1], crop_points[0][2] - height], # face-bottomleft
    [crop_points[2][0], crop_points[2][1], crop_points[0][2] - height], # rear-bottomleft
    [crop_points[3][0], crop_points[3][1], crop_points[0][2] - height], # rear-bottomright
    ]).astype("float64")

    return points_raw


# calculate and return distances from points and points moved by some distance factor in an array
def moveByDistanceFromClosestTwoPoints(points, distanceFactor):
    eachPointVectors = []
    eachPointDistances = []
    for i in range(len(points)):
        vectors = []
        distance = []
        for j in range(len(points)):
            if(i == j):
                continue
            
            vectors.append([points[i][0] - points[j][0], points[i][1] - points[j][1]])
            distance.append(np.linalg.norm(points[i]-points[j]))
        eachPointVectors.append(vectors)
        eachPointDistances.append(distance)
    
    # find the shortest two distances and move the original points by a fraction of that vector
    for i, pointVectors in enumerate(eachPointVectors):
        longestDistance = max(eachPointDistances[i])

        for j, vector in enumerate(pointVectors):
            if(eachPointDistances[i][j] == longestDistance):
                continue

            points[i][0] += vector[0] / distanceFactor
            points[i][1] += vector[1] / distanceFactor
    
    return [points, eachPointDistances]


# Get a plane normal from a selected set of points
def createPlaneParametersFromPoints(points):
    p0, p1, p2, p3 = points
    x0, y0, z0 = p0
    x1, y1, z1 = p1
    x2, y2, z2 = p2

    ux, uy, uz = [x1-x0, y1-y0, z1-z0]
    vx, vy, vz = [x2-x0, y2-y0, z2-z0]

    a = uy*vz-uz*vy  # If the problem in line 84 happens only for hand selected points try negating value of a instead
    b = uz*vx-ux*vz
    c = ux*vy-uy*vx
    d = -a*x0 - b*y0 - c*z0

    return [a, b, c, d] if c >= 0 else [-a, -b, -c, -d]
    

# Get vector angle of two vectors
def vectorAngle(u, v):
    return np.arccos(np.dot(u,v) / (np.linalg.norm(u)* np.linalg.norm(v)))


# Calculate rotation axis
def rotationAxis(floor):
    [a, b, c, d] = floor
    plane_normal_length = math.sqrt(a**2 + b**2 + c**2)
    u1 = b / plane_normal_length
    u2 = -a / plane_normal_length
    return (u1, u2, 0)


# getRotationMatrix from one vector to another. Source: https://stackoverflow.com/questions/45142959/calculate-rotation-matrix-to-align-two-vectors-in-3d-space
def getRotationMatrix(v1, v2):
    """ Find the rotation matrix that aligns v1 to v2
    :param v1: A 3d "source" vector
    :param v2: A 3d "destination" vector
    :return mat: A transform matrix (3x3) which when applied to v1, aligns it with v2.
    """
    a, b = (v1 / np.linalg.norm(v1)).reshape(3), (v2 / np.linalg.norm(v2)).reshape(3)
    v = np.cross(a, b)
    if any(v):  # if v != null
        c = np.dot(a, b)
        s = np.linalg.norm(v)
        kmat = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
        rotation_matrix = np.eye(3) + kmat + kmat.dot(kmat) * ((1 - c) / (s ** 2))
        return rotation_matrix
    else:
        return np.eye(3)    # Return identity matrix


# Calculate the Delaunay triangulation mesh
def calculateTriangulation(pcd):
    xyz = np.asarray(pcd.points)
    xy_catalog = []
    for point in xyz:
        xy_catalog.append([point[0], point[1]])
    
    mesh = Delaunay(np.array(xy_catalog))
    return [xyz, mesh]


# Code explained at https://jose-llorens-ripolles.medium.com/stockpile-volume-with-open3d-fa9d32099b6f
def volumeUnderTriangle(triangle):
    p1, p2, p3 = triangle
    x1, y1, z1 = p1
    x2, y2, z2 = p2
    x3, y3, z3 = p3
    return abs((z1+z2+z3)*(x1*y2-x2*y1+x2*y3-x3*y2+x3*y1-x1*y3)/6)


# Code explained at https://jose-llorens-ripolles.medium.com/stockpile-volume-with-open3d-fa9d32099b6f
def getTrianglesVertices(triangles, vertices):
    triangles_vertices = []
    for triangle in triangles:
        new_triangles_vertices = [vertices[triangle[0]], vertices[triangle[1]], vertices[triangle[2]]]
        triangles_vertices.append(new_triangles_vertices)
    return np.array(triangles_vertices)


# average array
def average(number_array):
    return reduce(lambda a, b: a + b, number_array) / len(number_array)


# crop image, create a triangulation and calculate volume
def preformVolumeCalculations(pcd, empty_container_volume = 0, measurment = -1):
    if(measurment <= 0):
        
        # get the location of the cropped item and create a floor
        floor_height_offset = math.ceil(pcd.get_max_bound()[2] - 2*pcd.get_min_bound()[2])
        floor_width = pcd.get_max_bound()[0] - pcd.get_min_bound()[0]
        floor_height = pcd.get_max_bound()[1] - pcd.get_min_bound()[1]

        # define the floor under the object
        origin_box = createBox(width = 0.1, height = 0.1, depth = 0.1)
        floor_plane = createBox(width = floor_width*2, height = floor_height*2, depth = 0.01)
        floor_plane.translate((-floor_width, -floor_height, -floor_height_offset))
        floor_plane.paint_uniform_color([0.83, 0.83, 0.83])

        plotGeometriesWithOriginVectors([pcd, floor_plane, origin_box])

    ## Start calculating volume
    #  calculate triangulation positions and define it in open3d
    xyz, tMesh = calculateTriangulation(pcd)
    surface = defineTriangulation(xyz, tMesh)

    if(measurment <= 0):
        plotGeometriesWithOriginVectors([surface, floor_plane], True)

    # Calculate volume
    volume = reduce(lambda a, b: a + volumeUnderTriangle(b), getTrianglesVertices(surface.triangles, surface.vertices), 0)

    print(f'The total volume measured is: {round(volume, 3)} m^3. The volume of the stock is: {round(volume - empty_container_volume, 3)} m^3.')
    return volume


# get distance between two points
# imput: array of two n-dimentional points
def get2PointDistance(points):
    return np.linalg.norm(np.asarray(points[0]) - np.asarray(points[1]))
