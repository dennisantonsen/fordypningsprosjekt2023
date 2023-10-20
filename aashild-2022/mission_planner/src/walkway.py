
import numpy as np
from utils import *

class Walkway():
    def __init__(
        self, 
        models_path      = '../../resources/huldra-models/',
        walkway_path     = 'huldra-mini-walkway/meshes/',
        walkway_filename = 'huldra-mini-walkway.obj',
        ):

        walkway_file = models_path + walkway_path + walkway_filename

        self.walkway_line = self._find_walkway_line(walkway_file)
    
    def get_walkway_line(self):
        return self.walkway_line

    def normal(self, p1, p2, p3):
        v1 = p2-p1
        v2 = p3-p1
        n = np.cross(v1,v2) #kanskje snu denne tilbake
        return n

    def obj_normals(self, faces):
        normals = []
        for f in faces:
            p1 = np.asarray(faces[f[0]])
            p2 = np.asarray(faces[f[1]])
            p3 = np.asarray(faces[f[2]])
            normals.append(self.normal(p1,p2,p3))
        return normals




    def _find_walkway_line(self, walkway_file):
        #Read vertices, vertex normals and faces from file
        with open(walkway_file) as file:
            lines = file.readlines()
        vertices_n = len([line for line in lines if line.startswith('v ')])
        faces_n = len([line for line in lines if line.startswith('f ')])
    
        vertices = []
        faces = []
        vertex_normals = []
        for line in lines:
            if line.startswith('v '):
                line = line.strip('v ')
                line = line.strip('\n')
                line_splitted = line.split(' ')
                for i in range(0,len(line_splitted)):
                    line_splitted[i] = float(line_splitted[i])
                vertices.append(line_splitted)
            

            elif line.startswith('f '):
                line = line.strip('f ')
                line = line.strip('\n')
                line_splitted = line.split(' ')
    
                for i in range(0,len(line_splitted)):
                    line_splitted[i] = int(line_splitted[i].split('//')[0])
                
                faces.append(line_splitted)
            elif line.startswith('vn '):
                line = line.strip('vn ')
                line = line.strip('\n')
                line_splitted = line.split(' ')
                for i in range (0,len(line_splitted)):
                    line_splitted[i] = float(line_splitted[i])
                vertex_normals.append(line_splitted)
        

        face_normals = self.obj_normals(faces)

        # Convert face normals to gazebo coordinates
        face_normals_converted = [0] * len(face_normals)
        for i in range (0,len(face_normals)):
            face_normals_converted[i] = obj_to_gazebo_coordinates_only_roll(face_normals[i])
        faces_upwards_indices = []
        
        # Remove negative faces
        for i in range(0, len(face_normals_converted)):
            if face_normals_converted[i][2] > 0:
                faces_upwards_indices.append(i)
       # Find face centroids 
        face_centroids = []
        for f in range (0, len(faces)):
            surface_vertices = []
            for v in range (0,3):
                surface_vertices.append(vertices[faces[f][v]-1])
            x = (surface_vertices[0][0]+surface_vertices[1][0]+surface_vertices[2][0])/3
            y = (surface_vertices[0][1]+surface_vertices[1][1]+surface_vertices[2][1])/3
            z = (surface_vertices[0][2]+surface_vertices[1][2]+surface_vertices[2][2])/3
            face_centroids.append([x,y,z])

        
        # Convert face centroids to gazebo coordinates
        face_centroids_converted = [0]* len(face_centroids)
        for i in range (0,len(face_centroids)):
            face_centroids_converted[i] = obj_to_gazebo_coordinates(face_centroids[i])

        face_centroids_upwards = [0]*len(faces_upwards_indices)
        for i, index in enumerate(faces_upwards_indices):
            face_centroids_upwards[i] = face_centroids_converted[index]

        # Find coordinate points
        points = [None] * len(face_centroids_upwards)
        for i in range (0,len(face_centroids_upwards)):
            
            points[i] = [face_centroids_upwards[i][0], face_centroids_upwards[i][1], face_centroids_upwards[i][2]] #Point()


        #Sort points based on x coordinate
        sorted_points = points
        
        for i in range (0,len(points)):
            
            for j in range (i+1,len(points)-i-1):
                if sorted_points[j][0] > sorted_points[j-1][0]:
                    sorted_points[j], sorted_points[j-1] = sorted_points[j-1], sorted_points[j]
        

        # Convert sorted points into Point objects to be used in mission_planner module
        inspection_points = [None] * len(sorted_points)
        for i in range (0,len(sorted_points)):
            inspection_points[i] = Point(points[i][0], points[i][1], points[i][2])
        
        return inspection_points
    
    def get_points_along_walkway_with_resolution(self, resolution):
         # Note: "resolution" means distance between the returned points.
         #       This means that (counter intuitively, I know) that e.g. resolution=10 actually means a lower resolution than resolution=1.
        
        resolution_points = []
        resolution_points.append(self.walkway_line[0])
        unused_distance_previous_iteration = 0
        for i in range(0, len(self.walkway_line) - 1):
            p0 = self.walkway_line[i]
            p1 = self.walkway_line[i + 1]

            distance_p0_p1 = get_distance_between_points(p0, p1)
            distance_walked_from_p0 = 0
            next_distance_to_walk = resolution - unused_distance_previous_iteration

            j = 1
            while distance_p0_p1 - (distance_walked_from_p0 + next_distance_to_walk) > 0:
                resolution_points.append(get_point_between_at_distance(p0, p1, distance_walked_from_p0 + next_distance_to_walk))
                distance_walked_from_p0 += next_distance_to_walk

                next_distance_to_walk = resolution
                unused_distance_previous_iteration = 0
                j += 1
                unused_distance_previous_iteration = unused_distance_previous_iteration + distance_p0_p1 - distance_walked_from_p0
        
        resolution_points.append(self.walkway_line[-1])

        return resolution_points


