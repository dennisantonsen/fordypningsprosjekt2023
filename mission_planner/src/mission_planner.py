#!/usr/bin/env python3
import datetime
import json
from math import inf
from time import sleep, time

import constants
import numpy as np
import pandas as pd
import rospy
import trimesh
from POI import POI
from utils import *
from walkway import Walkway


class MissionPlanner:
    def __init__(self, model_name, points_of_interest = []):
        self.model_name = model_name
        self.points_of_interest = points_of_interest

        self.scores_distance = [0] * len(points_of_interest)
        self.scores_angle = [0] * len(points_of_interest)
        self.scores_obstacles = [0] * len(points_of_interest)
        self.scores_obstacles_around = [0] * len(points_of_interest)
        self.scores_total = [0] * len(points_of_interest)

        self.ray_time_total = 0

        self.count_discarded_too_big_angle = 0
        self.count_discarded_all_inf = 0
        self.pois_discarded_inf = []

        self.wp = Walkway(
            models_path = '/home/catkin_ws/src/mission_planner/src/huldra-models/',
            walkway_path = model_name + '-walkway/meshes/',
            walkway_filename = model_name + '-walkway.obj'
        )


        self.walkway_line = self.wp.get_walkway_line()
        publish_markers(self.walkway_line)
        publish_line_marker(self.walkway_line, scale = 0.02)

        print('Walkway line:')
        for i in range(0, len(self.walkway_line)):
            x = str(self.walkway_line[i].x)
            y = str(self.walkway_line[i].y)
            z = str(self.walkway_line[i].z)
            print('Point(' + x + ', ' + y + ', ' + z + '),')

        print('Loading mesh file...')
        self.mesh_file = '/home/catkin_ws/src/mission_planner/src/huldra-models/' + model_name + '/meshes/' + model_name + '.obj'
        self.mesh = trimesh.load(self.mesh_file, force='mesh')
        print('Mesh file loaded.')

    def find_inspection_poses(self):
        inspection_poses = [None] * len(self.points_of_interest)
        
        for i in range(0, len(self.points_of_interest)):
            print('POI no.', i+1, '/', len(self.points_of_interest), ':')
            inspection_poses[i] = self.find_inspection_pose(self.points_of_interest[i])

        return inspection_poses

    def find_inspection_pose(self, poi: POI):

        # TODO: Redo all of this with higher resolution, if we find any interesting parts of the walkway

        resolution = 1 #0.1
        possible_inspection_points = self.wp.get_points_along_walkway_with_resolution(resolution)
        publish_markers(possible_inspection_points, r=0.1, g=0.5, b=0.1)
        print('Finding possible inspection points with resolution', resolution)

        possible_inspection_points_scores = [0] * len(possible_inspection_points)
        for i in range(0, len(possible_inspection_points)):
            print(i+1, '/', len(possible_inspection_points), ':')

            possible_inspection_point = possible_inspection_points[i]
            score = 0

            distance_to_poi = get_distance_between_points(possible_inspection_point, poi.point)
            angle_towards_poi = self.get_angle_towards_poi(poi, possible_inspection_point)

            # Filtering before rays:
            if angle_towards_poi > pi/4:
               possible_inspection_points_scores[i] = inf
               self.count_discarded_too_big_angle += 1
               continue

            # Filtering on distance
            if distance_to_poi > 20:
                possible_inspection_points_scores[i] = inf
                self.count_discarded_too_big_angle += 1
                continue


            obstacles_count = self.get_obstacles_between(possible_inspection_point, poi, self.mesh_file)


            # Lower score is better
            score = (100)*obstacles_count + (100)*angle_towards_poi + (1)*distance_to_poi # TODO: Find a better score function

            
            possible_inspection_points_scores[i] = score
        

        if min(possible_inspection_points_scores) == inf:
            self.count_discarded_all_inf += 1
            print('Lowest score: Inf')
            self.pois_discarded_inf.append(poi.identifier)

        # Publish markers of possible inspection points with color grading based on its score
        colors = values_to_colors(possible_inspection_points_scores)
        for i in range(0, len(possible_inspection_points)):
            publish_marker(possible_inspection_points[i], r=colors[i][0], g=colors[i][1], b=colors[i][2], scale=0.2)

        possible_inspection_points_sorted = [x for _, x in sorted(zip(possible_inspection_points_scores, possible_inspection_points), key=lambda pair: pair[0])]
        inspection_point = possible_inspection_points_sorted[0]
        inspection_orientation = get_orientation_towards_point(inspection_point, poi.point)
        inspection_pose = Pose(inspection_point, inspection_orientation)

        publish_marker(inspection_point, r=0.0, g=1.0, b=0.0, scale=0.3)

        return inspection_pose
    
    #def will_inspector_crash_at_point(self, point):
    #    # TODO: Check if robot crashes at the given point. E.g. are there anything placed on the walkway?
    #    return False # Dummy return value
    
    def get_obstacles_between(self, p1: Point, poi: POI, obj_file):
        ray_time_start = time()

        p2 = poi.point
        ray_origin = np.array(gazebo_to_obj_coordinates([p1.x, p1.y, p1.z]))
        ray_origin_point = Point(ray_origin[0], ray_origin[1], ray_origin[2])
        ray_direction = np.array(normalize(gazebo_to_obj_coordinates_only_roll([p2.x - p1.x, p2.y - p1.y, p2.z - p1.z])))

        intersections, _, face_indices = self.mesh.ray.intersects_location(ray_origins=[ray_origin], ray_directions=[ray_direction])
    

        distance_p1_p2 = get_distance_between_points(p1, p2)

        # TODO: Move this our of this function. Only needs to be done once per POI.
        low_face_index, high_face_index = find_face_indices_covering_model(obj_file, poi.identifier)

        # Discard intersections if distance from inspection point is greater than the distance inspection point - POI
        # Discard intersections if the intersection face is a part of the POI model
        intersections_indices = []
        for i in range(0, len(intersections)):
            intersection_point = Point(x = intersections[i][0], y = intersections[i][1], z = intersections[i][2])
            distance_ray_origin_intersection = get_distance_between_points(ray_origin_point, intersection_point)
            
            if distance_ray_origin_intersection >= distance_p1_p2:
                continue
            if face_indices[i] > low_face_index and face_indices[i] < high_face_index:
                continue
            
            intersections_indices.append(i)
        
        ray_time_now = time()
        self.ray_time_total += (ray_time_now - ray_time_start)

        return len(intersections_indices)

    def get_obstacles_around(self, p1: Point, poi: POI, obj_file):
        # Find a circle around poi, facing p1. Find n points evenly distributed on the circle.
        s = poi.point
        r = 0.1 #0.1
        poi_p1_direction = [p1.x - poi.point.x, p1.y - poi.point.y, p1.z - poi.point.z]
        count = 4 #4
        circle_points = get_points_on_circle(s, r, poi_p1_direction, count)

        # Then, count obstacles between p1 and poi along N vectors from poi to the circle.
        obstacles_count = 0
        for point in circle_points:
            obstacles_count += self.get_obstacles_between(p1, poi, obj_file)
        
        print('Obstacle count around poi with', count, 'rays on a circle:', obstacles_count)

        return obstacles_count
    
    def get_angle_towards_poi(self, poi: POI, point):
        orientation_poi_point = get_orientation_towards_point(poi.point, point)
        angle = get_angle_between_quaternions(poi.orientation, orientation_poi_point)

        return angle

    def get_score_from_image_analysis(self, inspection_point, poi_point):
        # Use camera plugin to take an image of POI from inspection position, apply image analysis and get a score.
        return 0 # Dummy return value

    def get_inspection_poses(self):
        inspection_poses = [None]*len(self.points_of_interest)
        for i in range(0, len(self.points_of_interest)):
            inspection_poses[i] = self.search_for_inspection_pose(self.points_of_interest[i])
            create_mission_file(i, self.points_of_interest[i], inspection_poses[i])
            
            publish_marker(points_of_interest[i].point)

        return inspection_poses

    def search_for_inspection_pose(self, poi_point):
        resolution = 5
        height_interval = np.linspace(0,1,5)
        possible_inspection_points = self.wp.get_points_along_walkway_with_resolution(resolution)
        distances = [None]*len(possible_inspection_points)
        min_distance = inf
        current_search_point_score = 0

        #Find the possible inspection point closest to the POI to decide where to start search
        for i in range (0,len(possible_inspection_points)):
            distances[i] = get_distance_between_points(possible_inspection_points[i], poi_point.point)
            if distances[i] < min_distance:
                min_distance = distances[i]
                start_search_point = possible_inspection_points[i]
                start_search_point_score = min_distance

        
        # Create empty dataframe to store Coordinates and Scores: 
        searches_df = pd.DataFrame({'Coordinate' : [], 'Score' : []})

        #Check angle and conduct search
        current_angle = mission_planner.get_angle_towards_poi(poi_point, start_search_point)
        last_angle = inf
        poi_orientation = [poi_point.orientation.x, poi_point.orientation.y, poi_point.orientation.z, poi_point.orientation.w]
        
        last_search_point = start_search_point
        current_search_point = start_search_point
        current_search_point_score = start_search_point_score

        #Add current datapoints score into dataframe
        new_row = pd.DataFrame([[current_search_point, current_search_point_score]], columns=['Coordinate', 'Score'])
        searches_df = pd.concat([searches_df, new_row]).reset_index(drop=True)
        #print("dataframe: ", searches_df)

        while abs(current_angle) < abs(last_angle):
            temp_search_point = [current_search_point.x,current_search_point.y,current_search_point.z, 0]
            temp_search_point = -quaternion_multiply(poi_orientation, temp_search_point)
            temp_search_point = quaternion_multiply(temp_search_point, quaternion_conjugate(poi_orientation))
            temp_search_point = Point(temp_search_point[0], temp_search_point[1], temp_search_point[2])
            #Project temp search variable onto walkway
            for i in range (0,len(possible_inspection_points)):
                distances[i] = get_distance_between_points(possible_inspection_points[i], temp_search_point)
                if distances[i] < min_distance:
                    min_distance = distances[i]
                    current_search_point = possible_inspection_points[i]
                    current_search_point_score = min_distance

            last_angle = current_angle

            current_angle = mission_planner.get_angle_towards_poi(poi_point, current_search_point)

            current_search_point_score += 100*current_angle
            new_row = pd.DataFrame([[current_search_point, current_search_point_score]], columns=['Coordinate', 'Score'])
            searches_df = pd.concat([searches_df, new_row]).reset_index(drop=True)

            for i in range(1,len(height_interval)):
                current_search_point_vertical = Point(current_search_point.x, current_search_point.y, current_search_point.z+height_interval[i])
                print("current search: ", current_search_point_vertical)
                new_row = pd.DataFrame([[current_search_point_vertical, current_search_point_score]], columns=['Coordinate', 'Score'])
                searches_df = pd.concat([searches_df, new_row]).reset_index(drop=True)
                
        for i in range(0,len(searches_df.index)):
            current_search_point = searches_df['Coordinate'][i]
            current_search_point_score = searches_df['Score'][i]
            obstacles_around_count = self.get_obstacles_around(current_search_point, poi_point, self.mesh_file) 

            current_search_point_score += 100*obstacles_around_count

            searches_df.at[i, 'Score'] = current_search_point_score
        
        #Sort dataframe by score, low to high
        sorted_searches_df = searches_df.sort_values(by=['Score'])
        print("Sorted df: ", sorted_searches_df)

        #Collect the coordinates of the point with lowest score
        current_search_point = sorted_searches_df['Coordinate'].iloc[0]
        inspection_orientation = get_orientation_towards_point(current_search_point, poi_point.point)
        inspection_pose = Pose(current_search_point, inspection_orientation)

        

        return inspection_pose


def get_vertical_scores(point, score, interval_list, poi_point):
    vertical_searches_df = pd.DataFrame({'Coordinate' : [], 'Score' : []})
    new_row = pd.DataFrame([[point, score]], columns=['Coordinate', 'Score'])
    vertical_searches_df = pd.concat([searches_df, new_row]).reset_index(drop=True)
    for i in range (0,len(interval_list)):
        point = Point(point.x, point.y, point.z+interval_list[i])
        obstacles_around_count = self.get_obstacles_around(current_search_point, poi_point, self.mesh_file)
        
        #Add current datapoints score into dataframe
        new_row = pd.DataFrame([[current_search_point, current_search_point_score]], columns=['Coordinate', 'Score'])
        vertical_searches_df = pd.concat([vertical_searches_df, new_row]).reset_index(drop=True)



def create_mission_file(id, poi_point, inspection_pose):
    mission_dict = {
        "id": id,
        "tasks": [
        {
            "steps": [
                {
                    "type": "drive_to_pose",
                    "pose": {
                        "position": {
                            "x": inspection_pose.position.x,
                            "y": inspection_pose.position.x,
                            "z": inspection_pose.position.z,
                            "frame": {"name": "asset"},
                        },
                        "orientation": {
                            "x": inspection_pose.orientation.x,
                            "y": inspection_pose.orientation.y,
                            "z": inspection_pose.orientation.z,
                            "w": inspection_pose.orientation.w,
                            "frame": {"name": "asset"},
                        },
                        "frame": {"name": "asset"},
                    },
                },
                {
                    "type": "take_image",
                    "target": {"x": poi_point.orientation.x, "y": poi_point.orientation.y, "z": poi_point.orientation.z, "frame": {"name": "asset"}},
                },
            ],
            },
        ],
    }
    #Write dictionary to json file
    outfile = open('/home/catkin_ws/src/mission_planner/output/missiondict'+str(id)+'.json', "w", encoding ='utf8')
    json.dump(mission_dict, outfile)
    outfile.close()



if __name__ == '__main__':
    start_timer = time()
    rospy.init_node('mission_planner', anonymous=True)
    trimesh.util.attach_to_log()
    try:
        huldra_model = None
        if 'HULDRA_MODEL' in os.environ:
            huldra_model = os.environ['HULDRA_MODEL']
        
        points_of_interest = []
        if huldra_model == 'huldra-smaller':
            points_of_interest = constants.huldra_smaller_points_of_interest

        mission_planner = MissionPlanner(huldra_model, points_of_interest)

        time_before = time()
        inspection_poses = mission_planner.get_inspection_poses()
        time_after = time()
        time_difference = time_after - time_before
        time_difference_formatted = str(datetime.timedelta(seconds=time_difference))
        ray_time_total_formatted = str(datetime.timedelta(seconds=mission_planner.ray_time_total))
        print('Time difference before-after finding inspection points:', time_difference, 'seconds')
        print('Time difference (formatted):', time_difference_formatted)
        print('Total ray time (including ray-around):', mission_planner.ray_time_total)
        print('Total ray time (including ray-around) (formatted):', ray_time_total_formatted)
        print('Number of points discarded due to too big angle:', mission_planner.count_discarded_too_big_angle)
        print('Number of POIs where all insp.points got inf score:', mission_planner.count_discarded_all_inf)
        
        i = 0
        while i <= 3:

            send_to_inspector([point.pose for point in points_of_interest], inspection_poses)
            sleep(1)
            i += 1
        
        stop_timer = time()
        total_time = stop_timer - start_timer
        print("Total time: ", total_time)
    except rospy.ROSInterruptException:
        pass
