from geometry_msgs.msg import Point
from POI import POI
from utils import *

huldra_smaller_points_of_interest = [
    POI('20-2000VF', obj_to_gazebo_point([-117.014, 30.016, 304.500]), orientation_from_euler(0, 0, -pi/2)), # "x": 304500, "y": 117014, "z": 30016
    POI('20-2007VF', obj_to_gazebo_point([-115.739, 31.149, 304.950]), orientation_from_euler(0, 0, -pi)),
    POI('20-2003VF', obj_to_gazebo_point([-114.401, 30.099, 307.900]), orientation_from_euler(0, -pi/4, -pi)), # not successful (angle weighted too little)
    POI('20-2006PL', obj_to_gazebo_point([-112.550, 30.238, 310.292]), orientation_from_euler(0, 0, -pi)),
    POI('20-2001PL', obj_to_gazebo_point([-111.588, 30.156, 310.117]), orientation_from_euler(0, 0, -pi)),
    POI('20-2031PL', obj_to_gazebo_point([-112.772, 29.722, 309.125]), orientation_from_euler(0, -pi/2, 0)),
    POI('20-2001WI', obj_to_gazebo_point([-113.725, 29.311, 301.05]), orientation_from_euler(0, 0, -pi/2)), # not successful (3/4 impossible to see, the last one due to mission outer edge of walkway)
    POI('20-2002WI', obj_to_gazebo_point([-113.725, 29.311, 305.1]), orientation_from_euler(0, 0, pi/2)), # not successful
    POI('20-2003WI', obj_to_gazebo_point([-111.975, 29.602, 304.68]), orientation_from_euler(0, -pi/4, pi/2)), # not successful
    POI('20-2004WI', obj_to_gazebo_point([-113.725, 29.581, 302.844]), orientation_from_euler(0, -pi/4, -pi)), # not successful
    POI('20-2005WI', obj_to_gazebo_point([-113.425, 29.445, 305.985]), orientation_from_euler(0, -pi/4, -pi)),
    POI('20-2006WI', obj_to_gazebo_point([-113.703, 29.456, 306.387]), orientation_from_euler(0, -pi/4, -pi)),
    POI('20-2001VF', obj_to_gazebo_point([-115.739, 30.729, 304.041]), orientation_from_euler(0, -pi/2, 0)),
    POI('20-2008VF', obj_to_gazebo_point([-115.739, 31.296, 305.2]), orientation_from_euler(0, 0, -pi)),
    POI('43-4507VF', obj_to_gazebo_point([-118.245, 31.027, 304.477]), orientation_from_euler(0, 0, -pi))
]

huldra_medium_points_of_interest = [
    POI('27-4453PV', obj_to_gazebo_point([-128.059, 47.474, 297.49]), orientation_from_euler(0, 0, pi/2)),
    POI('27-4510PV', obj_to_gazebo_point([-123.157, 47.268, 295.153]), orientation_from_euler(0, 0, pi)),
    POI('27-4511PV', obj_to_gazebo_point([-124.146, 47.289, 294.064]), orientation_from_euler(0, 0, -pi/2)),
    POI('27-4512PV', obj_to_gazebo_point([-123.157, 47.627, 294.496]), orientation_from_euler(0, 0, pi)),
    POI('27-4541PV', obj_to_gazebo_point([-123.157, 47.533, 295.814]), orientation_from_euler(0, 0, -pi/2)),
    POI('27-4542PV', obj_to_gazebo_point([-123.157, 47.605, 295.815]), orientation_from_euler(0, 0, pi)),
    POI('27-4501PV', obj_to_gazebo_point([-132.172, 47.435, 297.544]), orientation_from_euler(0, 0, pi/2)),
    POI('27-4502PV', obj_to_gazebo_point([-130.951, 47.447, 297.544]), orientation_from_euler(0, 0, pi/2)),
    POI('27-4514PV', obj_to_gazebo_point([-131.238, 47.041, 297.309]), orientation_from_euler(0, 0, pi/2)),
    POI('27-4503PV', obj_to_gazebo_point([-132.272, 47.434, 298.444]), orientation_from_euler(0, 0, -pi/2)),
    POI('27-4504PV', obj_to_gazebo_point([-130.951, 47.447, 298.444]), orientation_from_euler(0, 0, -pi/2)),
    POI('27-4515PV', obj_to_gazebo_point([-131.238, 47.041, 298.68]), orientation_from_euler(0, 0, -pi/2)),
    POI('43-4505VF', obj_to_gazebo_point([-134.195, 47.407, 297.545]), orientation_from_euler(0, 0, -3*pi/4)),
    POI('43-4516VF', obj_to_gazebo_point([-133.208, 47.918, 297.545]), orientation_from_euler(0, 0, 0)),
    POI('43-4506VF', obj_to_gazebo_point([-134.195, 47.407, 298.445]), orientation_from_euler(0, 0, -3*pi/4)),
    POI('43-4517VF', obj_to_gazebo_point([-133.208, 47.918, 298.445]), orientation_from_euler(0, 0, -pi/2)),
    POI('43-4508VF', obj_to_gazebo_point([-137.273, 47.423, 298.449]), orientation_from_euler(0, 0, pi/2)),
    POI('50-4581WS', obj_to_gazebo_point([-134.9, 45.422, 296.389]), orientation_from_euler(0, 0, 0)),
    POI('50-4567WS', obj_to_gazebo_point([-137.894, 45.813, 297.46]), orientation_from_euler(0, 0, -pi/2)),
    POI('50-4565WS', obj_to_gazebo_point([-128.529, 47.95, 298.841]), orientation_from_euler(0, 0, pi)),
    POI('53-4571WD', obj_to_gazebo_point([-118.32, 47.376, 298.638]), orientation_from_euler(0, 0, -pi/2)),
    POI('63-4490AI', obj_to_gazebo_point([-115.725, 45.929, 293.101]), orientation_from_euler(0, 0, 0)),
    POI('63-4489AI', obj_to_gazebo_point([-115.853, 45.929, 293.101]), orientation_from_euler(0, 0, pi)),
    POI('63-4477AI', obj_to_gazebo_point([-120.006, 45.567, 299.356]), orientation_from_euler(0, 0, pi/2)),
    POI('63-4478AI', obj_to_gazebo_point([-123.182, 45.567, 299.356]), orientation_from_euler(0, 0, pi/2)),
    POI('63-4460AP', obj_to_gazebo_point([-118.87, 47.396, 298.545]), orientation_from_euler(0, 0, -pi/2)),
    POI('63-4461AP', obj_to_gazebo_point([-118.87, 47.554, 298.704]), orientation_from_euler(0, -pi/2, 0)),
    POI('63-4479AI', obj_to_gazebo_point([-125.856, 45.567, 299.356]), orientation_from_euler(0, 0, pi/2)),
    POI('63-4480AI', obj_to_gazebo_point([-132.425, 45.567, 300.0]), orientation_from_euler(0, 0, pi/2)),
    POI('63-4481AI', obj_to_gazebo_point([-122.858, 48.435, 298.747]), orientation_from_euler(0, 0, pi)),
    POI('63-4482AI', obj_to_gazebo_point([-125.312, 48.435, 295.485]), orientation_from_euler(0, 0, 0)),
    POI('64-4462GI', obj_to_gazebo_point([-118.62, 47.396, 298.545]), orientation_from_euler(0, 0, -pi/2)),
    POI('64-4463GI', obj_to_gazebo_point([-118.62, 47.554, 298.704]), orientation_from_euler(0, -pi/2, 0)),
]

huldra_mini_points_of_interest = [
    POI('62-2828OF', obj_to_gazebo_point([-94.314, 25.678, 281.183]), orientation_from_euler(0, 0, 0)),
]
