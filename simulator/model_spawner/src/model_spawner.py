#!/usr/bin/env python3

import os
import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Quaternion, Pose, Point
from generate_valve import Valve
from math import pi
from tf.transformations import quaternion_from_euler

class ModelSpawner():
    # Default offsets for Huldra models
    huldra_model = None
    huldra_offset_x = 116
    huldra_offset_y = 287
    huldra_offset_z = -27
    huldra_offset_yaw = 0
    huldra_offset_pitch = 0
    huldra_offset_roll = pi/2

    # Offsets for Huldra models, from Docker environment variables
    if 'HULDRA_MODEL' in os.environ:
        huldra_model = os.environ['HULDRA_MODEL']
    if 'HULDRA_OFFSET_X' in os.environ:
        huldra_offset_x = float(os.environ['HULDRA_OFFSET_X'])
    if 'HULDRA_OFFSET_Y' in os.environ:
        huldra_offset_y = float(os.environ['HULDRA_OFFSET_Y'])
    if 'HULDRA_OFFSET_Z' in os.environ:
        huldra_offset_z = float(os.environ['HULDRA_OFFSET_Z'])
    if 'HULDRA_OFFSET_YAW' in os.environ:
        huldra_offset_yaw = float(os.environ['HULDRA_OFFSET_YAW'])
    if 'HULDRA_OFFSET_PITCH' in os.environ:
        huldra_offset_pitch = float(os.environ['HULDRA_OFFSET_PITCH'])
    if 'HULDRA_OFFSET_ROLL' in os.environ:
        huldra_offset_roll = float(os.environ['HULDRA_OFFSET_ROLL'])

    huldra_offset_position = Point(x=huldra_offset_x, y=huldra_offset_y, z=huldra_offset_z)

    # Counter for adding valves
    valve_index = 1

    def __init__(self):
        pass
    
    def add_valve(self, x, y, z, yaw):
        rospy.loginfo('Generating valve ...')

        valve = Valve(
            model_path = "/home/catkin_ws/src/model_spawner/src/models/valve",
            x_offset = 0,
            y_offset = 0,
            z_offset = 1,
            roll_offset = 0,
            pitch_offset = 0,
            yaw_offset = 0,
            pipe_radius = 0.2,
            pipe_length = 1,
            valve_pipe_radius = 0.2,
            valve_pipe_length = 0.5,
            valve_radius = 0.2
        )
        valve.generate_sdf()

        rospy.loginfo('Adding generated valve ...')

        with open('/home/catkin_ws/src/model_spawner/src/models/valve/model.sdf') as file:
            model_xml = file.read()
        
        q = quaternion_from_euler(0, 0, yaw)
        orientation = Quaternion(q[0],q[1],q[2],q[3])
        pose = Pose(Point(x=x, y=y, z=z), orientation)

        spawn_sdf_model = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)
        spawn_sdf_model("valve" + str(self.valve_index), model_xml, "", pose, "world")
        self.valve_index += 1
    
    def add_huldra_model(self, model_name):
        # Add Huldra model with the specified name
        rospy.loginfo('Adding Huldra model: ' + model_name + '...')

        with open('/home/catkin_ws/src/model_spawner/src/huldra-models/' + model_name + '/model.sdf') as file:
            model_xml = file.read()
        
        q = quaternion_from_euler(self.huldra_offset_roll, self.huldra_offset_pitch, self.huldra_offset_yaw)
        orientation = Quaternion(q[0], q[1], q[2], q[3])
        pose = Pose(self.huldra_offset_position, orientation)

        spawn_sdf_model = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)
        spawn_sdf_model(model_name, model_xml, "", pose, "world")
    
    def add_huldra_model_from_env(self):
        # Add the Huldra model specified in config.env at project root
        if self.huldra_model is not None:
            self.add_huldra_model(self.huldra_model)
        else:
            print('Not adding Huldra model due to environment variable not set.')

if __name__ == '__main__':
    try:
        rospy.init_node('model_spawner', anonymous=True)
        rospy.loginfo("Waiting for service gazebo/spawn_sdf_model ...")
        rospy.wait_for_service("gazebo/spawn_sdf_model")
        spawner = ModelSpawner()
        spawner.add_huldra_model_from_env()
    except rospy.ROSInterruptException:
        pass