#!/usr/bin/env python3

import rospy
from gazebo_msgs.srv import SpawnModel, GetModelState
from gazebo_msgs.msg import ModelState, ModelStates
from sensor_msgs.msg import Image
from geometry_msgs.msg import Quaternion, Pose, Point, PoseArray
from tf.transformations import quaternion_from_euler
from time import sleep, time
import cv2 as cv
from cv_bridge import CvBridge
from pcg_gazebo.simulation import create_object
from pcg_gazebo.generators import WorldGenerator
from pcg_gazebo.task_manager import GazeboProxy

def add_robot_model():
    rospy.loginfo("Waiting for service gazebo/spawn_sdf_model ...")
    rospy.wait_for_service("gazebo/spawn_sdf_model")
    rospy.wait_for_service("gazebo/get_model_state")

    # Test if robot exist
    robot_model_state_service = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
    robot_model = robot_model_state_service("inspector_robot", "chassis")
    if robot_model.success:
        print("robot exist")
    else:
        # No robot found, create a new one
        rospy.loginfo('Adding Inspector Robot model (this can take some time - camera plugin will be downloaded)...')
        spawn_sdf_model = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)
        with open('/home/catkin_ws/src/inspector/src/inspector_robot/inspector_robot.sdf') as file:
            inspector_robot_xml = file.read()
        
        q = quaternion_from_euler(0, 0, 0)
        orientation = Quaternion(q[0], q[1], q[2], q[3])
        pose = Pose(Point(x=-2, y=-2, z=0), orientation)

        spawn_sdf_model("inspector_robot", inspector_robot_xml, "", pose, "world")
        rospy.loginfo("Inspector Robot model added.")
    

class Inspector():
    def __init__(self, wordGenerator=None):
        self._poi_poses = None
        self._inspection_poses = None
        self._robot_state = None
        self._image_message = None
        self._worldGenerator = wordGenerator

        rospy.loginfo('Listening for POI poses and inspection poses...')
        rospy.Subscriber("inspector/poi_poses", PoseArray, self._store_poi_poses)
        rospy.Subscriber("inspector/inspection_poses", PoseArray, self._store_inspection_poses)
        rospy.Subscriber("gazebo/model_states", ModelStates, self._store_robot_state)
        rospy.Subscriber("inspector_robot/camera/image_raw", Image, self._store_image_message)
    
    def _store_poi_poses(self, poses_array: PoseArray):
        self._poi_poses = poses_array.poses
    
    def _store_inspection_poses(self, poses_array: PoseArray):
        self._inspection_poses = poses_array.poses
    
    def _store_robot_state(self, model_states: ModelStates):
        if len(model_states.name) < 1:
            return
        
        robot_state_index = False
        for i in range(0, len(model_states.name)):
            if model_states.name[i] == 'inspector_robot':
                robot_state_index = i
                break
        if not robot_state_index:
            return
        
        new_robot_state = ModelState(
            model_states.name[robot_state_index],
            model_states.pose[robot_state_index],
            model_states.twist[robot_state_index],
            'map'
        )

        if self._robot_state == None:
            print('Storing Inspector Robot state.')
            self._robot_state = new_robot_state

    def _store_image_message(self, image_message):
        self._image_message = image_message
    
    def _save_image(self, filename_suffix, index):
        valve_highlighter = create_object('sphere')
        valve_highlighter.radius = 0.1
        
        valve_highlighter.visual.enable_property('transparency')
        valve_highlighter.visual.set_xkcd_color()       

        pose=[self._poi_poses[index].position.x, self._poi_poses[index].position.y, self._poi_poses[index].position.z]
        self._worldGenerator.spawn_model(model=valve_highlighter, robot_namespace='sphere_{}'.format(index), pos=pose)
        img = CvBridge().imgmsg_to_cv2(self._image_message, desired_encoding='passthrough')
        cv.imwrite('/home/catkin_ws/src/inspector/output/inspection' + str(filename_suffix) + '.jpg', img)


    def has_poses(self):
        if self._poi_poses != None and self._inspection_poses != None:
            return True
        return False
    
    def has_robot_state(self):
        if self._robot_state != None:
            return True
        return False

    def inspect(self):
        print('Inspecting...')

        for i in range(0, len(self._inspection_poses)):
            self._inspect_one_poi(i)
    
    def _inspect_one_poi(self, index):
        print('Inspecting POI with index', index, '...')
        inspection_pose = self._inspection_poses[index]
        new_robot_state = ModelState(
            self._robot_state.model_name,
            inspection_pose,
            self._robot_state.twist,
            'map'
        )
        pub = rospy.Publisher('gazebo/set_model_state', ModelState, queue_size=10)
        print('Moving Inspector Robot to new pose:')
        print(new_robot_state)

        i = 0
        while i <= 3:
            pub.publish(new_robot_state)
            sleep(1)
            i += 1

        # Save inspection image
        suffix = '-' + str(time()) + '-' + str(index)
        self._save_image(suffix, index)

def get_world_generator():
    gazebo_proxy = GazeboProxy(ros_host='ros_noetic',
                                 ros_port=11311,
                                 gazebo_host='gazebo',
                                 gazebo_port=11345,
                                 timeout=30,
                                 ignore_services=None)
    return WorldGenerator(gazebo_proxy=gazebo_proxy)

def test_create_something_anything(world_generator):
    valve_highlighter = create_object('sphere')
    valve_highlighter.radius = 0.1
    valve_highlighter.visual.enable_property('material')
    valve_highlighter.visual.set_xkcd_color()

    world_generator.spawn_model(model=valve_highlighter, robot_namespace='sphere_1', pos=[1,1,1])

if __name__ == '__main__':
    rospy.init_node('inspector', anonymous=True)
    try:
        add_robot_model()                
        inspector = Inspector(get_world_generator())
        while not inspector.has_poses() or not inspector.has_robot_state():
            sleep(1)
        inspector.inspect()
        #test_create_something_anything(get_world_generator())        

    except rospy.ROSInterruptException:
        pass