#!/usr/bin/env python
import rospy
import rospkg
import math
import numpy
from gazebo_msgs.srv import SpawnModel
from gazebo_msgs.srv import SpawnModelRequest

def main():
    rospy.init_node("spawn_person_standing")
    print("Waiting for service /gazebo/spawn_sdf_model")
    rospy.wait_for_service("/gazebo/spawn_sdf_model")
    print("Service is now available")
    rospack  = rospkg.RosPack()
    pkg_path = rospack.get_path('autonomos_gazebo_simulation')
    
    spawn_model    = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    get_link_state = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
    spawn_req      = SpawnModelRequest()
    get_link_req   = GetLinkStateRequest()

    angle = numpy.random.uniform(math.pi-0.3, math.pi+0.3)
    pos_x = numpy.random.uniform(1.2,3.8) 

    spawn_req.model_name = "AutoModelMini"
    spawn_req.model_xml  = open(pkg_path + '/models/AutoNOMOS_mini/model.sdf','r').read()
    spawn_req.robot_namespace = ''
    spawn_req.initial_pose.position.x = pos_x
    spawn_req.initial_pose.position.y = 3.3
    spawn_req.initial_pose.position.z = 0.17
    spawn_req.initial_pose.orientation.z = math.sin(angle/2)
    spawn_req.initial_pose.orientation.w = math.cos(angle/2)
    spawn_model(spawn_req)
    
    spawn_req.model_name = "WalkingPerson1"
    spawn_req.model_xml  = open(pkg_path + '/models/person_walking/model.sdf','r').read()
    spawn_req.robot_namespace = ''
    spawn_req.initial_pose.position.x = pos_x-1.0
    spawn_req.initial_pose.position.y = 3.3
    spawn_req.initial_pose.position.z = 0.17
    spawn_req.initial_pose.orientation.z = math.sin(angle/2)
    spawn_req.initial_pose.orientation.w = math.cos(angle/2)
    spawn_model(spawn_req)

    
if __name__ == "__main__":
    main()
