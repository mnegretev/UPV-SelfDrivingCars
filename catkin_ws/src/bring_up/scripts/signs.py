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
    
    spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    spawn_req   = SpawnModelRequest()

    angle = numpy.random.uniform(-0.1, 0.1)
    pos_x = numpy.random.uniform(-2.4,-2.2)

    spawn_req.model_name = "AutoModelMini"
    spawn_req.model_xml  = open(pkg_path + '/models/AutoNOMOS_mini/model.sdf','r').read()
    spawn_req.robot_namespace = ''
    spawn_req.initial_pose.position.x = pos_x
    spawn_req.initial_pose.position.y = -1.15
    spawn_req.initial_pose.position.z = 0.05
    spawn_req.initial_pose.orientation.z = math.sin(angle/2)
    spawn_req.initial_pose.orientation.w = math.cos(angle/2)
    spawn_model(spawn_req)

    spawn_req.model_name = "StopLightPost1"
    spawn_req.model_xml  = open(pkg_path + '/models/stop_light_post/model.sdf','r').read()
    spawn_req.robot_namespace = ''
    spawn_req.initial_pose.position.x = -0.28
    spawn_req.initial_pose.position.y = -1.34
    spawn_req.initial_pose.position.z = 0.00
    spawn_req.initial_pose.orientation.z = math.sin(-1.5708/2)
    spawn_req.initial_pose.orientation.w = math.cos(-1.5708/2)
    spawn_model(spawn_req)

    spawn_req.model_name = "StopSign1"
    spawn_req.model_xml  = open(pkg_path + '/models/stop_sign_small/model.sdf','r').read()
    spawn_req.robot_namespace = ''
    spawn_req.initial_pose.position.x = -1.33
    spawn_req.initial_pose.position.y = -1.43
    spawn_req.initial_pose.position.z = 0.0
    spawn_req.initial_pose.orientation.z = math.sin(-1.5708/2)
    spawn_req.initial_pose.orientation.w = math.cos(-1.5708/2)
    spawn_model(spawn_req)

    spawn_req.model_name = "SpeedLimit1"
    spawn_req.model_xml  = open(pkg_path + '/models/speed_limit_sign_small/model.sdf','r').read()
    spawn_req.robot_namespace = ''
    spawn_req.initial_pose.position.x = -1.33
    spawn_req.initial_pose.position.y = -0.42
    spawn_req.initial_pose.position.z = 0.0
    spawn_req.initial_pose.orientation.z = math.sin(1.5708/2)
    spawn_req.initial_pose.orientation.w = math.cos(1.5708/2)
    spawn_model(spawn_req)

    spawn_req.model_name = "PersonWalking1"
    spawn_req.model_xml  = open(pkg_path + '/models/person_walking/model.sdf','r').read()
    spawn_req.robot_namespace = ''
    spawn_req.initial_pose.position.x = -0.28
    spawn_req.initial_pose.position.y = -0.58
    spawn_req.initial_pose.position.z = 0.0
    spawn_req.initial_pose.orientation.z = math.sin(0)
    spawn_req.initial_pose.orientation.w = math.cos(0)
    spawn_model(spawn_req)
    

    
if __name__ == "__main__":
    main()
