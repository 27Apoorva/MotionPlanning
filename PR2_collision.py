#!/usr/bin/env python
# -*- coding: utf-8 -*-
#HW1 for RBE 595/CS 525 Motion Planning
#code based on the simplemanipulation.py example
import time
import openravepy

if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *

def waitrobot(robot):
    """busy wait for robot completion"""
    while not robot.GetController().IsDone():
        time.sleep(0.5)

def tuckarms(env,robot):
    with env:
        jointnames = ['l_shoulder_lift_joint','l_elbow_flex_joint','l_wrist_flex_joint','r_shoulder_lift_joint','r_elbow_flex_joint','r_wrist_flex_joint']
        robot.SetActiveDOFs([robot.GetJoint(name).GetDOFIndex() for name in jointnames])
        robot.SetActiveDOFValues([1.29023451,-2.32099996,-0.69800004,1.27843491,-2.32100002,-0.69799996]);
        robot.GetController().SetDesired(robot.GetDOFValues());
    waitrobot(robot)


# r_shoulder_pan_joint')
#  ('r_upper_arm_roll_joint'), 
#  RaveGetEnvironment(1).GetKinBody('PR2').GetJoint( 
#  RaveGetEnvironment(1).GetKinBody('PR2').GetJoint('r_forearm_roll_joint'),
#   RaveGetEnvironment(1).GetKinBody('PR2').GetJoin
#    RaveGetEnvironment(1).GetKinBody('PR2').GetJoint('r_wrist_roll_joint'), 

if __name__ == "__main__":

    env = Environment()
    env.SetViewer('qtcoin')
    env.Reset()
    # load a scene from ProjectRoom environment XML file
    env.Load('data/pr2test2.env.xml')
    time.sleep(0.1)

    # 1) get the 1st robot that is inside the loaded scene
    # 2) assign it to the variable named 'robot'
    robot = env.GetRobots()[0]
    wall=env.GetBodies()[0] #wall would be the platform

    # tuck in the PR2's arms for driving
    tuckarms(env,robot);

    #### YOUR CODE HERE ####
    Tz=matrixFromPose([1,0,0,0,3.6,1.41,0]) #move robot near wall
    robot.SetTransform(numpy.dot(Tz,robot.GetTransform())) 

    with env:
        
    
        c=env.CheckCollision(robot,wall) #check if collision is occuring with wall
        print 'Collision before opening the arm near the wall:',c
        jointnames=['r_shoulder_pan_joint','r_upper_arm_roll_joint','r_forearm_roll_joint','r_wrist_roll_joint','r_shoulder_lift_joint','r_elbow_flex_joint','r_wrist_flex_joint']
        robot.SetActiveDOFs([robot.GetJoint(name).GetDOFIndex() for name in jointnames])
        robot.SetActiveDOFValues([0,0,0,0,0,0,0]) #get right arm joints of robot and open it
        robot.GetController().SetDesired(robot.GetDOFValues()); #move robot from initial position to position near wall
        c=env.CheckCollision(robot,wall) #now check collision with wall again
        print 'Collision after opening right arm:',c

        
    waitrobot(robot)


    

    #### END OF YOUR CODE ###


    raw_input("Press enter to exit...")

