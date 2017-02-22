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
        time.sleep(0.01)

def tuckarms(env,robot):
    with env:
        jointnames = ['l_shoulder_lift_joint','l_elbow_flex_joint','l_wrist_flex_joint','r_shoulder_lift_joint','r_elbow_flex_joint','r_wrist_flex_joint']
        robot.SetActiveDOFs([robot.GetJoint(name).GetDOFIndex() for name in jointnames])
        robot.SetActiveDOFValues([1.29023451,-2.32099996,-0.69800004,1.27843491,-2.32100002,-0.69799996]);
        robot.GetController().SetDesired(robot.GetDOFValues());
    waitrobot(robot)

def move_left_arm_pr2(env,robot): #function that manipulates degree of freedom of left arm of pr2
    with env:
        jointnames = ['l_shoulder_lift_joint','l_elbow_flex_joint','l_wrist_flex_joint']
        robot.SetActiveDOFs([robot.GetJoint(name).GetDOFIndex() for name in jointnames])
        robot.SetActiveDOFValues([0.29023451,1.32099996,-1.69800004]);
        robot.GetController().SetDesired(robot.GetDOFValues());
    waitrobot(robot)

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

    # tuck in the PR2's arms for driving
    tuckarms(env,robot);

    #### YOUR CODE HERE ####

    env.Load('robots/puma.robot.xml') #Loading puma robot at orgin
    pu=env.GetBodies()[9] 
    Tz=matrixFromPose([1,0,0,0,-2.50287,-1.48834,0]) #moving puma robot in front of pr2
    pu.SetTransform(numpy.dot(Tz,pu.GetTransform())) 
    move_left_arm_pr2(env,robot); #call function to move left arm of pr2
    #If you comment move_left_arn_pr2 function collision will be false
    with env: #check if collision occured
        check=env.CheckCollision(robot,pu)
        print "Result of collision between robots:",check
    #If check is true collision happened otherwise not


    #### END OF YOUR CODE ###


    raw_input("Press enter to exit...")

