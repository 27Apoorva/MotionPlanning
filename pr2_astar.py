#!/usr/bin/env python
# -*- coding: utf-8 -*-
#HW1 for RBE 595/CS 525 Motion Planning
#code based on the simplemanipulation.py example
#### YOUR IMPORTS GO HERE ####
import time
import openravepy
if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *
from sympy import *  
from copy import deepcopy 

#### END OF YOUR IMPORTS ####
class node:
    def __init__(self,point_3d):
        self.point=point_3d
        self.x=None
        self.y=None
        self.g=0
        self.h=0
        self.parent=None
    def __eq__(self, other):
        return self.point.x == other.point.x and self.point.y == other.point.y

    def __hash__(self):
        n_hash = hash((self.point.x,self.point.y))
        return n_hash
    # def set_pos(self,data):
    #     self.x=data[0]
    #     self.y=data[1]

class point_3d:
    def __init__(self, array_3d):
        self.x = array_3d[0]
        self.y = array_3d[1]
        self.z = array_3d[2]
    def __repr__(self):
        return "(%s, %s, %s)"%(self.x, self.y, self.z)
    def value(self):
        return (self.x,self.y)

    def __len__(self):
        return 0#len(self.x) + len(self.y) +  len(self.z)
def check_node(a, b):
    if (abs(a.point.x-b.point.x)<0.01)&(abs(a.point.y-b.point.y)<0.01):
        # a.parent = b
        return True
    else:
        return False
def check_collision(neigh_node, env, robot):
    point = [neigh_node.point.x,neigh_node.point.y,neigh_node.point.z]
    robot.SetActiveDOFValues(point)
    return env.CheckCollision(robot)

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
def find_successor(curr_node,env, robot, handles):
    step_size=0.1
    #print curr_node.x,curr_node.y
    diag_size = step_size*1.4
    # Comment out last four step size from moves list to get 4-connected configuration
    moves = [[0,step_size,0],[0,-step_size,0],[step_size,0,0],[-step_size,0,0],[step_size,step_size,0],[-step_size,step_size,0],[step_size,-step_size,0],[-step_size,-step_size,0]]
    neigh_list=set()
    for i in moves:
        neigh_node = node(point_3d([curr_node.point.x + i[0],curr_node.point.y + i[1],curr_node.point.z + i[2]]))
        if(check_collision(neigh_node, env, robot)):
            #print red for non-valid points
            point = [neigh_node.point.x,neigh_node.point.y,neigh_node.point.z]
            handles.append(env.plot3(points=array(point),pointsize=4.0,colors=array(((1,0,0)))))
            
        else:   
            #print blue for valid points
            point = [neigh_node.point.x,neigh_node.point.y,neigh_node.point.z]
            handles.append(env.plot3(points=array(point),pointsize=4.0,colors=array(((0,0,1)))))
            neigh_list.add(neigh_node)
  
    return neigh_list

def man_distance(a,b):
    dist=abs(a.point.x-b.point.x) + abs(a.point.y-b.point.y) + abs(a.point.z-b.point.z)
    return dist*100
def eucli_distance(a,b):
    dist=((a.point.x-b.point.x)**2 + (a.point.y-b.point.y)**2) **0.5
    return dist

def check_closed_set(a, closedSet):
    p = 0
    for b in closedSet:
        if (abs(a.point.x-b.point.x)<0.01)&(abs(a.point.y-b.point.y)<0.01):#&(abs(a.point.z-b.point.z)<0.01):
            p = 1

    return p
def check_pass(a, closedSet, new_cost):
    p = 1
    for point in closedSet:
        if point == a:
            if new_cost < point.g:
                p = 1
            else:
                p = 0
    return p


def a_star(start,goal,env,robot):
    with env:
        start_node=node(point_3d(start))
        goal_node=node(point_3d(goal))
        open_list=set()
        closed_list=set()
        start_node.h =  eucli_distance(start_node, goal_node)
        start_node.g =  0
        current =  start_node
        open_list.add(current)

        step_cost = 0.5
        count = 0
        while len(open_list)>0:
            current = min(open_list, key=lambda k:k.h + k.g)
            if(check_node(current, goal_node)):
                # current.parent = goal_node
                break
            neighbour = set()
            neighbour = find_successor(current, env, robot, handles)
            open_list.remove(current)
            closed_list.add(current)
            for n in neighbour:
                #calculate the cost and get the min of openset
                
                new_cost = current.g + step_cost
                if(check_pass(n, closed_list, new_cost)):
                    n.g = new_cost
                    n.h = eucli_distance(n, goal_node)
                    n.parent = current
                    open_list.add(n)
                    count = count + 1

        
    return current


if __name__ == "__main__":

    env = Environment()
    env.SetViewer('qtcoin')
    env.Reset()
    
    env.Load('data/pr2test2.env.xml')
    time.sleep(0.1)

   
    robot = env.GetRobots()[0]

    # tuck in the PR2's arms for driving
    tuckarms(env,robot);
    
    with env:
        # the active DOF are translation in X and Y and rotation about the Z axis of the base of the robot.
        robot.SetActiveDOFs([],DOFAffine.X|DOFAffine.Y|DOFAffine.RotationAxis,[0,0,1])

        goalconfig = [2.6,-1.3,-pi/2]
        #startconfig = robot.GetActiveDOFValues()
        config =[-3.4, -1.4, .05]
        start=[robot.GetTransform()[0][3],robot.GetTransform()[1][3],robot.GetTransform()[2][3]]
        goal=[2.6,-1.3,0]

        handles=[]
        path = set()

        aStar=a_star(start,goal,env,robot)
        final_path = []
        current = aStar
        current.point = [current.point.x,current.point.y,-pi/2]
        final_path.append(current.point)
        current = current.parent
        prev_point = []
        #### YOUR CODE HERE ####


        #### Implement the A* algorithm to compute a path for the robot's base starting from the current configuration of the robot and ending at goalconfig. The robot's base DOF have already been set as active. It may be easier to implement this as a function in a separate file and call it here.
        while(True):
            #to get the robot move with proper orientation, atan2(y,x) is given as third value, where, x,y:difference between present and last coordinate
            prev = [current.point.x ,current.point.y ]
            current = current.parent
            p_curr = [current.point.x ,current.point.y ] #math.atan2(current.point.y,current.point.x)]
            p_final = [p_curr[0]-prev[0],p_curr[1]-prev[1]]
            point = [current.point.x,current.point.y,math.atan2(p_final[1],p_final[0])]
            final_path.append(point)
            point_12 = [current.point.x,current.point.y,0.05]
            handles.append(env.plot3(points=array(point_12),pointsize=4.0,colors=array(((0,0,0)))))
            if config[0] == current.point.x and config[1] == current.point.y:
                break



        #### Draw your path in the openrave here (see /usr/lib/python2.7/dist-packages/openravepy/_openravepy_0_8/examples/tutorial_plotting.py for examples)

        #### Draw the X and Y components of the configurations explored by A*


        #### Now that you have computed a path, execute it on the robot using the controller. You will need to convert it into an openrave trajectory. You can set any reasonable timing for the configurations in the path. Then, execute the trajectory using robot.GetController().SetPath(mypath);
        traject = RaveCreateTrajectory(env,'')
        traject.Init(robot.GetActiveConfigurationSpecification())     
        for s in final_path:
            traject.Insert(0,(s))         
        planningutils.RetimeActiveDOFTrajectory(traject,robot,hastimestamps=False,maxvelmult=1)
        print 'Duration',traject.GetDuration()
        #print robot.GetActiveDOFValues()
        robot.SetActiveDOFValues(goalconfig)
    robot.GetController().SetPath(traject)
    robot.WaitForController(0)
    waitrobot(robot)

    raw_input("Press enter to exit...")

