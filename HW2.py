#!/usr/bin/env python
# -*- coding: utf-8 -*-
#HW1 for RBE 595/CS 525 Motion Planning
#code based on the simplemanipulation.py example
import time
import openravepy

#### YOUR IMPORTS GO HERE ####
from Astar import *
handles=[]
#### END OF YOUR IMPORTS ####

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


    with env:
        # the active DOF are translation in X and Y and rotation about the Z axis of the base of the robot.
        robot.SetActiveDOFs([],DOFAffine.X|DOFAffine.Y|DOFAffine.RotationAxis,[0,0,1])

        goalconfig = [2.6,-1.3,-pi/2]

        #### YOUR CODE HERE ####

        # start point is loaded

        start=[-3.4,-1.4,0]
        # goal point is marked for reference

        handles.append(env.plot3(points=n.array(((goalconfig[0],goalconfig[1],0.5))),pointsize=0.05,colors=n.array(((1,1,1))),drawstyle=1))

        # t can be 1 or 2, co can be one or 2. t is for manhattan or Euclidean toggle and co is for 4 connected or 8 connected toggle
        t=2
        co=2
        # goal node is returned
        goal=Astar(env,robot,start,goalconfig,t,co)
        # trajectory is returned
        traj_path=path(env,robot,goal,start)
        # trajectory is traversed
        traj = RaveCreateTrajectory(env, '')
        config = robot.GetActiveConfigurationSpecification('linear')
        config.AddDeltaTimeGroup()

        traj.Init(config)

        finalPath = list()

        for i, point in enumerate(traj_path):
            node = traj_path[i]
            finalPath.append([node[0], node[1], node[2], i*0.003])

        for i, p in enumerate(finalPath):
           traj.Insert(i, p, config, True)

        robot.GetController().SetPath(traj)


        #### Implement the A* algorithm to compute a path for the robot's base starting from the current configuration of the robot and ending at goalconfig. The robot's base DOF have already been set as active. It may be easier to implement this as a function in a separate file and call it here.



        #### Draw your path in the openrave here (see /usr/lib/python2.7/dist-packages/openravepy/_openravepy_0_8/examples/tutorial_plotting.py for examples)

        #### Draw the X and Y components of the configurations explored by A*


        #### Now that you have computed a path, execute it on the robot using the controller. You will need to convert it into an openrave trajectory. You can set any reasonable timing for the configurations in the path. Then, execute the trajectory using robot.GetController().SetPath(mypath);

        #### END OF YOUR CODE ###
    waitrobot(robot)

    raw_input("Press enter to exit...")
