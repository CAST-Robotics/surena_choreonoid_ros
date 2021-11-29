#!/usr/bin/env python3

from numpy.lib.function_base import rot90
import pybullet_data
import pybullet

import numpy as np
import cv2

import rospy

from trajectory_planner.srv import JntAngs, Trajectory, GeneralTraj
from bullet_sim.srv import Walk, WalkResponse
import math
import os

class robot_sim:
    def __init__(self, render, robot_vel = 0.7, time = 5.0, real_time = False, freq = 240.0):
        ## rosrun bullet_sim bullet.py
        ## run above command in catkin_work space
        
        self.real_time = real_time
        self.iter = 0
        self.robotVel = robot_vel
        self.simTime = time
        self.freq = freq
        self.render = render

        self.jointLimitsLow_ = np.array([-0.33, -0.35, -0.8, -.05, -0.6016, -0.3, -0.33, -0.35, -0.8, -0.05, -0.6016, -0.3])  #Ankle mechanism is considered -0.4805 0.3937
        self.jointLimitsHigh_ = np.array([0.24,  0.35,  0.35,  1.1, 0.43,   0.3,  0.24,  0.35,  0.35, 1.1, 0.43,  0.3])

        rospy.init_node('surena_sim')
        self.rate = rospy.Rate(self.freq)

        self.phisycsClient = pybullet.connect(pybullet.GUI,options= "--opengl2")
        # self.phisycsClient = pybullet.connect(pybullet.DIRECT)

        pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())

        self.robotID = None
        self.planeID = None

        pass

    def simulationSpin(self):
        rospy.spin()
        pass

    def run(self):
        self.reset()

        rospy.wait_for_service("/general_traj")
        general_motion_handle = rospy.ServiceProxy("/general_traj", GeneralTraj)
        init_com_pos = [0, 0, 0.73]
        init_com_orient = [0, 0, 0]  
        final_com_pos = [0, 0, 0.68]
        final_com_orient = [0, 0, 0]
        init_lankle_pos = [0, 0.115, 0]
        init_lankle_orient = [0, 0, 0]
        final_lankle_pos = [0, 0.115, 0]
        final_lankle_orient = [0, 0, 0]
        init_rankle_pos = [0, -0.115, 0]
        init_rankle_orient = [0, 0, 0]
        final_rankle_pos = [0, -0.115, 0]
        final_rankle_orient = [0, 0, 0]
        init_motion_time = 2.0
        done = general_motion_handle(init_com_pos,init_com_orient,final_com_pos,final_com_orient,
                    init_lankle_pos,init_rankle_orient,final_lankle_pos,final_lankle_orient,
                    init_rankle_pos,init_rankle_orient,final_rankle_pos,final_rankle_orient, init_motion_time, 1 / self.freq)
        
        while (not done):
            print("General Motion Failed, calling again...")
            done = general_motion_handle(init_com_pos,init_lankle_orient,final_com_pos,final_com_orient,
                    init_lankle_pos,init_rankle_orient,final_lankle_pos,final_lankle_orient,
                    init_rankle_pos,init_rankle_orient,final_rankle_pos,final_rankle_orient, init_motion_time, 1 / self.freq)

        alpha = 0.44
        t_ds = 0.1
        t_step = 1.0
        step_length = -0.1
        step_width = 0.0
        CoM_height = 0.68
        step_count = 10
        ankle_height = 0.025
        theta = 0.15
        rospy.wait_for_service("/traj_gen")

        trajectory_handle = rospy.ServiceProxy("/traj_gen", Trajectory)

        done = trajectory_handle(alpha,t_ds,t_step,step_length,step_width,CoM_height,
                                         step_count, ankle_height, 1 / self.freq, theta)
        
        while not done:
            print("Trajectory generation failed, calling again...")
            done = trajectory_handle(alpha,t_ds,t_step,step_length,step_width,CoM_height,
                                         step_count, ankle_height, 1 / self.freq, theta)
        if done:
            print("trajectory has been recieved...")

        size = int((init_motion_time + (step_count + 2) * t_step) * self.freq)
        feasible = True
        pre_vel = np.array([0, 0, 0])
        while ((self.iter <= size - 1) and feasible):
            rospy.wait_for_service("/jnt_angs")
            try:
                joint_state_handle = rospy.ServiceProxy("/jnt_angs", JntAngs)
                left_ft = pybullet.getJointState(self.robotID, 11)[2]
                left_ft = [left_ft[2], left_ft[3], left_ft[4]]
                right_ft = pybullet.getJointState(self.robotID, 5)[2]
                right_ft = [right_ft[2], right_ft[3], right_ft[4]]
                config = [0 for i in range(12)]
                jnt_vel = [0 for i in range(12)]
                for idx in range(12):
                    config[idx] = pybullet.getJointState(self.robotID, idx)[0]
                    jnt_vel[idx] = pybullet.getJointState(self.robotID, idx)[1]
                #vel = pybullet.getLinkState(self.robotID, 12)[6]
                vel = np.array([0, 0, 0])
                acc = (vel - pre_vel) * self.freq
                #gyro = pybullet.getLinkState(self.robotID, 12)[7]
                gyro = np.array([0, 0, 0])

                All = joint_state_handle(self.iter, left_ft, right_ft, config, jnt_vel, acc, gyro).jnt_angs

                leftConfig = All[6:12]
                rightConfig = All[0:6]
                for index in range (6):
                    pybullet.setJointMotorControl2(bodyIndex=self.robotID,
                                            jointIndex=index,
                                            controlMode=pybullet.POSITION_CONTROL,
                                            targetPosition = rightConfig[index], force = 85.0)
                    pybullet.setJointMotorControl2(bodyIndex=self.robotID,
                                            jointIndex=index + 6,
                                            controlMode=pybullet.POSITION_CONTROL,
                                            targetPosition = leftConfig[index], force = 85.0)
                pybullet.stepSimulation()

            except rospy.ServiceException as e:
                print("Jntangls Service call failed: %s"%e)
            
            # disturbance
            '''
            if self.iter > 2 * self.freq and self.iter < 2 * self.freq + 7:
                force = (1000.0, 1000.0, 0.0)
                pybullet.applyExternalForce(objectUniqueId=self.robotID,
                                            linkIndex=-1,
                                            forceObj=force,
                                            posObj=[0.0, 0.0, 0.0],
                                            flags=pybullet.LINK_FRAME)
                '''
            self.iter += 1

    
    def reset(self):
        
        self.iter = 0
        pybullet.resetSimulation()
        self.planeID = pybullet.loadURDF("plane.urdf")
        pybullet.setGravity(0,0,-9.81)
        os.chdir("/home/surena/DynCont/Choreonoid_ROS/src/surena_choreonoid_ros")

        self.robotID = pybullet.loadURDF("/bullet_sim/surena4.urdf",useFixedBase = 0)
        #self.box = pybullet.loadURDF("src/surena_choreonoid_ros/bullet_sim/box.urdf", [0.6,0.115,0],useFixedBase = 1)
        if self.real_time:
            pybullet.setRealTimeSimulation(1)
        else:
            pybullet.setRealTimeSimulation(0)
        
        pybullet.enableJointForceTorqueSensor(self.robotID,5)
        pybullet.enableJointForceTorqueSensor(self.robotID,11)

        if self.render:
            cv2.startWindowThread()
            cv2.namedWindow("Surena Optimization")

        print("simulation restarted")
        pass

    def close():
        cv2.destroyAllWindows()
        pybullet.disconnect()


if __name__ == "__main__":

    robot = robot_sim(time = 7.0, robot_vel = 0.6 / 3.6, render=False)
    robot.run()
    pass
