#!/usr/bin/env python
import numpy as np
import math
import sys
import os
import time
import argparse
import pybullet as p
from onshape_to_robot.simulation import Simulation
import pygame

import kinematics
from constants import *

# from squaternion import Quaternion
from scipy.spatial.transform import Rotation

pygame.display.set_mode((600,400))
pygame.init()

      
class Parameters:
    def __init__(
        self,
        z=-0.06,
    ):
        self.z = z
        # Angle between the X axis of the leg and the X axis of the robot for each leg
        self.legAngles = LEG_ANGLES
        # Initial leg positions in the coordinates of each leg.
        self.initLeg = []  # INIT_LEG_POSITIONS
        self.initLeg.append([0.170, 0])
        self.initLeg.append([0.170, 0])
        self.initLeg.append([0.170, 0])
        self.initLeg.append([0.170, 0])
        self.initLeg.append([0.170, 0])
        self.initLeg.append([0.170, 0])

        # Motor re-united by joint name for the simulation
        self.legs = {}
        self.legs[1] = ["j_c1_rf", "j_thigh_rf", "j_tibia_rf"]
        self.legs[6] = ["j_c1_rm", "j_thigh_rm", "j_tibia_rm"]
        self.legs[5] = ["j_c1_rr", "j_thigh_rr", "j_tibia_rr"]
        self.legs[2] = ["j_c1_lf", "j_thigh_lf", "j_tibia_lf"]
        self.legs[3] = ["j_c1_lm", "j_thigh_lm", "j_tibia_lm"]
        self.legs[4] = ["j_c1_lr", "j_thigh_lr", "j_tibia_lr"]


def to_pybullet_quaternion(roll, pitch, yaw, degrees=False):
    # q = Quaternion.from_euler(roll, pitch, yaw, degrees=degrees)
    # return [q[1], q[2], q[3], q[0]]

    # Create a rotation object from Euler angles specifying axes of rotation
    rot = Rotation.from_euler("xyz", [roll, pitch, yaw], degrees=degrees)

    # Convert to quaternions and print
    rot_quat = rot.as_quat()
    # print(rot_quat)
    return rot_quat


# Updates the values of the dictionnary targets to set 3 angles to a given leg
def set_leg_angles(alphas, leg_id, targets, params):
    leg = params.legs[leg_id]
    i = -1
    for name in leg:
        i += 1
        targets[name] = alphas[i]


# m_friction
parser = argparse.ArgumentParser()
parser.add_argument("--mode", "-m", type=str, default="direct", help="test")
args = parser.parse_args()
controls = {}
robotPath = "phantomx_description/urdf/phantomx.urdf"
sim = Simulation(robotPath, gui=True, panels=True, useUrdfInertia=False)
# sim.setFloorFrictions(lateral=0, spinning=0, rolling=0)
pos, rpy = sim.getRobotPose()
sim.setRobotPose([0, 0, 0.5], [0, 0, 0, 1])

leg_center_pos = [0.1248, -0.06164, 0.001116 + 0.5]
leg_angle = -math.pi / 4
params = Parameters()

def walk(time, angle, leg_id, params):
    if(leg_id == 1 or leg_id == 3 or leg_id == 5):
        t = (2 * math.pi * time - math.pi/2.0)%(2.0*math.pi)
        if(t <= math.pi):
            return kinematics.computeIKOriented(0.02 * math.cos(t), 0.0, 0.03 * math.sin(t), angle, leg_id, params)
        else:
            return kinematics.computeIKOriented(-0.02 + 0.04 * (t - math.pi)/math.pi, 0.0, 0.0, angle, leg_id, params)
    else:
        t = (2 * math.pi * time + math.pi/2.0)%(2.0*math.pi)
        if(t <= math.pi):
            return kinematics.computeIKOriented(0.02 * math.cos(t), 0.0, 0.03 * math.sin(t), angle, leg_id, params)
        else:
            return kinematics.computeIKOriented(-0.02 + 0.04 * (t - math.pi)/math.pi, 0.0, 0.0, angle, leg_id, params)


def rotate(time, direction_rot, leg_id, params):
    angle = 0
    if (leg_id == 1 or leg_id == 3 or leg_id == 5):
        if(leg_id == 3):
            t = (-direction_rot* 2 * math.pi * time - math.pi/2.0)%(2.0*math.pi)
        else:
            t = (direction_rot*2 * math.pi * time - math.pi/2.0)%(2.0*math.pi)
        if(t <= math.pi):
            return kinematics.computeIKOriented(0.02 * math.cos(t), 0.0, 0.03 * math.sin(t), angle, leg_id, params)
        else:
            return kinematics.computeIKOriented(-0.02 + 0.04 * (t - math.pi)/math.pi, 0.0, 0.0, angle, leg_id, params)
    elif (leg_id == 2 or leg_id == 4 or leg_id == 6):
        if(leg_id == 6):
            t = (direction_rot*2 * math.pi * time + math.pi/2.0)%(2.0*math.pi)
        else:
            t = (-direction_rot*2 * math.pi * time + math.pi/2.0)%(2.0*math.pi)
        if(t <= math.pi):
            return kinematics.computeIKOriented(0.02 * math.cos(t), 0.0, 0.03 * math.sin(t), angle, leg_id, params)
        else:
            return kinematics.computeIKOriented(-0.02 + 0.04 * (t - math.pi)/math.pi, 0.0, 0.0, angle, leg_id, params)

def robot_controller_walk():
    move = True
    angle = 0
    pygame.event.pump()
    keys = pygame.key.get_pressed()
    if keys[pygame.K_LEFT] and keys[pygame.K_DOWN]:
        angle = np.pi/4
    elif keys[pygame.K_RIGHT] and keys[pygame.K_DOWN]:
        angle = 3*np.pi/4
    elif keys[pygame.K_RIGHT] and keys[pygame.K_UP]:
        angle = 5*np.pi/4
    elif keys[pygame.K_LEFT] and keys[pygame.K_UP]:
        angle = 7*np.pi/4
    elif keys[pygame.K_UP]:
        angle = 3*np.pi/2
    elif keys[pygame.K_DOWN]:
        angle = np.pi/2
    elif keys[pygame.K_RIGHT]:
        angle = np.pi
    elif keys[pygame.K_LEFT]:
        angle = 0
    else :
        move = False
    return move, angle

def robot_controller_rot():
    rota = True
    direction_rot = 0
    pygame.event.pump()
    keys = pygame.key.get_pressed()
    if keys[pygame.K_a]:
        direction_rot = -1
    elif keys[pygame.K_e]:
        direction_rot = 1
    else:
        rota = False
    return rota, direction_rot
    
if args.mode == "frozen-direct":
    crosses = []
    for i in range(4):
        crosses.append(p.loadURDF("target2/robot.urdf"))
    for name in sim.getJoints():
        print(name)
        if "c1" in name or "thigh" in name or "tibia" in name:
            controls[name] = p.addUserDebugParameter(name, -math.pi, math.pi, 0)
elif args.mode == "direct":
    for name in sim.getJoints():
        print(name)
        if "c1" in name or "thigh" in name or "tibia" in name:
            controls[name] = p.addUserDebugParameter(name, -math.pi, math.pi, 0)
elif args.mode == "inverse":
    cross = p.loadURDF("target2/robot.urdf")
    # Use your own DK function
    alphas = kinematics.computeDK(0, 0, 0, use_rads=True)
    controls["target_x"] = p.addUserDebugParameter("target_x", -0.4, 0.4, alphas[0])
    controls["target_y"] = p.addUserDebugParameter("target_y", -0.4, 0.4, alphas[1])
    controls["target_z"] = p.addUserDebugParameter("target_z", -0.4, 0.4, alphas[2])


while True:
    targets = {}
    for name in sim.getJoints():
        if "c1" in name or "thigh" in name or "tibia" in name:
            targets[name] = 0
    if args.mode == "frozen-direct":
        for name in controls.keys():
            targets[name] = p.readUserDebugParameter(controls[name])
            #Use your own DK function, the result should be: https://www.youtube.com/watch?v=w3psAbh3AoM
            
            points = kinematics.computeDKDetailed(
                targets["j_c1_rf"],
                targets["j_thigh_rf"],
                targets["j_tibia_rf"],
                use_rads=True,
            )
            i = -1
            T = []
        for pt in points:
            # Drawing each step of the DK calculation
            i += 1
            T.append(kinematics.rotaton_2D(pt[0], pt[1], pt[2], leg_angle))
            T[-1][0] += leg_center_pos[0]
            T[-1][1] += leg_center_pos[1]
            T[-1][2] += leg_center_pos[2]
            #print("Drawing cross {} at {}".format(i, T))
            p.resetBasePositionAndOrientation(
                crosses[i], T[-1], to_pybullet_quaternion(0, 0, leg_angle)
            )

        sim.setRobotPose(
            leg_center_pos,
            to_pybullet_quaternion(0, 0, 0),
        )
        state = sim.setJoints(targets)
    elif args.mode == "direct":
        for name in controls.keys():
            targets[name] = p.readUserDebugParameter(controls[name])
        state = sim.setJoints(targets)
    elif args.mode == "inverse":
        x = p.readUserDebugParameter(controls["target_x"])
        y = p.readUserDebugParameter(controls["target_y"])
        z = p.readUserDebugParameter(controls["target_z"])
        # Use your own IK function
        alphas = kinematics.computeIK(x, y, z, verbose=True, use_rads=True)

        targets["j_c1_rf"] = alphas[0]
        targets["j_thigh_rf"] = alphas[1]
        targets["j_tibia_rf"] = alphas[2]

        state = sim.setJoints(targets)
        # Temp
        sim.setRobotPose([0, 0, 0.5], [0, 0, 0, 1])

        T = kinematics.rotaton_2D(x, y, z, leg_angle)
        T[0] += leg_center_pos[0]
        T[1] += leg_center_pos[1]
        T[2] += leg_center_pos[2]
        # print("Drawing cross {} at {}".format(i, T))
        p.resetBasePositionAndOrientation(
             cross, T, to_pybullet_quaternion(0, 0, leg_angle)
         )
    elif args.mode == "robot-ik":
        None
        # Use your own IK function
        for leg_id in range(1, 7):
            alphas = kinematics.computeIKOriented(
                0.01 * math.sin(2 * math.pi * 0.5 * time.time()),
                0.02 * math.cos(2 * math.pi * 0.5 * time.time()),
                0.03 * math.sin(2 * math.pi * 0.2 * time.time()),
                0.0,
                leg_id,
                params,
                verbose=True,
            )
            set_leg_angles(alphas, leg_id, targets, params)
        state = sim.setJoints(targets)
    elif args.mode == "legs":
        None
        # Use your own IK function
        
        move, angle = robot_controller_walk()
        if (move):
            for leg_id in range(1, 7):
                alphas = walk(time.time(), angle, leg_id, params)
                set_leg_angles(alphas, leg_id, targets, params)
            state = sim.setJoints(targets)
    elif args.mode == "legs-rot":
        None
        # Use your own IK function
        
        rota, direction_rot = 1, 1
        if (rota):
            for leg_id in range(1, 7):
                alphas = rotate(time.time(), direction_rot, leg_id, params)
                set_leg_angles(alphas, leg_id, targets, params)
            state = sim.setJoints(targets)
    elif args.mode == "legs-rot-walk":
        None
        # Use your own IK function
        move, angle = robot_controller_walk()
        rota, direction_rot = robot_controller_rot()

        if (rota and move):
            for leg_id in range(1, 7):            
                alphasw = np.array(walk(time.time(), angle, leg_id, params))
                alphasr = np.array(rotate(time.time(), direction_rot, leg_id, params))
                alphas = (alphasr + alphasw)/2.0
                set_leg_angles(alphas, leg_id, targets, params)
            state = sim.setJoints(targets)
        
        elif(rota):
            for leg_id in range(1, 7):
                alphas = rotate(time.time(), direction_rot, leg_id, params)
                set_leg_angles(alphas, leg_id, targets, params)
            state = sim.setJoints(targets)

        elif(move):
            for leg_id in range(1, 7):
                alphas = walk(time.time(), angle, leg_id, params)
                set_leg_angles(alphas, leg_id, targets, params)
            state = sim.setJoints(targets)

    sim.tick()
