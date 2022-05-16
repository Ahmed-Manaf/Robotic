import pypot.dynamixel
import time
import math
from utils import *
from onshape_to_robot.simulation import Simulation
import time
# from kinematics import *
import math
import sys
from signal import signal, SIGINT
import traceback
import os
import kinematics
#We change the robot type

# import display
def set_leg_angles(alphas, leg_id, robot,params):
    #convert to degres
    # if leg_id!=1:
    #     return
    robot.legs[leg_id][0].goal_position = alphas[0] * 180 / math.pi
    robot.legs[leg_id][1].goal_position = alphas[1] * 180 / math.pi
    robot.legs[leg_id][2].goal_position = alphas[2] * 180 / math.pi
    
def setPositionToRobot(x,y,z,robot,params):
    # Use your own IK function
    for leg_id in range(1, 7):
        if(leg_id == 1 or leg_id == 3 or leg_id == 5):
            t = (2 * math.pi * time.time() - math.pi/2.0)%(2.0*math.pi)
            # print(time.time())
            if(t <= math.pi):
                alphas = kinematics.computeIKOriented(
                    0.02 * math.cos(t),
                    0.0,
                    0.03 * math.sin(t),
                    0.0,
                    leg_id,
                    params,
                    verbose=True,
                )
            else:
                alphas = kinematics.computeIKOriented(
                    (-0.02 + 0.04 * (t - math.pi)/math.pi),
                    0.0,
                    0.0,
                    0.0,
                    leg_id,
                    params,
                    verbose=True,
                )
        else:
            t = (2 * math.pi * time.time() + math.pi/2.0)%(2.0*math.pi)
            if(t <= math.pi):
                alphas = kinematics.computeIKOriented(
                    0.02 * math.cos(t),
                    0.0,
                    0.03 * math.sin(t),
                    0.0,
                    leg_id,
                    
                    params,
                    verbose=True,
                )
            else:
                alphas = kinematics.computeIKOriented(
                    (-0.02 + 0.04 * (t - math.pi)/math.pi),
                    0.0,
                    0.0,
                    0.0,
                    leg_id,
                    params,
                    verbose=True,
                )
        set_leg_angles(alphas, leg_id, robot,params)
    #state = sim.setJoints(targets)


def main():
    ports = pypot.dynamixel.get_available_ports()
    print("Available ports:", ports)
    dxl_io = pypot.dynamixel.DxlIO(ports[0], baudrate=1000000)
    # robotPath = "phantomx_description/urdf/phantomx.urdf"
    # sim = Simulation(robotPath, gui=True, panels=True, useUrdfInertia=False)
    robot = SimpleRobot(dxl_io)
    robot.init()
    time.sleep(0.1)
    robot.enable_torque()
    # Defining the shutdown function here so it has visibility over the robot variable
    def shutdown(signal_received, frame):
        # Handle any cleanup here
        print("SIGINT or CTRL-C detected. Setting motors to compliant and exiting")
        robot.disable_torque()
        print("Done ticking. Exiting.")
        #sys.exit()
        os._exit(1)

    # Tell Python to run the shutdown() function when SIGINT is recieved
    signal(SIGINT, shutdown)
    try:

        params = Parameters(
            freq=50,
            speed=1,
            z=-60,
            travelDistancePerStep=80,
            lateralDistance=90,
            frontDistance=87,
            frontStart=32,
            method="minJerk",
        )

        print("Setting initial position")
        for k, v in robot.legs.items():
            # Setting 0 s the goal position for each motor (such sadness! When we could be using the glorious inverse kinematics!)
            v[0].goal_position = 0
            v[1].goal_position = 0
            v[2].goal_position = 0

        robot.smooth_tick_read_and_write(3, verbose=False)
        setPositionToRobot(0, 0, 0, robot, params)
        robot.smooth_tick_read_and_write(3, verbose=False)
        time.sleep(2.0)
        while(True):
            print("Looping")
            setPositionToRobot(0, 0, 0, robot, params)
            robot.tick_read_and_write(True)#verbose = True
            time.sleep(0.02)
        print("Init position reached")
        time.sleep(2)
        print("Closing")
    except Exception as e:
        track = traceback.format_exc()
        print(track)
    finally:
        shutdown(None, None)


print("A new day dawns")
main()
print("Done !")
