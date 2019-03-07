import rob1a_v01 as rob1a  # get robot simulator
import robot_control  # get robot control functions 
import numpy as np
import time


if __name__ == "__main__":
    rb = rob1a.Rob1A()   # create a robot (instance of Rob1A class)
    ctrl = robot_control.RobotControl() # create a robot controller

    speed = 60  
    dist = 0.5
    ctrl.goLineOdometer(rb,dist,speed)
    dist = -0.5
    ctrl.goLineOdometer(rb,dist,speed)

    angle = 90.0
    ctrl.inPlaceTurnRight(rb,angle)
    ctrl.inPlaceTurnLeft(rb,angle)


    # safe end : stop the robot, then stop the simulator
    rb.stop()
    rb.log_file_off() # end log
    rb.full_end()
