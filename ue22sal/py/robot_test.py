import rob1a_v01 as rob1a  # get robot simulator
import robot_control  # get robot control functions 
import numpy as np
import time


if __name__ == "__main__":
    rb = rob1a.Rob1A()   # create a robot (instance of Rob1A class)
    ctrl = robot_control.RobotControl() # create a robot controller

    rb.log_file_on()   # start log
    speedLeft = 5008
    speedRight = 50
    duration = 6.0
    ctrl.testMove (rb,speedLeft,speedRight,duration)

    speedLeft = -58
    speedRight = -50
    duration = 6.0
    ctrl.testMove (rb,speedLeft,speedRight,duration)

    # safe end : stop the robot, then stop the simulator
    rb.stop()
    rb.log_file_off() # end log
    rb.full_end()
