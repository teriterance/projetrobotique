import rob1a_v01 as rob1a  # get robot simulator
import robot_control  # get robot control functions 
import sonar_filter # low pass filters
import numpy as np
import time

# the best approach is to place all your functions in robot_control.py
# however, if you need additionnal functions, you have to put them here
# ...


if __name__ == "__main__":
    rb = rob1a.Rob1A()   # create a robot (instance of Rob1A class)
    ctrl= robot_control.RobotControl() # create a robot controller

    # put here your code to solve the challenge
    # ..
    #ctrl.inPlaceTurnRight(rb,50)
    #ctrl.inPlaceTurnLeft(rb,50)
    ctrl.goLineOdometer(rb, 0.5, 30)
    time.sleep(0.5)
    ctrl.inPlaceTurnRight(rb,90)
    time.sleep(0.5)
    ctrl.goLineOdometer(rb, 0.5, 20)
    time.sleep(0.5)
    ctrl.inPlaceTurnLeft(rb, 90)
    time.sleep(0.5)
    ctrl.goLineOdometer(rb, 1.5, 25)
    time.sleep(0.5)
    ctrl.inPlaceTurnRight(rb,90)
    time.sleep(0.5)
        
    ## on le fait suivre un mur 
    #on recupere sa distance au mur
    print ("sqfjklvgb")
    while rb.get_sonar('front') > 0.3:
        distmur =  rb.get_sonar('left')
        err = 0.05
        if abs(distmur - 0.6) > err and distmur < 0.5:
            #dans ce cas il faut le faire tourner
            ctrl.inPlaceTurnLeft(rb, 30)
            time.sleep(0.5)
        elif abs(distmur - 0.6) > err and distmur >0.5:
            ctrl.inPlaceTurnRight(rb,30)
            time.sleep(0.5)
        ctrl.goLineOdometer(rb, 0.5, 20)
        time.sleep(0.5)

    ctrl.goLineOdometer(rb, 1.5, 20)
    ctrl.inPlaceTurnRight(rb,90)
    ctrl.goLineOdometer(rb, 1.5, 20)
    # safe end : stop the robot, then stop the simulator
    rb.stop()
    rb.log_file_off() # end log
    rb.full_end()
