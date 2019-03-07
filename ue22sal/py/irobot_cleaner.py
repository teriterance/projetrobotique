import rob1a_v01 as rob1a  #  robot simulator
import robot_control  # robot control functions 

if __name__ == "__main__":
    rb = rob1a.Rob1A()   # create a robot (instance of Rob1A class)
    ctrl = robot_control.RobotControl() # create a robot controller
    rb.log_file_on()   # start log

    ctrl.testInfiniteObstacle (rb)
    
    # safe end : stop the robot, then stop the simulator
    rb.stop()
    rb.log_file_off() # end log
    rb.full_end()
