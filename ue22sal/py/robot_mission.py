import rob1a_v01 as rob1a  # get robot simulator
import robot_control  # get robot control functions 
import sonar_filter # low pass filters
import numpy as np
import time

# the best approach is to place all your functions in robot_control.py
# however, if you need additionnal functions, you have to put them here
# ...
def suivreMur(rb, flt, mur, vitNom, pointPassage):
    lastError = 0
    derivOk = False
    derivError = 0
    deltaSpeed = 0
    kp = 1.25
    kd = 250
    distWallFront = 1
    ts = np.zeros(10)
    for i in range(10):
        t0 = time.time()
        ts[i] = rb.get_sonar(mur) -(0.045 + 0.20284722507)
        flt.ma_filter(ts[i])
        t1 = time.time()
        time.sleep(abs(0.1-(t1-t0)))
    i = 0
    while distWallFront > 0.5 or distWallFront < 0.2 :
        t0 = time.time()
        distWallLeft = flt.ma_filter(rb.get_sonar(mur) -(0.045 + 0.20081346333))
        controlError = -distWallLeft + pointPassage[0]
        if derivOk:
            derivError = controlError - lastError
            deltaSpeed = kp*controlError + kd*derivError
        else:
            deltaSpeed = kp*controlError
        rb.set_speed(vitNom+deltaSpeed,vitNom - deltaSpeed)
        lastError = controlError
        derivOk = True
        t1 = time.time()
        if i>8:
            distWallFront = rb.get_sonar("front") -(0.03 + 0.10081346333)
        i = i+1
        print(distWallLeft)
        time.sleep(abs(0.5-(t1-t0)))

if __name__ == "__main__":
    rb = rob1a.Rob1A()   # create a robot (instance of Rob1A class)
    ctrl= robot_control.RobotControl() # create a robot controller
    flt = sonar_filter.SonarFilter() #create filter
    
    # put here your code to solve the challenge
    # ..
    
    #on fait que le robot avance de 0.5 metre 
    ctrl.goLineOdometer(rb,0.5,60) #il va a 80 pour cent de sa vitesse max
    time.sleep(0.5)
    #on le fait tourner de 90degres a droite
    ctrl.inPlaceTurnRight(rb,90)
    time.sleep(0.5)
    #on le fait avancer de 0.5metres a 80%
    ctrl.goLineOdometer(rb,0.5,60)
    time.sleep(0.5)
    #on le fait tournee sur la gauche
    ctrl.inPlaceTurnLeft(rb,90)
    time.sleep(0.5)
    #on le fait tourner avancer de 0.5m
    ctrl.goLineOdometer(rb,1.5,60)
    time.sleep(0.5)
    #on le fait tourner de 90 degre a droite
    ctrl.inPlaceTurnRight(rb,90)
    time.sleep(0.5)
    #la fonction suivre un mur
    suivreMur(rb, flt, "left", 60, [0.5,1.5])  
    time.sleep(0.5)
    ctrl.inPlaceTurnRight(rb,100)
    time.sleep(0.5)
    suivreMur(rb, flt, "left", 60, [0.5,1.5])
    time.sleep(0.5)
    ctrl.goLineOdometer(rb,0.1,60)
    ctrl.inPlaceTurnRight(rb,90)
    time.sleep(0.5)
    suivreMur(rb, flt, "left", 60, [0.5,1.5])
    ctrl.goLineOdometer(rb,0.1,60)
    #on le fait tourner de 90 degre a droite
    
    # safe end : stop the robot, then stop the simulator
    rb.stop()
    rb.log_file_off() # end log
    rb.full_end()
