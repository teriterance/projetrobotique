import rob1a_v01 as rob1a  # get robot simulator
import robot_control  # get robot control functions 
import sonar_filter # low pass filters
import numpy as np
import time

# the best approach is to place all your functions in robot_control.py
# however, if you need additionnal functions, you have to put them here
# ...
#def suivreMur(rb, ctrl, mur, vitnom, pointpassage):
    

if __name__ == "__main__":
    rb = rob1a.Rob1A()   # create a robot (instance of Rob1A class)
    ctrl= robot_control.RobotControl() # create a robot controller

    # put here your code to solve the challenge
    # ..
    
    #on fait que le robot avance de 0.5metre 
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
    #on calcl tetha l'angle de redressement
#    while d < 1.5:
#        r = rb.get_sonar("left") 
#        r = r -(0.045 + 0.10081346333 + 0.5)
#        ang = np.arctan(abs(r)/1.5)
#        ang = ang*180/np.pi
#        ang = np.floor(ang)
#        print(ang)
#        dist_parcourir = np.sqrt(1.5*1.5 +r*r)
#        print(dist_parcourir)
#        #on le fait aller tout droit
#        if r<0:
#            ctrl.inPlaceTurnRight(rb,ang)
#        else:
#            ctrl.inPlaceTurnLeft(rb,ang)
#        print(dist_parcourir)
#        ctrl.goLineOdometer(rb,dist_parcourir,60)
#        d = d + dist_parcourir
    time.sleep(0.5)
    pointPassage = [0.5,1.5]
    vitNom = 80
    lastError = 0
    derivOk = False
    derivError = 0
    deltaSpeed = 0
    kp = 1
    kd = 1
    distWallFront = 1
    i = 0
    while distWallFront > 0.6 :
        t0 = time.time()
        distWallLeft = rb.get_sonar("left") -(0.045 + 0.10081346333)
        controlError = -distWallLeft + pointPassage[0]
        if derivOk:
            derivError = controlError - lastError
            deltaSpeed = kp*controlError + kd*derivError
        else:
            deltaSpeed = kp*controlError
        rb.set_speed(vitNom+deltaSpeed,vitNom - deltaSpeed)
        lasError = controlError
        derivOk = True
        t1 = time.time()
        if i>8:
            distWallFront = rb.get_sonar("front") -(0.03 + 0.10081346333)
        i = i+1
        print(distWallFront)
        time.sleep(abs(0.5-(t1-t0)))
    #on le fait tourner de 90 degre a droite
    
    time.sleep(1)
    ctrl.inPlaceTurnRight(rb,120)
    
    time.sleep(0.5)
    pointPassage = [0.5,1.5]
    vitNom = 90
    lastError = 0
    derivOk = False
    derivError = 0
    deltaSpeed = 0
    kp = 1
    kd = 4
    distWallFront = 1
    i = 0
    while distWallFront > 0.5 :
        t0 = time.time()
        distWallLeft = rb.get_sonar("left") -(0.045 + 0.10081346333)
        print(distWallLeft)
        controlError = -distWallLeft + pointPassage[0]
        if derivOk:
            derivError = controlError - lastError
            deltaSpeed = kp*controlError + kd*derivError
        else:
            deltaSpeed = kp*controlError
        rb.set_speed(vitNom+deltaSpeed,vitNom - deltaSpeed)
        lasError = controlError
        derivOk = True
        t1 = time.time()
        if i>39:
            distWallFront = rb.get_sonar("front") -(0.03 + 0.10081346333)
        i = i+1
        print(distWallFront)
        print("keeevsvsdv :")
        print(controlError)
        time.sleep(abs(0.5-(t1-t0)))
    # safe end : stop the robot, then stop the simulator
    rb.stop()
    rb.log_file_off() # end log
    rb.full_end()
