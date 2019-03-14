import rob1a_v01 as rob1a  # get robot simulator
import robot_control  # get robot control functions 
import sonar_filter # low pass filters
import numpy as np
import time

# the best approach is to place all your functions in robot_control.py
# however, if you need additionnal functions, you have to put them here
# ...
def suivreMurGauche(rb, flt, mur, vitNom, pointPassage, kp, kd):# declaration des vriables 
    #initialisation des donnee utilile pour le PD
    print(mur)
    lastError = 0
    derivOk = False
    derivError = 0
    deltaSpeed = 0
    distWallFront = 1
    #fin initialisation
    
    #debut de conversion de distance pour test
    odoLeft, odoRight = rb.get_odometers()
    dismoyInit = (odoLeft + odoRight)*0.5
    dismoy = dismoyInit
    distance_a_faire = pointPassage[1]*360/(np.pi*0.03)+0.05
    #fin de convertion pour test d'arret
    
    #initialisation du filtre 
    for i in range(10):
        t0 = time.time()
        ts = rb.get_sonar(mur) -(0.045 + 0.20284722507)
        flt.median_filter(ts)
        t1 = time.time()
        time.sleep(abs(0.1-(t1-t0)))
    #fin initialisation du filtre
    
    #realisation du PD proprement dit 
    while (distWallFront > 0.5 or distWallFront < 0.2) and abs(dismoy-dismoyInit) < distance_a_faire:#condition d'arret
        t0 = time.time()
        #dd
        odoLeft, odoRight = rb.get_odometers()
        dismoy = (odoLeft + odoRight)*0.5
        #filtrage
        distWallLeft = flt.median_filter(rb.get_sonar(mur) -(0.045 + 0.20081346333))
        
        #fin filtrage
        controlError = -distWallLeft + pointPassage[0]
        if derivOk:
            derivError = controlError - lastError
            deltaSpeed = kp*controlError + kd*derivError
        else:
            deltaSpeed = kp*controlError
        
        #on introduit une valeur limite de la viesse 
        deltaSpeedMax = vitNom/8
        if deltaSpeed >deltaSpeedMax:
            deltaSpeed = deltaSpeedMax
        elif deltaSpeed < -deltaSpeedMax:
            deltaSpeed = -deltaSpeedMax
        
        rb.set_speed(vitNom+deltaSpeed,vitNom - deltaSpeed)
        lastError = controlError
        derivOk = True
        t1 = time.time()
        if i>8:
            distWallFront = rb.get_sonar("front") -(0.03 + 0.10081346333)
        i = i+1
        print(distWallLeft)
        #traitement pour l'arret
        
        time.sleep(abs(0.5-(t1-t0)))
    
def suivreMurDroit(rb, flt, mur, vitNom, pointPassage, kp, kd):# declaration des vriables 
    #initialisation des donnee utilile pour le PD
    print(mur)
    lastError = 0
    derivOk = False
    derivError = 0
    deltaSpeed = 0
    distWallFront = 1
    #fin initialisation
    
    #debut de conversion de distance pour test
    odoLeft, odoRight = rb.get_odometers()
    dismoyInit = (odoLeft + odoRight)*0.5
    dismoy = dismoyInit
    distance_a_faire = pointPassage[1]*360/(np.pi*0.03)+0.05
    #fin de convertion pour test d'arret
    
    #initialisation du filtre 
    for i in range(10):
        t0 = time.time()
        ts = rb.get_sonar(mur) -(0.045 + 0.20284722507)
        flt.median_filter(ts)
        t1 = time.time()
        time.sleep(abs(0.1-(t1-t0)))
    #fin initialisation du filtre
    
    #realisation du PD proprement dit 
    while (distWallFront > 0.5 or distWallFront < 0.2) and abs(dismoy-dismoyInit) < distance_a_faire:#condition d'arret
        t0 = time.time()
        #dd
        odoLeft, odoRight = rb.get_odometers()
        dismoy = (odoLeft + odoRight)*0.5
        #filtrage
        distWallLeft = flt.median_filter(rb.get_sonar(mur) -(0.045 + 0.20081346333))
        
        #fin filtrage
        controlError = -distWallLeft + pointPassage[0]
        if derivOk:
            derivError = controlError - lastError
            deltaSpeed = kp*controlError + kd*derivError
        else:
            deltaSpeed = kp*controlError
        
        #on introduit une valeur limite de la viesse 
        deltaSpeedMax = vitNom/8
        if deltaSpeed >deltaSpeedMax:
            deltaSpeed = deltaSpeedMax
        elif deltaSpeed < -deltaSpeedMax:
            deltaSpeed = -deltaSpeedMax
        
        rb.set_speed(vitNom-deltaSpeed,vitNom + deltaSpeed)
        lastError = controlError
        derivOk = True
        t1 = time.time()
        if i>8:
            distWallFront = rb.get_sonar("front") -(0.03 + 0.10081346333)
        i = i+1
        print(distWallLeft)
        #traitement pour l'arret
        
        time.sleep(abs(0.5-(t1-t0)))

if __name__ == "__main__":
    rb = rob1a.Rob1A()   # create a robot (instance of Rob1A class)
    ctrl= robot_control.RobotControl() # create a robot controller
    flt = sonar_filter.SonarFilter() #create filter
    
    # put here your code to solve the challenge
    # ..
    
#    on fait que le robot avance de 0.5 metre 
    ctrl.goLineOdometer(rb,0.5,70) #il va a 80 pour cent de sa vitesse max
    time.sleep(0.5)
    #on le fait tourner de 90degres a droite
    ctrl.inPlaceTurnRight(rb,90)
    time.sleep(0.5)
    #on le fait avancer de 0.5metres a 80%
    ctrl.goLineOdometer(rb,0.5,70)
    time.sleep(0.5)
    #on le fait tournee sur la gauche
    ctrl.inPlaceTurnLeft(rb,90)
    time.sleep(0.5)
    #on le fait tourner avancer de 0.5m
    ctrl.goLineOdometer(rb,1.5,70)
    time.sleep(0.5)
    #on le fait tourner de 90 degre a droite
    ctrl.inPlaceTurnRight(rb,90)
    time.sleep(0.5)
    #la fonction suivre un mur
    suivreMurGauche(rb, flt, "left", 70, [0.5,1.4], 1.25, 250)  
    time.sleep(0.5)
    ctrl.inPlaceTurnRight(rb,100)
    time.sleep(0.5)
    suivreMurGauche(rb, flt, "left", 70, [0.5,4], 1.3, 250)
    time.sleep(0.5)
    ctrl.inPlaceTurnRight(rb,110)
    time.sleep(0.5)
    suivreMurGauche(rb, flt, "left", 40, [0.6,0.5], 2, 250)
    time.sleep(0.5)
    suivreMurDroit(rb, flt, "right", 40, [0.45,2.8], 10, 500)
    time.sleep(0.5)
    suivreMurGauche(rb, flt, "left", 40, [0.6,1.7], 10, 250)
    time.sleep(0.5)
    ctrl.inPlaceTurnRight(rb,70)
    time.sleep(0.5)
    suivreMurGauche(rb, flt, "left", 40, [0.6,0.5], 10, 250)
    time.sleep(0.5)
    suivreMurDroit(rb, flt, "right", 40, [0.45,3], 10, 500)
    time.sleep(0.5)
    suivreMurGauche(rb, flt, "left", 20, [0.5,0.5], 16, 250)
#    time.sleep(0.5)
#    suivreMur(rb, flt, "right", 60, [0.5,1.5])
#    ctrl.goLineOdometer(rb,0.1,60)
    #on le fait tourner de 90 degre a droite
    
    # safe end : stop the robot, then stop the simulator
    rb.stop()
    rb.log_file_off() # end log
rb.full_end()