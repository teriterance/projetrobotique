import threading
import socket
import sys
import struct
import time
import signal
import numpy as np

"""
UE 2.2 sensor actuator loop
Simple robot for 1A , 2 Wheels drive with castor wheel
Actuators : differencial command with two motors
Sensors : 5 ultrasonic telemeters, 2 encoders 
Do not change this program - Ne modifiez pas ce programme
"""

class Rob1A:
    # interrupt handler
    def interrupt_handler(self,signal, frame):
        print ('You pressed Ctrl+C! Rob1A will stop in the next 3 seconds ')
        if self.vrep.isAlive():
            self.set_speed(0.,0.)
            self.full_end()
        sys.exit(0)

    def __init__(self):
        # Connect the socket to the port where the server is listening on
        self.server_address = ('localhost', 30100)

        self.distFront = 0.0
        self.distLeft = 0.0
        self.distRight = 0.0
        self.distBack = 0.0
        self.distFrontLeft = 0.0
        self.distFrontRight = 0.0
        self.encoderLeft = 0
        self.encoderRight = 0
        self.speedLeft = 0.0
        self.speedRight = 0.0

        #self.debug = True # display debug messages on console
        self.debug = False # do not display debug 

        self.log = 0.0 # controls writing in log file
                       #   0.0 : no log
                       #   1.0 : write log

        self.dtmx = -1.0
        self.dtmn = 1e38
        self.cnt_sock = 0
        self.dta_sock = 0.0
        self.upd_sock = False
        self.rob1a_ready = threading.Event()
        self.rob1a_ready.clear()
 
        # initiate communication thread with V-Rep
        self.simulation_alive = True
        srv = self.server_address
        ev = self.rob1a_ready
        self.vrep = threading.Thread(target=self.vrep_com_socket,args=(srv,ev,))
        self.vrep.start()
        # wait for robot to be ready
        self.rob1a_ready.wait()
        print ("Rob1A ready ...")
        # trap hit of ctrl-x to stop robot and exit (emergency stop!)
        signal.signal(signal.SIGINT, self.interrupt_handler)

    def vrep_com_socket(rob1a,srv,ev):
        while True:
            #print ("update vrep with sock")
            #print (rob1a.simulation_alive,rob1a.speedLeft,rob1a.speedRight)
            # Create a TCP/IP socket
            t0 = time.time()
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            try:
                sock.connect(srv)
            except:
                print ("Simulation must be alive to execute your python program properly.")
                print ("Type Ctrl-C to exit, start the simulation and then execute your python program.")
                break

            sock.settimeout(0.5)
            vtx = [rob1a.speedLeft,rob1a.speedRight]
            data = [ord(chr(59)),ord(chr(57)),12,0,rob1a.log,vtx[0],vtx[1]]
            strSend = struct.pack('<BBHHfff',data[0],data[1],data[2],data[3],data[4],data[5],data[6]) 
            sock.sendall(strSend)
            upd_sock = True

            data = b''
            try:
                while len(data) < 38:
                    data += sock.recv(38)
            except:
                print ("socker error , duration is %f ms, try to reconnect !!!"%((time.time() - t0)*1000.0))
                #sock.detach()
                #sock.connect(srv)
                #print ("socker error , type Ctrl-C to exit !!!")
                #exit(0)
            if len(data) == 38:
                vrx = struct.unpack('<ccHHffffffff',data)
                #rob1a.distFront = vrx[4]
                #rob1a.distLeft = vrx[5]
                #rob1a.distRight = vrx[6]
                #rob1a.distBack = vrx[7]
                #rob1a.distFrontLeft = vrx[8]
                #rob1a.distFrontRight = vrx[9]
                #rob1a.encoderLeft = int(vrx[10])
                #rob1a.encoderRight = int(vrx[11])
                rob1a.vrep_update_sim_param(upd_sock,vrx)
                #print vrx
            else:
                print ("bad data length ",len(data))


            sock.close()
            rob1a.cnt_sock = rob1a.cnt_sock + 1
            tsock = (time.time() - t0)*1000.0
            rob1a.dta_sock += tsock
            if tsock > rob1a.dtmx:
                rob1a.dtmx = tsock
            if tsock < rob1a.dtmn:
                rob1a.dtmn = tsock
            dtm = rob1a.dta_sock/float(rob1a.cnt_sock)
            #print ("tsock",tsock)
            if rob1a.debug:
                if (rob1a.cnt_sock % 100) == 99:
                    print ("min,mean,max socket thread duration (ms) : ",rob1a.dtmn,dtm,rob1a.dtmx)

            #time.sleep(0.2)
            #print (dir(ev))
            ev.set()

            if not rob1a.simulation_alive:
                break 


    def vrep_update_sim_param(self,upd_sock,vrx):
        #print (upd_sock)
        self.upd_sock = upd_sock
        self.distFront = vrx[4]
        self.distLeft = vrx[5]
        self.distRight = vrx[6]
        self.distBack = vrx[7]
        #self.distFrontLeft = vrx[8]     # only 4 sonars
        #self.distFrontRight = vrx[9]    # only 4 sonars
        self.encoderLeft = int(vrx[10])
        self.encoderRight = int(vrx[11])
        
    def stop (self):
        """
        Stop the robot by setting the speed of the motors to 0
        """
        self.set_speed(0.,0.)

    def full_end (self):
        """
        Fully stop the simulation of the robot , set motors speed to 0
        and close the connection with the simulator vrep
        sleep for 3 seconds to end cleanly the log file
        """
        self.set_speed(0.,0.)
        time.sleep(3.0)
        print ("clean stop of  Rob1A")
        self.simulation_alive = False

    def set_speed (self,speedLeft,speedRight):
        """
        speedLeft : set speed of left wheel
        speedLeft : set speed of right wheel
        the speed is a value between -100 and +100
        warning the motors have a "dead zone", |speed| must be > 20
        """
        #print ("set speed L,R",speedLeft,speedRight)
        self.speedLeft = float(round(speedLeft))
        self.speedRight = float(round(speedRight))
        self.upd_sock = False
        #print (self.upd_sock)
        while not self.upd_sock:
            time.sleep(0.0001)
        #print (self.upd_sock)

    def get_odometers (self):
        """
        return the values of ticks counter for the two wheels
        """
        return self.encoderLeft,self.encoderRight

    def get_sonar (self,name):
        """
        return the measurment of the selected sonar (ultrasonic sensor)
        if obstacle between 0.1 m to 1.5 m return the distance to 
        the nearest obstacle, otherwise return 0.0
        distance is returned in meters
        name can be "front","frontleft","frontright","left","right" or "back"
        """
        val = 0.0
        if name == "front":
            val = self.distFront
        elif name == "frontleft":
            val = self.distFrontLeft
        elif name == "frontRight":
            val = self.distFrontRight
        elif name == "left":
            val = self.distLeft           
        elif name == "right":
            val = self.distRight
        elif name == "back":
            val = self.distBack
        return val

    def get_multiple_sonars (self,names):
        val = []
        for name in names:
            if name == "front":
                val.append(self.distFront)
            elif name == "frontleft":
                val.append(self.distFrontLeft)
            elif name == "frontRight":
                val.append(self.distFrontRight)
            elif name == "left":
                val.append(self.distLeft)           
            elif name == "right":
                val.append(self.distRight)
            elif name == "back":
                val.append(self.distBack)
            elif name == "all":
                val.append(self.distFront)
                val.append(self.distFrontLeft)
                val.append(self.distFrontRight)
                val.append(self.distLeft)
                val.append(self.distRight)
                val.append(self.distBack)
        return val

    def log_file_on (self):
        self.log = 1.0

    def log_file_off (self):
        self.log = 0.0

        
if __name__ == "__main__":

    rb = Rob1A()
    odol0,odor0 = rb.get_odometers()
    rb.set_speed(50.0,50.0)
    dFront = rb.get_sonar("front")
    dObst = 0.3
    while dFront == 0.0 or dFront > dObst:
        time.sleep(0.0001)
        dFront = rb.get_sonar("front")       
    rb.stop()
    print (rb.get_multiple_sonars(["all"]))
    odol1,odor1 = rb.get_odometers()
    diam = 0.06
    nticks = 300
    distl = (odol1-odol0)/nticks*np.pi*diam
    distr = (odor1-odor0)/nticks*np.pi*diam
    print ("distl,r ",distl,distr)

    rb.turnLeft(90.0)

    v = []
    t0 = time.time()
    for i in range(100):
        vf = rb.get_sonar("front")
        if vf < 1.5:
            print ((time.time()-t0)*1000.,i,vf)
            v.append(vf)
        time.sleep(0.1)
    v = np.asarray(v)
    print (np.mean(v),np.std(v,ddof=1))
    rb.full_end()



