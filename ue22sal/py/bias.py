import rob1a_v01 as rob1a
import numpy as np
import matplotlib.pyplot as plt
import time

if __name__ == "__main__":
    rb = rob1a.Rob1A()
    rb.log_file_on()

    # loop to get 100 measurements of front sonar
    n = 100
    ts = np.zeros(n)
    loop_time = 0.100 # 100 ms (or 10 Hz)
    for i in range(n):
        t0 = time.time()
        # write here the code to put the distance measured
        # by the front sonar in ts[i]
        # ...
        ts[i] = rb.get_sonar("front")
        print (i,ts[i]) # show the measured distance
        t1 = time.time()
        dt = loop_time - (t1-t0)
        if dt>0:
            time.sleep(dt)
        else:
            print ("overtime !!")
        
    rb.stop()
    rb.log_file_off()
    rb.full_end()

    # compute the mean value of the 100 measurements
    # ...
    meann = 0.01*sum(ts)
    # define theoretical distance to the front obstacle
    # ...
    dis  =  1-0.03-0.1
    # compute the bias
    # ...
    biais = abs(meann - dis)
    print(biais)
    
    plt.plot(range(n), ts)
    plt.plot(range(n), np.ones(n)*dis)
    plt.plot(range(n), np.ones(n)*meann)