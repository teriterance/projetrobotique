import rob1a_v01 as rob1a
import numpy as np
import matplotlib.pyplot as plt
import time
import sonar_filter

if __name__ == "__main__":
    rb = rob1a.Rob1A()
    flt = sonar_filter.SonarFilter()
    rb.log_file_on()

    # if you want to change the parameters of the median filter
    # uncomment and modify the following line
    # flt.set_median_size(SZ)  # set filter size (odd number) 
    
    # get raw data
    n = 100
    tt = np.zeros(n)  # time axis
    ts = np.zeros(n)  # raw data
    tf = np.zeros(n)  # filtered data
    loop_time = 0.100 # 100 ms (or 10 Hz)
    tStart = time.time()
    for i in range(n):
        t0 = time.time()
        tt[i] = time.time() - tStart
        ts[i] = rb.get_sonar("front")

        # apply median filter
        tf[i] = flt.ma_filter(ts[i])

        t1 = time.time()
        dt = loop_time - (t1-t0)
        if dt>0:
            time.sleep(dt)
        else:
            print ("overtime !!")
        
    rb.stop()
    rb.log_file_off()
    rb.full_end()

    plt.plot (tt,ts,"*b",label="raw input")
    plt.plot (tt,tf,'m',label="filtered output")
    plt.legend(loc='lower right')
    plt.show()
