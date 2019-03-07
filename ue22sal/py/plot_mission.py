import numpy as np
import matplotlib.pyplot as plt
import sys

# to cope with french coma instead of decimal point
def cvt_float(st):
    try:
        v = float(st)
    except:
        v = float(st.replace(',', '.'))
    return v

vx=[]
vy=[]
vtht=[]
vt=[]

vsonf=[]
tsonf=[]
vsonl=[]
tsonl=[]

try:
    flog = open (sys.argv[1],"r")
except:
    flog = open ("../rob1a.log","r")
st = flog.readline()
while True:
    st = flog.readline()
    if len(st) == 0:
        break
    v = st[0:-1].split(";")
    timsim=cvt_float(v[0])
    tim=cvt_float(v[1])
    logid=v[2]
    if logid == "robot pose":
        vx.append(cvt_float(v[3]))
        vy.append(cvt_float(v[4]))
        vt.append(timsim)
        vtht.append(cvt_float(v[5]))
    elif logid == "sonar front":
        tsonf.append(timsim)
        vsonf.append(cvt_float(v[3]))
    elif logid == "sonar left":
        tsonl.append(timsim)
        vsonl.append(cvt_float(v[3]))
        
flog.close()

print (len(vx)," track points")

vx = np.asarray(vx)
vy = np.asarray(vy)
vtht = np.asarray(vtht)
vt =  np.asarray(vt)

xwp = np.array([0.5,2.0,2.0,-2.0,-2.0])
ywp = np.array([0.0,-0.5,-2.0,-2.0,2.0])

plt.figure(1)
plt.xlim([-2.5, 2.5])
plt.ylim([-2.5, 2.5])
plt.plot(vx,vy)
plt.plot(vx,vy,'*')
plt.plot(xwp,ywp,'r+')
plt.xlabel("x (m)")
plt.ylabel("y (m)")
plt.title("robot track")
plt.figure(2)
plt.plot (vt,vtht)
plt.xlabel("t (s)")
plt.ylabel("heading (degrees)")
plt.title("robot heading")
plt.figure(3)
plt.plot (tsonf,vsonf)
plt.xlabel("t (s)")
plt.ylabel("front sonar (m)")
plt.title("front sonar")
plt.figure(4)
plt.plot (tsonl,vsonl)
plt.xlabel("t (s)")
plt.ylabel("left sonar (m)")
plt.title("left sonar")
                     
plt.show()
