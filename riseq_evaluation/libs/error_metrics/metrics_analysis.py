import numpy as np 
import matplotlib.pyplot as plt 

x = np.arange(-10,10,0.001)
y1 = 1./np.sqrt(1+np.abs(x))
y2 = 1./(1+np.abs(x))
y3 = 1./(1 + np.sqrt(np.abs(x)))
fig = plt.figure()
ax1 = fig.add_subplot(1,1,1)
ax1.plot(x,y1,label="1/sqrt(1 + x)")
ax1.plot(x,y2,label="1/(1 + x)")
ax1.plot(x,y3,label="1/(1 + sqrt(x))")
ax1.legend(loc="best")
plt.show()