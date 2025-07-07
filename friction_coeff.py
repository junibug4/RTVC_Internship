#%% ---- Imports ------------------------------------------------
import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import curve_fit

#%% ------------------------------------------------------------

filename = 'datafiles/friction_05_1435.txt'
data = np.loadtxt(filename)
timeAxis = np.linspace(0,(len(data)*0.008),len(data)) # timestep for arduino is 8 ms

start,end = int(2.65/0.008) , -20# 0,len(data)
data = data[start:end]  # Remove first START and last END samples to remove initial noise
timeAxis = timeAxis[:-int(2.65/0.008) -20]  # Adjust time axis accordingly

# for i in range(len(data)):
#     if data[i] < 0:
#         data[i] += 360

peaksx, peaksy, negpeaksx, negpeaksy = [],[],[],[]
for i in range (len(data[2:])):
    if (data[i-2]-data[i-1] < 0.01 and data[i-1]-data[i] > -0.0):
        if (data[i] > 00):
            peaksx.append(timeAxis[i])
            peaksy.append(data[i])
        else:
            negpeaksx.append(timeAxis[i])
            negpeaksy.append(data[i])



plt.plot(timeAxis, data, label='Spin Data')
plt.scatter(peaksx[1:], peaksy[1:], color='red', label='Peaks', s=10)
# plt.scatter(negpeaksx[1:], negpeaksy[1:], color='yellow', label='Peaks', s=10)
plt.legend()
plt.vlines(2250*0.008,np.max(data), np.min(data), colors='orange', linestyles='dashed')
plt.xlabel('Time (s)')
plt.ylabel('Displacement (degrees)')   
plt.show()

#%% ----------- Writing a fitting function ---------------------

def friction_curve(x, a, b=0, c=0):
    return a*np.exp(-b*x) + c


params, covariance = curve_fit(friction_curve, peaksx[1:], peaksy[1:], p0=(0.002,1,0))
print(f"Fitted parameter: {params}")

# negparams, negcovariance = curve_fit(friction_curve, negpeaksx[1:], negpeaksy[1:], p0=(-200,0,100))
# print(f"Negative Fitted parameter: {negparams}")


plt.plot(timeAxis, data, label='Data', alpha=0.8)
plt.scatter(timeAxis, friction_curve(timeAxis, *params), color='red', 
        label='Fitted Curve', alpha=0.7, s = 0.5)
# plt.scatter(timeAxis, friction_curve(timeAxis, *negparams), color='yellow', 
#         label='Fitted Curve', alpha=0.7, s = 1)
plt.legend()
plt.show()
print("Natural damping coefficient =", params[1])

# %%
