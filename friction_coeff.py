#%% ---- Imports ------------------------------------------------
import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import curve_fit
# %% ------ Plots the acceleration of the spinning rocket -------
# ## --- Use this to calculate the rocket's moment of inertia ---


filename = 'datafiles/spin_fan15_04_1444.txt' # 'datafiles/spin_noFan_04_1413.txt'
data = np.loadtxt(filename)
timeAxis = np.linspace(0,(len(data)*0.008),len(data)) # timestep for arduino is 8 ms

start,end = 0,len(data)#180,590
data = data[start:end]  # Remove first START and last END samples to remove initial noise
timeAxis = timeAxis[start:end]  # Adjust time axis accordingly


# for i in range(len(data)):
#     if data[i] < 0:
#         data[i] += 360


counter=0
scaledData = np.zeros(len(data))
for i in range (len(data[1:])):
    if data[i-1]-data[i] > 100:
        counter += 1
        # print(timeAxis[i],counter)
    scaledData[i] = data[i] + counter*360

timeAxis, data = timeAxis[:-1], scaledData[:-1]

plt.plot(timeAxis, data, label='Spin Data')
plt.hlines(160,timeAxis[0], timeAxis[-1], colors='orange', linestyles='dashed')
plt.xlabel('Time (s)')
plt.ylabel('Displacement (degrees)')   
plt.show()

#%% ----------- Writing a fitting function ---------------------

def square(x, a, b=0, c=0):
    return a*x**2 + b*x + c

start = 190
end = 590

cutData = data[start:end]

params, covariance = curve_fit(square, timeAxis[start:end], cutData)
print(f"Fitted parameter: {params}")


plt.plot(timeAxis, data, label='Data', alpha=0.6)
plt.plot(timeAxis, square(timeAxis, *params), color='red', 
         linestyle='dashed', label='Fitted Curve', alpha=0.7)
plt.legend()
plt.show()

#%% ------------------------------------------------------------

filename = 'datafiles/spin_noFan_04_1413.txt'
data = np.loadtxt(filename)
timeAxis = np.linspace(0,(len(data)*0.008),len(data)) # timestep for arduino is 8 ms

start,end = 350,3100
data = data[start:end]  # Remove first START and last END samples to remove initial noise
timeAxis = timeAxis[start:end]  # Adjust time axis accordingly


for i in range(len(data)):
    if data[i] < 0:
        data[i] += 360

peaksx, peaksy, negpeaksx, negpeaksy = [],[],[],[]
for i in range (len(data[2:])):
    if (data[i-2]-data[i-1] < 0.055 and data[i-1]-data[i] > -0.01 and timeAxis[i] > 5):
        if (data[i] > 160):
            peaksx.append(timeAxis[i])
            peaksy.append(data[i])
        else:
            negpeaksx.append(timeAxis[i])
            negpeaksy.append(data[i])



plt.plot(timeAxis, data, label='Spin Data')
plt.scatter(peaksx[1:], peaksy[1:], color='red', label='Peaks', s=10)
plt.scatter(negpeaksx[1:], negpeaksy[1:], color='yellow', label='Peaks', s=10)
plt.legend()
plt.vlines(2250*0.008,np.max(data), np.min(data), colors='orange', linestyles='dashed')
plt.xlabel('Time (s)')
plt.ylabel('Displacement (degrees)')   
plt.show()

#%% ----------- Writing a fitting function ---------------------

def friction_curve(x, a, b=0, c=0):
    return a*np.exp(-b*x) + c


params, covariance = curve_fit(friction_curve, peaksx[1:], peaksy[1:], p0=(200,0,100))
print(f"Fitted parameter: {params}")

negparams, negcovariance = curve_fit(friction_curve, negpeaksx[1:], negpeaksy[1:], p0=(-200,0,100))
print(f"Negative Fitted parameter: {negparams}")


plt.plot(timeAxis, data, label='Data', alpha=0.8)
plt.scatter(timeAxis, friction_curve(timeAxis, *params), color='red', 
        label='Fitted Curve', alpha=0.7, s = 1)
plt.scatter(timeAxis, friction_curve(timeAxis, *negparams), color='yellow', 
        label='Fitted Curve', alpha=0.7, s = 1)
plt.legend()
plt.show()

#%% ------------------------------------------------------- %%#
#===--------- Display arbitrary file ----------------------===#
#===-------------------------------------------------------===#

filename = 'datafiles/pid_05_1028.txt'
data = np.loadtxt(filename)
timeAxis = np.linspace(0,(len(data)*0.008),len(data)) # timestep for arduino is 8 ms

start,end = 0,len(data)#180,590
data = data[start:end]  # Remove first START and last END samples to remove initial noise
timeAxis = timeAxis[start:end]  # Adjust time axis accordingly


# for i in range(len(data)):
#     if data[i] < 0:
#         data[i] += 360


# counter=0
# scaledData = np.zeros(len(data))
# for i in range (len(data[1:])):
#     if data[i-1]-data[i] > 100:
#         counter += 1
#         # print(timeAxis[i],counter)
#     scaledData[i] = data[i] + counter*360
# timeAxis, data = timeAxis[:-1], scaledData[:-1]

plt.plot(timeAxis, data, label='Spin Data')
plt.hlines(0,timeAxis[0], timeAxis[-1], colors='orange', linestyles='dashed')
plt.xlabel('Time (s)')
plt.ylabel('Displacement (degrees)')   
plt.show()

print(len(data))