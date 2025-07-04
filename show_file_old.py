#%% ---- Imports ------------------------------------------------
import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import curve_fit
# %% ------ Plots the acceleration of the spinning rocket -------
# ## --- Use this to calculate the rocket's moment of inertia ---


filename = 'datafiles/spin_fan45_05_1139.txt' # 'datafiles/spin_fan15_04_1444.txt' # 'datafiles/spin_noFan_04_1413.txt'
fullData = np.loadtxt(filename)
fulltimeAxis = np.linspace(0,(len(fullData)*0.008),len(fullData)) # timestep for arduino is 8 ms

start,end = int(5.05/0.008), int(6.1/0.008)
data = fullData[start:end]  # Remove first START and last END samples to remove initial noise
timeAxis = fulltimeAxis[start:end]  # Adjust time axis accordingly


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
plt.hlines(0,timeAxis[0], timeAxis[-1], colors='orange', linestyles='dashed')
plt.xlabel('Time (s)')
plt.ylabel('Displacement (degrees)')   
plt.show()

print(start, end)
print(len(data), len(timeAxis))

#%% ----------- Writing a fitting function ---------------------

def square(x, a, b=0, c=0):
    return a*x**2 + b*x + c

print(start, end)
print(len(data), len(timeAxis))

params, covariance = curve_fit(square,timeAxis , data)
print(f"Fitted parameter: {params}")


plt.plot(timeAxis, data, label='Data', alpha=0.6)
plt.plot(timeAxis, square(timeAxis, *params), color='red', 
         linestyle='dashed', label='Fitted Curve', alpha=0.7)
plt.legend()
plt.show()

#%% ------------------------------------------------------------

filename = 'datafiles/spin_noFan_05_1116.txt'
data = np.loadtxt(filename)
timeAxis = np.linspace(0,(len(data)*0.008),len(data)) # timestep for arduino is 8 ms

start,end = int(1/0.008),int(15.5/0.008) # 1 second to 15.5 seconds
data = data[start:end]  # Remove first START and last END samples to remove initial noise
timeAxis = timeAxis[start:end]  # Adjust time axis accordingly


# for i in range(len(data)):
#     if data[i] < 0:
#         data[i] += 360

peaksx, peaksy, negpeaksx, negpeaksy = [],[],[],[]
for i in range (len(data[2:])):
    if (data[i-2]-data[i-1] < 0.5 and data[i-1]-data[i] > -0.01):
        if (data[i] > 0):
            peaksx.append(timeAxis[i])
            peaksy.append(data[i])
        else:
            negpeaksx.append(timeAxis[i])
            negpeaksy.append(data[i])



# plt.plot(timeAxis, data, label='Spin Data')
plt.scatter(peaksx[1:], peaksy[1:], color='red', label='Peaks', s=10)
plt.scatter(negpeaksx[1:], negpeaksy[1:], color='yellow', label='Peaks', s=10)
plt.legend()
plt.hlines(0, timeAxis[0], timeAxis[-1], colors='orange', linestyles='dashed')
plt.xlabel('Time (s)')
plt.ylabel('Displacement (degrees)')   
plt.show()

#%% ----------- Writing a fitting function ---------------------

def friction_curve(x, a, b=0, c=0):
    return a*np.exp(-b*x) + c


params, covariance = curve_fit(friction_curve, peaksx[1:], peaksy[1:], p0=(200,1,100))
print(f"Fitted parameter: {params}")

# negparams, negcovariance = curve_fit(negfriction_curve, negpeaksx[1:], negpeaksy[1:], p0=(200,1,50))
# print(f"Negative Fitted parameter: {negparams}")


plt.plot(timeAxis, data, label='Data', alpha=0.8)
plt.scatter(timeAxis, friction_curve(timeAxis, *params), color='red', 
        label='Fitted Curve', alpha=0.7, s = 1)
# plt.scatter(timeAxis, friction_curve(timeAxis, *negparams), color='yellow', 
#         label='Fitted Curve', alpha=0.7, s = 1)
plt.legend()
plt.show()

#%% ------------------------------------------------------- %%#
#===--------- Display arbitrary file ----------------------===#
#===-------------------------------------------------------===#

filename = 'datafiles/spin_fan45_05_1139.txt'
data = np.loadtxt(filename)
timeAxis = np.linspace(0,(len(data)*0.008),len(data)) # timestep for arduino is 8 ms

start,end = int(5.05/0.008), int(6.1/0.008)
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
plt.hlines(0,timeAxis[0], timeAxis[-1], colors='orange', linestyles='dashed')
plt.xlabel('Time (s)')
plt.ylabel('Displacement (degrees)')   
plt.show()

print(len(data))