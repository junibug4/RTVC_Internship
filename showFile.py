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
