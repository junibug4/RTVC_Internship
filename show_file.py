#%% ---- Imports ------------------------------------------------
import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import curve_fit


#%% ------------------------------------------------------- %%#
#===--------- Display arbitrary file ----------------------===#
#===-------------------------------------------------------===#

filename = 'datafiles/morning06.txt'
data = np.loadtxt(filename)
timeAxis = np.linspace(0,(len(data)*0.008),len(data)) # timestep for arduino is 8 ms

start,end = 0, len(data)
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