#%% ---- Imports ------------------------------------------------
import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import curve_fit
%matplotlib inline
from IPython.display import HTML
import matplotlib.animation as animation
#%%
import os
print(os.getcwd())
# os.chdir('../')  
#%% ------------------------------------------------------- %%#
#===--------- Display arbitrary file ----------------------===#
#===-------------------------------------------------------===#

filename = 'datafiles/pid_233_multi.txt'
data = np.loadtxt(filename)
timeAxis = np.linspace(0,(len(data)*0.008),len(data)) # timestep for arduino is 8 ms

start,end = int(1/0.008), len(data)
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

splits = [3.05,7.5,10.9,14.3]

plt.plot(timeAxis, data, label='Spin Data')
plt.hlines(0,timeAxis[0], timeAxis[-1], colors='orange', linestyles='dashed')
plt.vlines(splits,np.min(data)-5,np.max(data)+5, colors='red', linestyles='dashed')
plt.xlabel('Time (s)')
plt.ylabel('Displacement (degrees)')   
plt.show()

print(len(data))

#%% ------------------------------------------------------- %%#
#===--------- Animate Cartesian ---------------------------===#

X, Y = [], []
for i in range(len(data)):
    X.append(np.sin(np.radians(data[i])))
    Y.append(-np.cos(np.radians(data[i])))

fig, ax = plt.subplots()
line, = ax.plot([], [], 'b-', label='Cartesian Data')
ax.set_xlim(-1.1, 1.1)
ax.set_ylim(-1.1, 1.1)
ax.set_aspect('equal')

def init():
    line.set_data([], [])
    return line,

def update(frame):
    idx = frame % len(X)
    line.set_data([0, X[idx]], [0, Y[idx]])  # pendulum rod from origin to tip
    return line,

ani = animation.FuncAnimation(fig, update, frames=len(X), init_func=init, blit=True, interval=20)

HTML(ani.to_jshtml())


# %%
