#%% ---- Imports -----------------------------------------------
import matplotlib.pyplot as plt
import numpy as np

#%% ------------------------------------------------------------

time = np.linspace(0, 500, 1000)  # 10 seconds, 100 steps
dt = time[1] - time[0]
displacement = 30
track_disp = []
velocity = 0
gravity = 9.810
thrust0 = .015
fan_angle = 10  # Angle of the fan in degrees
inertia = 0.04  # Moment of inertia of the pendulum
com_sep = 0.002  # Center of mass separation from pivot point
fan_sep = 0.2  # Fan separation from pivot point
friction_coefficient = 0.001  # Coefficient of friction


for t in time:
    
    # Implement PID control
    # Calculate change in displacement in one time step
    torque = gravity * np.sin(np.radians(displacement)) * com_sep #+ np.sin(np.radians(fan_angle)) * thrust0 * fan_sep
    
    friction = np.exp(-t * friction_coefficient)  # Simulate friction that decreases over time

    # torque *= friction
    angular_acceleration = torque / inertia
    velocity += angular_acceleration * dt
    displacement += velocity * dt * friction

    # if displacement > 180:
    #     displacement -= 360
    # elif displacement < -180:
    #     displacement += 360

    track_disp.append(displacement)

plt.plot(time, track_disp)
plt.xlabel('Time (s)')
plt.hlines(180,time[0], time[-1], colors='orange', linestyles='dashed')
plt.ylabel('Displacement (degrees)')
plt.show()

# %% ------------------------------------------------------------

filename = 'spin_noFan_04_1413.txt'
data = np.loadtxt(filename)

timeAxis = np.linspace(0,(len(data)*0.008),len(data)) # timestep for arduino is 8 ms

for i in range(len(data)):
    if data[i] < 0:
        data[i] += 360

plt.plot(timeAxis[350:], data[350:], label='Spin Data')
plt.hlines(160,timeAxis[0], timeAxis[-1], colors='orange', linestyles='dashed')
plt.xlabel('Time (s)')
plt.ylabel('Displacement (degrees)')   
plt.show()


