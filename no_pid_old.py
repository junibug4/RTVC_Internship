#%% ---- Imports -----------------------------------------------
import matplotlib.pyplot as plt
import numpy as np

#%% ------------------------------------------------------------

time = np.linspace(0, 20, 500)  # 10 seconds, 100 steps
dt = time[1] - time[0]
displacement = 70
track_disp = []
velocity = 0
gravity = 9.810
mass = 450.8E-3 # Mass of the rocket
thrust0 = .015
fan_angle = 10  # Angle of the fan in degrees
inertia = 4.75E-5  # Moment of inertia of the pendulum
com_sep = 0.0103  # Center of mass separation from pivot point
fan_sep = 0.2  # Fan separation from pivot point
friction_coefficient = .08576  # Coefficient of friction


# for t in time:
    
#     # Implement PID control
#     # Calculate change in displacement in one time step
#     torque = gravity * np.sin(np.radians(displacement)) * com_sep * mass #+ np.sin(np.radians(fan_angle)) * thrust0 * fan_sep
    
#     friction = np.exp(-t * friction_coefficient)  # Simulate friction that decreases over time

#     # torque *= friction
#     angular_acceleration = torque / inertia
#     velocity += angular_acceleration * dt
#     displacement += velocity * dt * friction

#     # if displacement > 180:
#     #     displacement -= 360
#     # elif displacement < -180:
#     #     displacement += 360

#     track_disp.append(displacement)

def rocket(t, theta0, damping):
    g = 9.81  # acceleration due to gravity
    l = 1.0   # length of the pendulum
    theta = theta0 * np.cos(np.sqrt(g/l) * t) * np.exp(-damping * t)
    return theta

measured_friction_coefficients = [1.11670039e+02 , 8.57610786e-02 , -1.85267345e+01]
def friction_curve(x, a, b=0, c=0):
    return a*np.exp(-b*x) + c


plt.plot(time, rocket(time, displacement, friction_coefficient), label='Damped Pendulum')
plt.xlabel('Time (s)')
plt.plot(time, friction_curve(time, *measured_friction_coefficients), label='Friction Curve', linestyle='dashed')
plt.legend()
plt.ylabel('Displacement (degrees)')
plt.show()

# %% ------------------------------------------------------------

filename = 'datafiles/spin_noFan_04_1413.txt'
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


#%%

# creating a simple damped pendulum model
def damped_pendulum(t, theta0, damping, com_sep):
    g = 9.81  # acceleration due to gravity
    l = com_sep   # length of the pendulum
    theta = theta0 * np.cos(np.sqrt(g*mass/l) * t) * np.exp(-damping * t)
    return theta

plt.plot(time, damped_pendulum(time, displacement, friction_coefficient, com_sep), label='Damped Pendulum')
# %%
