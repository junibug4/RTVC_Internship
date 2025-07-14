#%%
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit

#%% ====--------------- Load data from file --------------==== %%#

filename = 'datafiles/friction_05_1435.txt' # pendulum under gravity
data = np.loadtxt(filename)
timeAxis = np.linspace(0, (len(data) * 0.008), len(data))
start, end = int(2.65 / 0.008), -20  # start and end are used to clip the region of interest
data = data[start:end]
timeAxis , dt = timeAxis[:-int(2.65/0.008) -20] , timeAxis[1] - timeAxis[0]


plt.plot(timeAxis, data, label='Experimental Data', color='k', linestyle='dashed', alpha=0.4)
plt.legend()
plt.show()

#%% ====------------- Define Pendulum Motion -------------==== %%#

def pendulum_model(timeAxis, fan_angle, p0, L, Beta, thrust = 0.015, w0=0 , mass = 0.4508):
    dt = timeAxis[1] - timeAxis[0]
    g = 9.81


    position_array , W = [p0], [w0]

    X = np.zeros(len(timeAxis))
    for i in range(len(timeAxis)):
        if i == 0:
            p , w = p0 , w0
        else:
            p = p + w * dt 

            gravity_term = - abs(g / L) * np.sin(np.radians(p))
            driving_term = thrust / (mass * L**2) * np.sin(np.radians(fan_angle))
            drag_term = -2 * Beta * w / (mass * L**2)

            w = w + dt * (gravity_term + driving_term + drag_term) 
            # w = w + dt * (gravity_term  + drag_term) 

            position_array.append(p)
            # W.append(w)
    return position_array


P = pendulum_model(timeAxis, fan_angle=0, p0=60, L=0.01, Beta=5e-6, thrust=0)

plt.plot(timeAxis, P, label='Pendulum Model')
plt.legend()
plt.show()


#%% ====----------- Fit the model to the data -----------==== %%#

guess = [0, -100, 0.03, 6e-5]  # Initial guess for parameters: [fan_angle, p0, L, Beta, w0, mass, thrust]

params, covariance = curve_fit(pendulum_model, timeAxis, data, p0=guess)
print(f"fan_angle: {params[0]}\np0: {params[1]}\nL: {params[2]}\nBeta: {params[3]}")#\nthrust: {params[4]}")

plt.plot(timeAxis, data, label='Experimental Data', color='k', linestyle='dashed', alpha=0.4)
plt.plot(timeAxis, pendulum_model(timeAxis, *params), label='Fitted Model', alpha = 0.8)
plt.legend()
plt.show()
# %%
