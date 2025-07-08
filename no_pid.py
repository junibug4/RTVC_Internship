#%%
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit

#%% ===----------- Load data from file ----------=== %%#

filename = 'datafiles/friction_05_1435.txt' # 'datafiles/spin_nofan_05_1116.txt'
data = np.loadtxt(filename)
timeAxis = np.linspace(0, (len(data) * 0.008), len(data))
start, end = int(2.65 / 0.008), -20  # 0, len(data)
data = data[start:end]
timeAxis , dt = timeAxis[:-int(2.65/0.008) -20] , timeAxis[1] - timeAxis[0]


plt.plot(timeAxis, data, label='Spin Data')


#%% ------------------------------------------------ %%#

# timeAxis = np.arange(0,30,1000)  # Adjust time axis accordingly

def pendulum_model(timeAxis, fan_angle, p0, L, Beta, w0=0):
    dt = timeAxis[1] - timeAxis[0]
    mass = 0.4508
    thrust = 0.015
    g = 9.81

    # w0 = 0
    # p0 = 60
    # L = .04
    # Beta = 6e-5
    # fan_angle = 15 


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

            # w = w + dt * (gravity_term + driving_term + drag_term) 
            w = w + dt * (gravity_term  + drag_term) 

            position_array.append(p)
            W.append(w)
    return position_array

P = pendulum_model(timeAxis, fan_angle=15, p0=60, L=0.04, Beta=6e-5)



# plt.plot(timeAxis, W, label='Pendulum Model', color='green', linestyle='dashed', alpha=0.4)
plt.plot(timeAxis, P, label='Pendulum Model')


#%% --- Fit the model to the data --- %%#

guess = [0, -100, 0.04, 6e-5]

params, covariance = curve_fit(pendulum_model, timeAxis, data, p0=guess)
print(f"Fitted parameters: {params}")

plt.plot(timeAxis, pendulum_model(timeAxis, *params), color='red', linestyle='dashed', label='Fitted Model')
plt.plot(timeAxis, data, label='Measured Data', alpha=0.6)
# %%
