#%% ------------------------------------------------------------
import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import curve_fit
#%% ------------------------------------------------------------

class PIDController:
    def __init__(self, Kp, Ki, Kd, setpoint):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self.previous_error = 0
        self.integral = 0

def compute(self, process_variable, dt):
        # Calculate error
        error = self.setpoint - process_variable
        
        # Proportional term
        P_out = self.Kp * error
        
        # Integral term
        self.integral += error * dt
        I_out = self.Ki * self.integral
        
        # Derivative term
        derivative = (error - self.previous_error) / dt
        D_out = self.Kd * derivative
        
        # Compute total output
        output = P_out + I_out + D_out
        
        # Update previous error
        self.previous_error = error
        
        return output

#%% ------------------------------------------------------------
# Initialize PID controller
setpoint = 0  # Desired temperature

# Simulation parameters
time = np.linspace(0, 10, int(10/0.008))  # 10 seconds, 1000 steps
dt = time[1] - time[0]


#%% ===------ Implementing pid -----------------------------===#


experimental_args = [0 , 0.0078 , 4.8e-6, 360]# , 0.015 , 0.45] # 

def driven_pendulum_model(timeAxis, p0, L, Beta = 6e-6 , w0=0, thrust = 0.015,  mass = 0.4508):
    dt = timeAxis[1] - timeAxis[0]
    g = 9.81

    position_array , W = [p0], [w0]

    X = np.zeros(len(timeAxis))
    for i in range(len(timeAxis)):
        if i == 0:
            p , w = p0 , w0 #, fan_angle0
        else:
            p = p + w * dt 

            # Compute PID output
            fan_angle = compute(pid, p, dt)

            gravity_term = - abs(g / L) * np.sin(np.radians(p))
            driving_term = thrust / (mass * L**2) * np.sin(np.radians(fan_angle))
            drag_term = -2 * Beta * w / (mass * L**2)

            w = w + dt * (gravity_term + driving_term + drag_term) 
            # w = w + dt * (gravity_term  + drag_term) 

            position_array.append(p)
            W.append(w)
    return position_array

# ------------------------------------------------------------
# Simulate the driven pendulum model

pid = PIDController(Kp=.2, Ki=0.3, Kd=.3, setpoint=setpoint)

timeAxis = np.linspace(0, 5, 1000)  # 10 seconds, 1000 steps
P = driven_pendulum_model(timeAxis, *experimental_args)

plt.plot(timeAxis, P, label='Driven Pendulum Model', color='blue')

filename = 'datafiles/pid_233.txt'
data = np.loadtxt(filename)


start,end = int(1/0.008), len(data)
data = data[start:end]  # Remove first START and last END samples to remove initial noise
timeAxis = np.linspace(0,(len(data)*0.008),len(data))

plt.plot(timeAxis, data, label='Experimental Data', color='orange')

plt.show()


#%% --- Fit the model to the data --- %%#

params, covariance = curve_fit(driven_pendulum_model, timeAxis, data, p0=experimental_args)
# print(f"p0: {params[0]}\nL: {params[1]}\nBeta: {params[2]}\nthrust: {params[3]}\nw0: {params[4]}\nmass: {params[5]}")

plt.plot(timeAxis, driven_pendulum_model(timeAxis, *params), color='red', linestyle='dashed', label='Fitted Model')
plt.plot(timeAxis, data, label='Measured Data', alpha=0.6)
plt.show()

print(experimental_args,'\n', params)
# %%
