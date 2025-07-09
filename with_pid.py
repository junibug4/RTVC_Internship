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

#%% ===------ Defining Physical Model -----------------------------===#

pid = PIDController(Kp=.2, Ki=0.1, Kd=.8, setpoint=0)

def driven_pendulum_model(timeAxis, p0, L, Beta_L = 6e-9 , Beta_Q = 8e-8 , w0=0, thrust = 0.015,  mass = 0.4508):
    dt = (timeAxis[1] - timeAxis[0])*1.1
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

            fan_angle = np.clip(fan_angle, -70, 70)

            gravity_term = - abs(g / L) * np.sin(np.radians(p))
            driving_term = thrust / (mass * L**2) * np.sin(np.radians(fan_angle))
            linear_drag_term = -2 * Beta_L * w**2 / (mass * L**2)
            quadratic_drag_term = -2 * Beta_Q * w**2 / (mass * L**2)

            w = w + dt * (gravity_term + driving_term + linear_drag_term + quadratic_drag_term) 
            # w = w + dt * (gravity_term  + drag_term) 

            position_array.append(p)
            W.append(w)
    return position_array


#%% ------------------------------------------------------------
# Simulate the driven pendulum model

setpoint = 0
experimental_args = [-40, 0.009, 6e-09, 8e-08, 350, 0.0132]

pid = PIDController(Kp=.2, Ki=0.3, Kd=.3, setpoint=setpoint)

timeAxis = np.linspace(0.05, 2.5, 60)  # 10 seconds, 1000 steps
P = driven_pendulum_model(timeAxis, *experimental_args)

# plt.plot(timeAxis, P, label='Driven Pendulum Model', color='blue')

# plt.show()

# --- Load Experimetnal Data And Compare --- %%#

plt.plot(timeAxis, P , label = 'Simulation')

filename , start , end = 'datafiles/pid_233.txt' , int(0.8/0.008), int(2.3/0.008)
data = np.loadtxt(filename)
data = data[start:end]
timeAxis = np.linspace(0,(len(data)*0.008),len(data))

plt.plot(timeAxis, data, label='Set 1', alpha = 0.4, linestyle = '--')

################### vvvv 2 vvvv ###### ^^^^ 1 ^^^^ #######################

filename , start , end = 'datafiles/pid_233_2.txt' , 0, int(2.3/0.008)
data = np.loadtxt(filename)
data = data[start:end]

for i in range (len(data)):
    data[i] = -data[i]

timeAxis = np.linspace(0,(len(data)*0.008),len(data))

plt.plot(timeAxis, data, label='Set 2', alpha = 0.4, linestyle = '--')#, color='orange')

################### vvvv 3 vvvv ###### ^^^^ 2 ^^^^ #######################

filename , start , end = 'datafiles/pid_233_3.txt' , 0, int(2.3/0.008)
data = np.loadtxt(filename)
data = data[start:end]
timeAxis = np.linspace(0.1,(len(data)*0.008),len(data))

plt.plot(timeAxis, data, label='Set 3', alpha = 0.4, linestyle = '--')#, color='orange')

################### vvvv 4 vvvv ###### ^^^^ 3 ^^^^ #######################

filename , start , end = 'datafiles/pid_233_4.txt' , 0, int(2.3/0.008)
data = np.loadtxt(filename)
data = data[start:end]
timeAxis = np.linspace(0,(len(data)*0.008),len(data))
data4 , time4 = data , timeAxis

plt.plot(timeAxis, data, label='Set 4', alpha = 0.4, linestyle = '--')#, color='orange')

################### vvvv 5 vvvv ###### ^^^^ 4 ^^^^ #######################

filename , start , end = 'datafiles/pid_233_5.txt' , 0, int(2.5/0.008)
data = np.loadtxt(filename)
data = data[start:end]
timeAxis = np.linspace(0,(len(data)*0.008),len(data))

plt.plot(timeAxis, data, label='Set 5', alpha = 0.4, linestyle = '--')#, color='orange')
################### ---- - ---- ###### ^^^^ 5 ^^^^ #######################

plt.legend()
plt.show()

#%% --- Fit the model to the data --- %%#

params, covariance = curve_fit(driven_pendulum_model, time4, data4, p0=experimental_args)
# print(f"p0: {params[0]}\nL: {params[1]}\nBeta: {params[2]}\nthrust: {params[3]}\nw0: {params[4]}\nmass: {params[5]}")

plt.plot(timeAxis, driven_pendulum_model(timeAxis, *params), color='red', linestyle='dashed', label='Fitted Model')
plt.plot(timeAxis, data, label='Measured Data', alpha=0.6)
plt.show()

print(experimental_args,'\n', params)
#%% ------ Define Convergence Function ------- %%#

def datapoint_is_zero(x):
    if abs(x) < 0.5:
        return True
    else:
        return False

def get_convergence(array):
    consecutive_convergence_counter = 0
    for index , value in enumerate(array):
        if datapoint_is_zero(value):
            consecutive_convergence_counter += 1
        else:
            consecutive_convergence_counter = 0
        if consecutive_convergence_counter >= 20:
            return index - 21
        if index == len(array)-1:
            print("did not converge")
            return len(array)-1
        
        


#%% 
range_p = [1]#[0.01 ,0.1 , 0.3, 0.5, .75 , 1 , 1.5]
range_d = [1]#[0.5, .75 , 1 , 1.5, 2]

ten_secs = np.linspace(0,10,1000)

for test_p in range_p:
    for test_d in range_d:
        if test_d != 0:
            pid = PIDController(Kp=test_p, Ki=00, Kd=test_d, setpoint=0)
            pid = PIDController(Kp=1, Ki=00, Kd=.01, setpoint=0)

            simulation = driven_pendulum_model(ten_secs, *params)

            # plt.plot(ten_secs, simulation , label = str(test_p))

    # print("^^^  kp = ", test_p, " ^^^")

    plt.plot(ten_secs, simulation , label = str(test_p))
    plt.show()

    print(get_convergence(simulation))

    print("Model converges within a degree after ",ten_secs[get_convergence(simulation)], " seconds.")

