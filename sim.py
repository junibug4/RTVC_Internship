#%% ------------------------------------------------------------
import matplotlib.pyplot as plt
import numpy as np
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
time = np.linspace(0, 100, 1000)  # 100 seconds, 1000 steps
dt = time[1] - time[0]

#===--- Parameters from pendulum.py ------------------------===#
displacement = 3
track_disp = []
velocity = 0
gravity = 9.810
thrust0 = .15
fan_angle = 45  # Angle of the fan in degrees
mass = 450.8E-3
inertia = 0.2  # Moment of inertia of the pendulum
#===--------------------------------------------------------===#


# Simulate the process

pid = PIDController(Kp=1.0, Ki=1.0, Kd=1.0, setpoint=setpoint)
for t in time:
    # PID control output
    control_output = compute(pid,fan_angle, dt)
    
    #===--- Process from pendulum.py -----------------------===#
    torque = -gravity * np.sin(np.radians(displacement)) + np.sin(np.radians(fan_angle)) * thrust0
    angular_acceleration = torque / inertia
    velocity += angular_acceleration * dt
    displacement += velocity * dt
    track_disp.append(displacement)
    #===----------------------------------------------------===#


# Plot results

plt.figure(figsize=(10, 6))
plt.plot(time, track_disp, label='Process Variable (Angle)')
plt.axhline(y=setpoint, color='r', linestyle='--', label='Setpoint')
plt.xlabel('Time (s)')
plt.ylabel('Angle')
plt.title('PID Controller Simulation')
plt.legend()
plt.grid()
plt.show()

