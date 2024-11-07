# Import necessary libraries
from numpy import *
import matplotlib.pyplot as plt  # Necessary for graphing
from random import *

######################################################################################
# Time
step_size = 0.5
time_limit = 1000  # Increase or decrease if timing out too quickly / taking too long
######################################################################################
# Stuff to change
kP = 0.25             # kP 0.25
kI = 0.05             # kI 0.05
kD = 2.25             # kD 2.25
initial_angle = 0     # Initial Angle
target_angle = 1000   # Target Angle

# Settle controller parameters
acceptable_error = 5
settle_speed = 5
######################################################################################
# Keep same
max_speed = 300        # Max Velocity of model
max_acceleration = 75  # Max Acceleration of model
minimum_acceleration = 20
coefficient_of_friction = 0.15  # Coefficient of kinetic friction #0.15
randomness_coefficient = 0.5    # Randomness of how quickly it slows down
######################################################################################
# Arrays
time_array = [0]
current_angle_array = [initial_angle]
velocity_array = [0]
acceleration_array = [0]
error_array = [initial_angle - target_angle]
integral_error_array = [0]
derivative_error_array = [0]
######################################################################################
def controller(starting_angle, target_angle, kP, kI, kD):
    global coefficient_of_friction, max_acceleration, minimum_acceleration, step_size, max_speed
    error_array.append(current_angle_array[-1]- target_angle)
    derivative_error = (error_array[-1] - error_array[-2]) / step_size
    derivative_error_array.append(derivative_error)


    if abs(error_array[-1]) < 50:
        integral_error_array.append(integral_error_array[-1] + error_array[-1] * step_size)
    else:
        integral_error_array.append(0)


    acceleration = - (error_array[-1] * kP + integral_error_array[-1] * kI + derivative_error * kD)


    friction_deceleration = velocity_array[-1] * coefficient_of_friction * uniform(0, randomness_coefficient)


    if abs(acceleration) > max_acceleration:
        acceleration = max_acceleration * (1 if acceleration > 0 else -1)

    if abs(acceleration) < minimum_acceleration:
        acceleration = 0


    acceleration -= friction_deceleration
    acceleration_array.append(acceleration)


    new_velocity = velocity_array[-1] + acceleration_array[-1] * step_size
    if abs(new_velocity) > max_speed:
        new_velocity = max_speed * (1 if new_velocity > 0 else -1)
    velocity_array.append(new_velocity)


    current_angle_array.append(current_angle_array[-1] + velocity_array[-1] * step_size)
    
while abs(error_array[-1]) > acceptable_error or abs(velocity_array[-1]) > settle_speed:
    controller(current_angle_array[-1], target_angle, kP, kI, kD)
    time_array.append(time_array[-1] + step_size)
    if time_array[-1] > time_limit:
        print("timeout")
        break

print("Time steps: ", time_array[-1])

# Plotting
plt.figure()
plt.plot(time_array, velocity_array, label='Velocity', color='blue')
plt.plot(time_array, acceleration_array, label='Acceleration', color='green')
plt.legend()
plt.title("Velocity and Acceleration vs. Time")
plt.xlabel("Time")
plt.grid(True)

# plt.figure()
# plt.plot(time_array, error_array, label='Error', color='red')
# plt.legend()
# plt.title("Error vs. Time")
# plt.xlabel("Time")
# plt.grid(True)

plt.figure()
plt.plot(time_array, derivative_error_array, label='Derivative Error', color='red')
plt.legend()
plt.title("Derivative Error vs. Time")
plt.xlabel("Time")
plt.grid(True)

plt.figure()
plt.plot(time_array, current_angle_array, label='Current Angle', color='green')
plt.axhline(y=target_angle, color='r', linestyle='--', label='Target Angle')
plt.legend()
plt.title("Angle vs. Time")
plt.xlabel("Time")
plt.grid(True)

plt.figure()
plt.plot(time_array, integral_error_array, label='Integral Error', color='green')
plt.legend()
plt.title("Integral Error vs. Time")
plt.xlabel("Time")
plt.grid(True)

plt.show()
