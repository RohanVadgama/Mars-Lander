import numpy as np
import matplotlib.pyplot as plt

# mass, spring constant, initial position and velocity
m = 1
k = 1
x = 0
v = 1
'''
critical value for dt = 1
Above 1, simulation breaks
'''

# simulation time, timestep and time
t_max = 100
dt = 1
t_array = np.arange(0, t_max, dt)   #return array with evenly spacec values

x_1 = x + dt*v
# initialise lists to record trajectories with first two values
x_list = [x, x_1]
v_list = [v]

# Verlet integration
for t in range(2, len(t_array)):
    a = -k * x_list[t-1] / m
    x_t = 2*x_list[t-1] - x_list[t-2] + a*(dt**2)
    v_tminus1 = (1/(2*dt))*(x_t - x_list[t-2])
    # append current state to trajectories
    x_list.append(x_t)
    v_list.append(v_tminus1)
v_list.append(v_tminus1) #make length match

# convert trajectory lists into arrays, so they can be sliced (useful for Assignment 2)
x_array = np.array(x_list)
v_array = np.array(v_list)

# plot the position-time graph

plt.clf()
plt.title("verlet")
plt.xlabel('time (s)')
plt.grid()
plt.plot(t_array, x_array, label='x (m)')
plt.plot(t_array, v_array, label='v (m/s)')
plt.legend()
plt.show()
