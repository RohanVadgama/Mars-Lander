# uncomment the next line if running in a notebook
# %matplotlib inline
import numpy as np
import matplotlib.pyplot as plt

# mass of mars, constant, initial position and velocity
M = 6.42 * 10**23
r_mars = 3.389e6
G = 6.6743015 / (10**(11))
k = G*M

'''
#Straight down descent
r = np.array([0, 0, r_mars])
v = np.array([0, 0, 0])
'''
'''
# circular orbit
r = np.array([0, 0, r_mars + 100e3])  # 100km above surface
v_circular = -np.sqrt(G*M/(np.linalg.norm(r)))  #circular orbit velocity
v = np.array([v_circular, 0, 0])
'''

# elliptical orbit
r = np.array([0, 0, r_mars + 100e3])  # 100km above surface
v_elliptical = np.sqrt(G*M/(np.linalg.norm(r))) * 0.8  # 0.8x circular velocity
v = np.array([v_elliptical, 0, 0])

'''
# hyperbolic escape
r = np.array([0, 0, r_mars + 100e3])
v_escape = np.sqrt(2*G*M/(np.linalg.norm(r)))  # Escape velocity
v = np.array([v_escape * 1.2, 0, 0])  # 1.2x escape velocity
'''


# simulation time, timestep and time
t_max = 10000
dt = 0.01
t_array = np.arange(0, t_max, dt)   #return array with evenly spacec values

# initialise empty lists to record trajectories
r_list = [r]
v_list = []
r = r + dt * v
# Euler integration
for t in range(1 , len(t_array)):

    # append current state to trajectories
    r_list.append(r)
    v_list.append(v)

    #magnitude of array as below

    mag = np.sqrt(r.dot(r))

    # calculate new position and velocity
    a = -(k/(mag**3)) * r
    r = 2*r - r_list[t - 1] + a*(dt**2)
    v = (1/(2*dt))*(r - r_list[t - 2])

# convert trajectory lists into arrays, so they can be sliced (useful for Assignment 2)
r_array = np.array(r_list)
x_array = r_array[:, 0]
y_array = r_array[:, 1]
z_array = r_array[:, -1]
v_array = np.array(v_list)

# plot the position-time graph

plt.clf()
plt.title("euler")

plt.grid()
'''
#for straight down descent
plt.xlabel('time (s)')
plt.plot(t_array, z_array, label='x (m)')
'''
#for others
plt.xlabel('x (m)')
plt.ylabel('z (m)')
plt.plot(x_array, z_array, label='position (m)')
plt.legend()
plt.show()


'''
euler method leads to a growth in v/x over time which is impossible,
as the estimated expansion is always an overestimate.
'''