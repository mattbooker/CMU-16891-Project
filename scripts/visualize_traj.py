import numpy as np
import matplotlib.pyplot as plt

def draw_ball(loc, rad, ax):
    x_loc, y_loc, z_loc = loc
    u, v = np.mgrid[0:2*np.pi:20j, 0:np.pi:10j]
    x = rad * np.cos(u)*np.sin(v) + x_loc
    y = rad * np.sin(u)*np.sin(v) + y_loc
    z = rad * np.cos(v) + z_loc
    ax.plot_wireframe(x, y, z, color="r")

data = np.loadtxt("test.txt", delimiter=',')
data_jul = np.loadtxt("test_julia.txt", delimiter=',')

print(data.shape)
ax = plt.figure().add_subplot(projection='3d')
ax.plot(data[:, 0], data[:, 1], data[:, 2])
ax.plot(data_jul[:, 0], data_jul[:, 1], data_jul[:, 2])


# draw sphere
draw_ball([1, 1, 1], 0.8, ax)
draw_ball([1, 1, 2], 0.8, ax)
draw_ball([1, 1, 0], 0.8, ax)

plt.show()
