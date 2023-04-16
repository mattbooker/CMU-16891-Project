import numpy as np
import matplotlib.pyplot as plt

def draw_ball(loc, rad, ax):
    x_loc, y_loc, z_loc = loc
    u, v = np.mgrid[0:2*np.pi:20j, 0:np.pi:10j]
    x = rad * np.cos(u)*np.sin(v) + x_loc
    y = rad * np.sin(u)*np.sin(v) + y_loc
    z = rad * np.cos(v) + z_loc
    ax.plot_wireframe(x, y, z, color="k")

data = np.loadtxt("test.txt", delimiter=',')
data_jul = np.loadtxt("test_julia.txt", delimiter=',')

ax = plt.figure().add_subplot(projection='3d')
ax.plot(data[:, 0], data[:, 1], data[:, 2])
ax.plot(data_jul[:, 0], data_jul[:, 1], data_jul[:, 2])

ax.scatter(data[:, 0], data[:, 1], data[:, 2])
ax.scatter(data_jul[:, 0], data_jul[:, 1], data_jul[:, 2])

plt.xlim([-2, 2])
plt.ylim([-2, 2])

diffs = np.linalg.norm(data[:, ] - data_jul[:, ], axis=1)
idx = np.argmin(diffs, axis=0)
print(idx)

plt.plot([data[idx, 0], data_jul[idx, 0]], [data[idx, 1], data_jul[idx, 1]], [data[idx, 2], data_jul[idx, 2]], "r")
# plt.plot(data[idx, 0], data[idx, 1], data[idx, 2], "r")

# draw sphere
draw_ball([-1.37395e-08, 0.958905, 0.393272], 0.5, ax)
draw_ball([0, 2, -2], 0.5, ax)
draw_ball([0.26048, 1.36839, -0.329271], 0.5, ax)

plt.show()
