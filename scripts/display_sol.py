import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation

with open("all_traj.txt") as f:
    data = f.readlines()

numAgents = sum([1 if l == "---\n" else 0 for l in data]) + 1
print("Num agents = ", numAgents)

trajs = [[] for _ in range(numAgents)]

current_agent = 0
for line in data:
    if line == "---\n":
        current_agent += 1
        continue

    trajs[current_agent].append([float(val) for val in line.split(", ")])

trajs = np.array(trajs)

# fig = plt.figure()
fig = plt.figure(figsize=plt.figaspect(1))
ax = fig.add_subplot(projection='3d')

for i in range(numAgents):
    ax.plot(trajs[i, :, 0], trajs[i, :, 1], trajs[i, :, 2])
    # ax.scatter(trajs[i, :, 0], trajs[i, :, 1], trajs[i, :, 2])
    # ax.scatter(trajs[i, -1, 0], trajs[i, -1, 1], trajs[i, -1, 2])

max_x = np.max(trajs[:, :, 0])
min_x = np.min(trajs[:, :, 0])
max_y = np.max(trajs[:, :, 1])
min_y = np.min(trajs[:, :, 1])
max_z = np.max(trajs[:, :, 2])
min_z = np.min(trajs[:, :, 2])

ax.axes.set_xlim3d(left=min_x - 0.5, right=max_x + 0.5)
ax.axes.set_ylim3d(bottom=min_y - 0.5, top=max_y + 0.5) 
ax.axes.set_zlim3d(bottom=min_z - 0.5, top=max_z + 0.5) 

plt.xlabel("x")
plt.ylabel("y")

def animate(frame):
    ax.view_init(elev=10., azim=frame*2)
    return None

# Animate
anim = animation.FuncAnimation(fig, animate,
                               frames=180, interval=20)
# Save
anim.save('rotate_view.gif', fps=30)
# plt.show()