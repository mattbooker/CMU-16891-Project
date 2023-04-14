import numpy as np
import matplotlib.pyplot as plt


with open("all_traj.txt") as f:
    data = f.readlines()

numAgents = sum([1 if l == "---\n" else 0 for l in data]) + 1
print(numAgents)

trajs = [[] for _ in range(numAgents)]

current_agent = 0
for line in data:
    if line == "---\n":
        current_agent += 1
        continue

    trajs[current_agent].append([float(val) for val in line.split(", ")])

trajs = np.array(trajs)
print(trajs.shape)

ax = plt.figure().add_subplot(projection='3d')

for i in range(numAgents):
    ax.plot(trajs[i, :, 0], trajs[i, :, 1], trajs[i, :, 2])
    ax.scatter(trajs[i, -1, 0], trajs[i, -1, 1], trajs[i, -1, 2])

ax.axes.set_xlim3d(left=-5, right=5)
ax.axes.set_ylim3d(bottom=-5, top=5) 
ax.axes.set_zlim3d(bottom=-5, top=5) 

plt.xlabel("x")
plt.ylabel("y")
plt.show()
