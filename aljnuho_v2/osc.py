import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

xstart = np.array([0.0, 0.0])
xgoal  = np.array([1.0, 1.0])

omega = 2.0  # oscillation speed

fig, ax = plt.subplots()
ax.set_xlim(-0.2, 1.2)
ax.set_ylim(-0.2, 1.2)
ax.set_aspect('equal')

# draw endpoints
ax.scatter(*xstart)
ax.scatter(*xgoal)

# point to animate
point, = ax.plot([0], [0], 'o')

def update(frame):
    t = frame * 0.05
    alpha = 0.5 * (1 + np.sin(omega * t))
    x = (1 - alpha) * xstart + alpha * xgoal
    point.set_data([x[0]], [x[1]])
    return point,

ani = animation.FuncAnimation(
    fig, update,
    frames=300,
    interval=30,
    blit=True
)

plt.show()