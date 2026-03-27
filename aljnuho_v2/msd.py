import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# system params
m = 1.0
# c = 0.5
k_pos = 2.0  # normal spring
k_neg = -1.0  # negative stiffness
dt = 0.02
c = 2.2 * np.sqrt(k_pos * m)

# initial state
x_pos, vx_pos = 0.8, 0.0
y_pos, vy_pos = 0.0, 0.0
x_wall = 0.0
y_wall = 0.0

fig, ax = plt.subplots()
ax.set_xlim(-2, 2)
ax.set_ylim(-0.5, 0.5)

# draw wall
ax.axvline(x_wall)

# points
(pt_pos,) = ax.plot([], [], "o")

ax.legend()


def step(x, v, k):
    # spring-damper force: F = -k(x-xw) - c*v
    a = (-k * (x - x_wall) - c * v) / m
    v = v + a * dt
    x = x + v * dt
    return x, v


def step_y(y, v, k):
    # spring-damper force: F = -k(y-yw) - c*v
    a = (-k * (y - y_wall) - c * v) / m
    v = v + a * dt
    y = y + v * dt
    return y, v


krange = list(range(150, 520))


def mouse_move(event):
    global x_pos, vx_pos, k_pos, x_wall, y_wall, y_pos, vy_pos

    x, y = event.xdata, event.ydata
    if x is not None and y is not None:
        x_wall = x
        y_wall = y
        x_pos, vx_pos = step(x_pos, vx_pos, k_pos)
        y_pos, vy_pos = step_y(y_pos, vy_pos, k_pos)
        pt_pos.set_data([x_pos], [y_pos])

        plt.draw()


def change_k(event):
    global k_pos
    if event.key == "up":
        k_pos = 2.0
    elif event.key == "down":
        k_pos = -1.0
    print(f"Current k: {k_pos}")


plt.connect("motion_notify_event", mouse_move)
plt.connect("key_press_event", change_k)
plt.show()
