import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Circle

# system params
m = 1.0
# c = 0.5
k_pos = 2.0  # normal spring
k_neg = -1.0  # negative stiffness
dt = 0.02
c = 2.2 * np.sqrt(k_pos * m)

# initial state
x_pos, vx_pos = 0.0, 0.0
y_pos, vy_pos = 0.0, 0.0
xobj = 0.0
yobj = 0.0
r_obj_to_pos = np.linalg.norm([xobj - x_pos, yobj - y_pos])
r_reaching = 1.0


def step_x(x, v, k):
    # spring-damper force: F = -k(x-xw) - c*v
    a = (-k * (x - xobj) - c * v) / m
    v = v + a * dt
    x = x + v * dt
    return x, v


def step_y(y, v, k):
    # spring-damper force: F = -k(y-yw) - c*v
    a = (-k * (y - yobj) - c * v) / m
    v = v + a * dt
    y = y + v * dt
    return y, v


fig, ax = plt.subplots()
ax.set_xlim(-2, 2)
ax.set_ylim(-0.5, 0.5)
creaching = Circle((0, 0), r_reaching, color="green", fill=False, linestyle="--", label="reaching radius")
ax.add_patch(creaching)
cir = Circle((xobj, yobj), r_obj_to_pos, color="blue", fill=False)
ax.add_patch(cir)
ax.axhline(0, color="gray", linestyle="--")
ax.axvline(0, color="gray", linestyle="--")
(pt_pos,) = ax.plot([], [], "o", label="robot ee")
(obj_pos,) = ax.plot([], [], "x", color="red", label="object")


def mouse_move(event):
    global x_pos, vx_pos, k_pos, xobj, yobj, y_pos, vy_pos

    x, y = event.xdata, event.ydata
    if x is not None and y is not None:
        xobj = x
        yobj = y
        r_obj_to_pos = np.linalg.norm([xobj - x_pos, yobj - y_pos])
        cir.set_center((xobj, yobj))
        cir.set_radius(r_obj_to_pos)

        x_pos, vx_pos = step_x(x_pos, vx_pos, k_pos)
        y_pos, vy_pos = step_y(y_pos, vy_pos, k_pos)

        # limit the position to be within the reaching radius
        X = np.array([x_pos, y_pos])
        if np.linalg.norm(X) >= r_reaching:
            x_pos, y_pos = (
                x_pos / np.linalg.norm(X) * r_reaching,
                y_pos / np.linalg.norm(X) * r_reaching,
            )
            vx_pos, vy_pos = 0.0, 0.0
        pt_pos.set_data([x_pos], [y_pos])
        obj_pos.set_data([xobj], [yobj])
        plt.draw()


def change_k(event):
    global k_pos
    if event.key == "up":
        k_pos = 2.0
        ax.set_title(f"Stiffness: {k_pos}, attract to object")
    elif event.key == "down":
        k_pos = -1.0
        ax.set_title(f"Stiffness: {k_pos}, repel from object")
    elif event.key == "right":
        k_pos = 0.0
        ax.set_title(f"Stiffness: {k_pos}, no movement")

    plt.draw()


ax.set_xlim(-1.0, 3.0)
ax.set_ylim(-2.0, 2.0)
ax.set_aspect("equal")
ax.legend(loc="upper right", bbox_to_anchor=(1.15, 1))
plt.connect("motion_notify_event", mouse_move)
plt.connect("key_press_event", change_k)
plt.show()
