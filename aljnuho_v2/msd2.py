import numpy as np
import matplotlib.pyplot as plt

# parameters
m = 1.0
k = 50.0
c = 8.0
dt = 0.01
k_wall = 50.0
c_wall = 5.0
# state
x = np.array([0.5, 0.5])  # mass position
v = np.array([0.0, 0.0])  # velocity
anchor = np.array([0.0, 0.0])  # mouse-controlled point
wall = np.array([0.0, 0.0])

# figure
fig, ax = plt.subplots()
ax.set_xlim(-2, 2)
ax.set_ylim(-2, 2)
ax.set_aspect("equal")
(mass_plot,) = ax.plot([], [], "bo", markersize=10, label="mass")
(anchor_plot,) = ax.plot([], [], "ro", markersize=8, label="anchor")
(spring_line,) = ax.plot([], [], "k-", label="spring")
(wall_plot,) = ax.plot([], [], "go", markersize=8)  # wall point
(wall_line,) = ax.plot([], [], "g--")  # spring to wall
ax.legend()


def on_mouse_move(event):
    global anchor
    if event.xdata is not None and event.ydata is not None:
        anchor = np.array([event.xdata, event.ydata])


def change_k(event):
    global k
    print(f"Key pressed: {event.key}")
    if event.key == "up":
        k += 1.0
        ax.set_title(f"Stiffness: {k}, attract to object")
    elif event.key == "down":
        k -= 1.0
        ax.set_title(f"Stiffness: {k}, repel from object")
    elif event.key == "right":
        k = 0.0
        ax.set_title(f"Stiffness: {k}, no movement")
    elif event.key == "left":
        k = 50.0
        ax.set_title(f"Stiffness: {k}, default")
    elif event.key == " ":
        k = 2*k_wall
        ax.set_title(f"Stiffness: {k}, wall spring")
    plt.draw()


fig.canvas.mpl_connect("motion_notify_event", on_mouse_move)
fig.canvas.mpl_connect("key_press_event", change_k)


def update():
    global x, v

    # spring-damper force toward anchor
    F = -k * (x - anchor) - c * v
    F += -k_wall * (x - wall) - c_wall * (v - 0)  # wall spring and damper
    # integrate (semi-implicit Euler)
    v += (F / m) * dt
    x += v * dt

    # update plot
    mass_plot.set_data([x[0]], [x[1]])
    anchor_plot.set_data([anchor[0]], [anchor[1]])
    spring_line.set_data([x[0], anchor[0]], [x[1], anchor[1]])
    wall_plot.set_data([wall[0]], [wall[1]])
    wall_line.set_data([x[0], wall[0]], [x[1], wall[1]])
    fig.canvas.draw_idle()


# simple loop
timer = fig.canvas.new_timer(interval=int(dt * 1000))
timer.add_callback(update)
timer.start()

plt.show()
