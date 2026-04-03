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
(wall_to_mass_line,) = ax.plot([], [], "g-")  # line from wall to mass

rreach = 1.0
# adaptive stiffness limits
k_base = 50.0
k_peak = 130.0
k_out_min = 8.0
k_wall_base = 50.0
k_wall_soft = 6.0
k_wall_hard = 220.0

creach = plt.Circle(
    (0, 0),
    rreach,
    color="cyan",
    fill=False,
    linestyle="--",
    label="reaching radius",
)
ax.add_patch(creach)
ax.legend()


def on_mouse_move(event):
    global anchor
    if event.xdata is not None and event.ydata is not None:
        anchor = np.array([event.xdata, event.ydata])


def change_k(event):
    global k, k_wall
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
        k += 10.0
        k_wall -= 10.0
        ax.set_title(f"Stiffness: {k}, wall spring")
    elif event.key == "m":
        k -= 10.0
        k_wall += 10.0
        ax.set_title(f"Stiffness: {k}, wall repulsion")
    plt.draw()


def update():
    global x, v, anchor, wall, k, k_wall

    dist_wall_to_anchor = np.linalg.norm(wall - anchor)

    # Adapt gains from the wall-anchor distance:
    # near boundary from inside -> stronger anchor pull, softer wall spring
    # outside boundary -> weaker anchor pull, stronger wall spring
    if dist_wall_to_anchor < rreach:
        t_in = np.clip(dist_wall_to_anchor / rreach, 0.0, 1.0)
        t_in = t_in * t_in
        k = k_base + (k_peak - k_base) * t_in
        k_wall = (k_wall_base + (k_wall_soft - k_wall_base) * t_in) * 0.1
        ax.set_title(
            f"Inside reach | d={dist_wall_to_anchor:.2f}, k={k:.1f}, k_wall={k_wall:.1f}"
        )
    else:
        t_out = np.clip((dist_wall_to_anchor - rreach) / rreach, 0.0, 1.0)
        k = k_peak + (k_out_min - k_peak) * t_out
        k_wall = (k_wall_soft + (k_wall_hard - k_wall_soft) * t_out) * 0.1
        ax.set_title(
            f"Outside reach | d={dist_wall_to_anchor:.2f}, k={k:.1f}, k_wall={k_wall:.1f}"
        )

    # spring-damper force toward anchor + wall spring and damper
    F = (-k * (x - anchor) - c * v) + (-k_wall * (x - wall) - c_wall * (v - 0))
    # integrate (semi-implicit Euler)
    v += (F / m) * dt
    x += v * dt

    # Hard constraint: keep the mass inside reaching radius.
    dist_wall_to_mass = np.linalg.norm(x - wall)
    if dist_wall_to_mass > rreach:
        n = (x - wall) / dist_wall_to_mass
        x = wall + n * rreach
        v_out = np.dot(v, n)
        if v_out > 0.0:
            v = v - v_out * n

    # update plot
    mass_plot.set_data([x[0]], [x[1]])
    anchor_plot.set_data([anchor[0]], [anchor[1]])
    spring_line.set_data([x[0], anchor[0]], [x[1], anchor[1]])
    wall_plot.set_data([wall[0]], [wall[1]])
    wall_line.set_data([x[0], wall[0]], [x[1], wall[1]])
    wall_to_mass_line.set_data([wall[0], anchor[0]], [wall[1], anchor[1]])
    fig.canvas.draw_idle()


fig.canvas.mpl_connect("motion_notify_event", on_mouse_move)
fig.canvas.mpl_connect("key_press_event", change_k)
timer = fig.canvas.new_timer(interval=int(dt * 1000))
timer.add_callback(update)
timer.start()
plt.show()
