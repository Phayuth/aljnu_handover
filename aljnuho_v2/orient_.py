import numpy as np
import matplotlib.pyplot as plt

dt = 0.02

# state (EE)
x = np.array([0.0, 0.0])
v = np.zeros(2)
theta = 0.0
omega = 0.0

# desired (mouse)
xd = np.array([1.0, 1.0])
thetad = 0.0

# gains
Kp = 5.0
Dp = 2.0
Kr = 4.0
Dr = 1.5


def wrap(a):
    return np.arctan2(np.sin(a), np.cos(a))


fig, ax = plt.subplots()
ax.set_xlim(-5, 5)
ax.set_ylim(-5, 5)
ax.set_aspect("equal")
(ee_point,) = ax.plot([], [], "bo")
(des_point,) = ax.plot([], [], "ro")
(ee_dir,) = ax.plot([], [], "b-")
(des_dir,) = ax.plot([], [], "r-")


def on_mouse(event):
    global xd
    if event.xdata is not None:
        xd = np.array([event.xdata, event.ydata])


def on_key(event):
    global thetad
    if event.key == "up":
        thetad += np.deg2rad(1)
    elif event.key == "down":
        thetad -= np.deg2rad(1)


fig.canvas.mpl_connect("motion_notify_event", on_mouse)
fig.canvas.mpl_connect("key_press_event", on_key)


def update(frame):
    global x, v, theta, omega

    # errors
    ep = x - xd
    et = wrap(theta - thetad)

    # dynamics
    a = -Kp * ep - Dp * v
    alpha = -Kr * et - Dr * omega

    # integrate
    v += a * dt
    x += v * dt

    omega += alpha * dt
    theta += omega * dt

    # draw points
    ee_point.set_data([x[0]], [x[1]])
    des_point.set_data([xd[0]], [xd[1]])

    # draw orientation lines
    L = 0.5
    ee_line = np.array([x, x + L * np.array([np.cos(theta), np.sin(theta)])])
    des_line = np.array([xd, xd + L * np.array([np.cos(thetad), np.sin(thetad)])])

    ee_dir.set_data(ee_line[:, 0], ee_line[:, 1])
    des_dir.set_data(des_line[:, 0], des_line[:, 1])

    return ee_point, des_point, ee_dir, des_dir


def loop():
    update(None)
    fig.canvas.draw_idle()


timer = fig.canvas.new_timer(interval=int(dt * 1000))
timer.add_callback(loop)
timer.start()

plt.show()
