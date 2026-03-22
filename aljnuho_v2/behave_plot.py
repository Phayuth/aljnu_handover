import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle

# model multiple points racing toward the grasp center
# if the object is moving. put the grasp back abit

rcx = 0.0
rcy = 0.0
rr = 1.0

objx = 0.0
objy = 0.0

rechx = 0.0
rechy = 0.0


def mouse_move(event):
    x, y = event.xdata, event.ydata
    if x is not None and y is not None:
        objx, objy = x, y
        line.set_data([rcx, objx], [rcy, objy])
        cobj.set_center((objx, objy))
        cobj.set_radius(np.hypot(objx - rcx, objy - rcy))
        crech.set_center((objx, objy))
        plt.draw()


fig, ax = plt.subplots()

(line,) = ax.plot([rcx, objx], [rcy, objy], "ro--")
plt.connect("motion_notify_event", mouse_move)
c = Circle((rcx, rcy), rr, fill=False)
ax.add_patch(c)
cobj = Circle((objx, objy), 0.1, color="blue", fill=False)
ax.add_patch(cobj)
crech = Circle((rechx, rechy), 0.3, color="green", fill=False)
ax.add_patch(crech)
ax.set_xlim(-2.0, 2.0)
ax.set_ylim(-2.0, 2.0)
ax.set_aspect("equal")
plt.show()
