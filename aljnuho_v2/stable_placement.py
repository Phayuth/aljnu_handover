import numpy as np
import matplotlib.pyplot as plt
from pytransform3d.transformations import plot_transform
from pytransform3d.plot_utils import make_3d_axis

np.random.seed(42)
# give a point cloud, find a stable point to place on a table 2d plane


def generate_table_points(num_points=100, table_size=1.0):
    # generate random points on a table plane
    points = np.random.rand(num_points, 2) * table_size - table_size / 2
    points = np.hstack((points, np.zeros((num_points, 1))))  # add z=0
    return points


def detect_2d_plane(points):
    # plane equation in 3d is ax + by + cz + d = 0, for table plane we can assume c=1 and d=0
    # so we just need to find a and b that best fit the points
    A = np.hstack((points[:, :2], np.ones((points.shape[0], 1))))
    B = points[:, 2]
    coeffs, _, _, _ = np.linalg.lstsq(A, B, rcond=None)
    a, b, d = coeffs
    return a, b, d


def generate_cube_pointcloud(center, size, num_points=500):
    # generate random points on the surface of a cube
    points = []
    for _ in range(num_points):
        face = np.random.randint(6)
        if face == 0:  # front
            point = center + np.array(
                [np.random.rand() * size, np.random.rand() * size, 0]
            )
        elif face == 1:  # back
            point = center + np.array(
                [np.random.rand() * size, np.random.rand() * size, size]
            )
        elif face == 2:  # left
            point = center + np.array(
                [0, np.random.rand() * size, np.random.rand() * size]
            )
        elif face == 3:  # right
            point = center + np.array(
                [size, np.random.rand() * size, np.random.rand() * size]
            )
        elif face == 4:  # top
            point = center + np.array(
                [np.random.rand() * size, size, np.random.rand() * size]
            )
        else:  # bottom
            point = center + np.array(
                [np.random.rand() * size, 0, np.random.rand() * size]
            )
        points.append(point)
    return np.array(points)


def cube_pointcloud_normal_vector_estimation(points):
    """Estimate a unit normal vector for each point in a point cloud.

    Method:
    1) For each point, find k-nearest neighbors (brute force with NumPy).
    2) Fit a local plane by PCA (smallest covariance eigenvector = normal).
    3) Orient normal consistently outward from cloud centroid.
    """
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("points must have shape (N, 3)")

    n_points = points.shape[0]
    if n_points < 4:
        raise ValueError("at least 4 points are required to estimate normals")

    k = min(20, n_points - 1)
    normals = np.zeros_like(points)
    cloud_center = np.mean(points, axis=0)

    # Pairwise distances for neighbor lookup.
    diff = points[:, None, :] - points[None, :, :]
    dists = np.linalg.norm(diff, axis=2)

    for i in range(n_points):
        # Skip index 0 (the point itself, distance == 0).
        nn_idx = np.argsort(dists[i])[1 : k + 1]
        neighborhood = points[nn_idx]

        local_center = np.mean(neighborhood, axis=0)
        centered = neighborhood - local_center
        cov = centered.T @ centered / max(centered.shape[0] - 1, 1)

        eigvals, eigvecs = np.linalg.eigh(cov)
        normal = eigvecs[:, np.argmin(eigvals)]

        # Keep a consistent outward direction for cube-like objects.
        if np.dot(normal, points[i] - cloud_center) < 0:
            normal = -normal

        normals[i] = normal / np.linalg.norm(normal)

    return normals



def detect_stable_plane_on_pointcloud(points):
    pass


table_points = generate_table_points()
a, b, d = detect_2d_plane(table_points)
xx, yy = np.meshgrid(np.linspace(-0.5, 0.5, 10), np.linspace(-0.5, 0.5, 10))
zz = a * xx + b * yy + d

center = np.array([0.0, 0.0, 0.5])
size = 0.2
cube_pc = generate_cube_pointcloud(center=center, size=size)
cube_normals = cube_pointcloud_normal_vector_estimation(cube_pc)

ax = make_3d_axis(1)
plot_transform(ax, np.eye(4), s=0.5, name="world")
ax.scatter(
    table_points[:, 0],
    table_points[:, 1],
    0,
    c="blue",
    label="table points",
)
ax.scatter(
    cube_pc[:, 0],
    cube_pc[:, 1],
    cube_pc[:, 2],
    c="red",
    label="cube points",
)

ax.quiver(
    cube_pc[:, 0],
    cube_pc[:, 1],
    cube_pc[:, 2],
    cube_normals[:, 0],
    cube_normals[:, 1],
    cube_normals[:, 2],
    length=0.03,
    linewidth=1,
    color="black",
    normalize=True,
    label="cube normals vectors",
)

ax.plot_surface(xx, yy, zz, alpha=0.5, color="cyan", label="detected plane")
ax.set_title("Stable Placement on Table")
ax.legend()
plt.show()
