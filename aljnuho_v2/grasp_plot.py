import numpy as np
import matplotlib.pyplot as plt
from pytransform3d.transformations import plot_transform
from pytransform3d.plot_utils import make_3d_axis
from sklearn.neighbors import NearestNeighbors
import trimesh
from scipy.spatial import ConvexHull


def plot_parallel_gripper(
    ax,
    Hgrasp,
    finger_gap=0.08,
    finger_length=0.12,
    jaw_depth=0.02,
):
    """Plot a simple 2-finger parallel gripper.
    Finger direction: +Z axis of gripper frame.
    Opening/closing direction: X axis of gripper frame.
    """

    def transform_points(Hgrasp, points_local):
        """Transform Nx3 points from gripper frame to world frame."""
        points_h = np.hstack([points_local, np.ones((points_local.shape[0], 1))])
        return (Hgrasp @ points_h.T).T[:, :3]

    half_gap = 0.5 * finger_gap

    # Finger center lines in local frame.
    left_finger_local = np.array(
        [[-half_gap, 0.0, 0.0], [-half_gap, 0.0, finger_length]]
    )
    right_finger_local = np.array(
        [[half_gap, 0.0, 0.0], [half_gap, 0.0, finger_length]]
    )

    # Palm bar at z = 0 linking both fingers.
    palm_local = np.array([[-half_gap, 0.0, 0.0], [half_gap, 0.0, 0.0]])

    # Small jaw-depth lines make each finger easier to see in 3D.
    left_depth_local = np.array(
        [[-half_gap, -jaw_depth, 0.0], [-half_gap, jaw_depth, 0.0]]
    )
    right_depth_local = np.array(
        [[half_gap, -jaw_depth, 0.0], [half_gap, jaw_depth, 0.0]]
    )

    left_finger = transform_points(Hgrasp, left_finger_local)
    right_finger = transform_points(Hgrasp, right_finger_local)
    palm = transform_points(Hgrasp, palm_local)
    left_depth = transform_points(Hgrasp, left_depth_local)
    right_depth = transform_points(Hgrasp, right_depth_local)

    ax.plot(
        left_finger[:, 0],
        left_finger[:, 1],
        left_finger[:, 2],
        "r-",
        linewidth=3,
    )
    ax.plot(
        right_finger[:, 0],
        right_finger[:, 1],
        right_finger[:, 2],
        "r-",
        linewidth=3,
    )
    ax.plot(
        palm[:, 0],
        palm[:, 1],
        palm[:, 2],
        "k-",
        linewidth=2,
    )
    ax.plot(
        left_depth[:, 0],
        left_depth[:, 1],
        left_depth[:, 2],
        "k-",
        linewidth=2,
    )
    ax.plot(
        right_depth[:, 0],
        right_depth[:, 1],
        right_depth[:, 2],
        "k-",
        linewidth=2,
    )


def generate_cylinder_points(radius=0.02, height=0.1, num_points=20):
    """Generate points on the surface of a cylinder for visualization."""
    angles = np.linspace(0, 2 * np.pi, num_points)
    z = np.linspace(0, height, num_points)
    points = []
    for angle in angles:
        for zi in z:
            x = radius * np.cos(angle)
            y = radius * np.sin(angle)
            points.append([x, y, zi])
    return np.array(points)


def estimate_normals(points, k=20):
    nbrs = NearestNeighbors(n_neighbors=k).fit(points)
    normals = np.zeros_like(points)
    for i, p in enumerate(points):
        idx = nbrs.kneighbors([p], return_distance=False)[0]
        neigh = points[idx]
        # center
        neigh = neigh - neigh.mean(axis=0)
        # covariance
        C = neigh.T @ neigh
        # eigenvectors
        _, _, vh = np.linalg.svd(C)
        n = vh[-1]  # smallest direction
        normals[i] = n / np.linalg.norm(n)
    return normals


def orient_normals_propagation(points, normals, k=10):
    nbrs = NearestNeighbors(n_neighbors=k).fit(points)
    oriented = normals.copy()
    visited = set([0])
    stack = [0]

    while stack:
        i = stack.pop()
        neighbors = nbrs.kneighbors([points[i]], return_distance=False)[0]

        for j in neighbors:
            if j in visited:
                continue
            if np.dot(oriented[i], oriented[j]) < 0:
                oriented[j] *= -1
            visited.add(j)
            stack.append(j)

    return oriented


def find_antipodal_pairs(points, normals, w_min=0.02, w_max=0.08):
    candidates = []
    candidates_normals = []
    candidates_dist = []
    N = len(points)
    for i in range(N):
        for j in range(i + 1, N):
            p, q = points[i], points[j]
            n_p, n_q = normals[i], normals[j]

            dvec = q - p
            dist = np.linalg.norm(dvec)

            # 1. width check
            if dist < w_min or dist > w_max:
                continue

            x = dvec / dist

            # 2. opposing normals
            if np.dot(n_p, n_q) > -0.8:
                continue

            # 3. alignment
            if abs(np.dot(n_p, x)) < 0.8:
                continue
            if abs(np.dot(n_q, -x)) < 0.8:
                continue

            candidates.append((p, q))
            candidates_normals.append((n_p, n_q))
            candidates_dist.append(dist)

    return candidates, candidates_dist, candidates_normals


def score_grasp(p, q, n_p, n_q, center, w_center=1.0, w_align=2.0, w_width=0.5):
    """
    p, q   : contact points
    n_p,n_q: inward normals
    center : object centroid
    """

    # closing direction
    d = q - p
    dist = np.linalg.norm(d)
    if dist < 1e-6:
        return -np.inf
    x = d / dist

    # 1. alignment (antipodal quality)
    s_align = abs(np.dot(n_p, x)) + abs(np.dot(n_q, -x))

    # 2. centering (torque minimization)
    mid = 0.5 * (p + q)
    s_center = -np.linalg.norm(mid - center)

    # 3. width preference (slightly favor larger grasps)
    s_width = dist

    # final score
    score = w_align * s_align + w_center * s_center + w_width * s_width

    return score


def score_all_candidates(candidates, candidates_normals, center):
    scores = []
    for (p, q), (n_p, n_q) in zip(candidates, candidates_normals):
        s = score_grasp(p, q, n_p, n_q, center)
        scores.append(s)
    print(f"Candidate scores: {scores}")
    return scores


def sort_candidates_by_score(candidates, candidates_normals, scores):
    sorted_indices = np.argsort(scores)[::-1]  # Descending order
    sorted_candidates = [candidates[i] for i in sorted_indices]
    sorted_normals = [candidates_normals[i] for i in sorted_indices]
    sorted_scores = [scores[i] for i in sorted_indices]
    return sorted_candidates, sorted_normals, sorted_scores


point3d_cylinder_world_frame = generate_cylinder_points()
cylinder_ch = ConvexHull(point3d_cylinder_world_frame)
cylinder_centroid = cylinder_ch.points[cylinder_ch.vertices].mean(axis=0)
print(f"==>> cylinder_centroid: \n{cylinder_centroid}")

normals = estimate_normals(point3d_cylinder_world_frame)
normals = orient_normals_propagation(point3d_cylinder_world_frame, normals)
# here i only get pinch pairs, not full grasp pose yet
candidates, candidates_dist, candidates_normals = find_antipodal_pairs(
    point3d_cylinder_world_frame, normals
)
print(f"Found {len(candidates)} antipodal pairs.")

scores = score_all_candidates(candidates, candidates_normals, cylinder_centroid)
sorted_candidates, sorted_normals, sorted_scores = sort_candidates_by_score(
    candidates, candidates_normals, scores
)
print("Top 5 candidates:")
for i in range(min(5, len(sorted_candidates))):
    p, q = sorted_candidates[i]
    n_p, n_q = sorted_normals[i]
    s = sorted_scores[i]
    print(f"Candidate {i+1}:")
    print(f"  P1: {p}, Normal: {n_p}")
    print(f"  P2: {q}, Normal: {n_q}")
    print(f"  Score: {s:.4f}")


def create_normal_arrow(origin, normal, length=0.1, radius=0.01):
    # 1. Create default arrow pointing up (0,0,1)
    arrow = trimesh.creation.cylinder(radius=radius, height=length)

    # 2. Align vector from default (0,0,1) to target normal
    # The cylinder is originally centered, so move it up
    arrow.apply_translation([0, 0, length / 2])

    # 3. Rotate and translate into place
    rot = trimesh.geometry.align_vectors([0, 0, 1], normal)
    arrow.apply_transform(rot)
    arrow.apply_translation(origin)

    return arrow


def grasp_pose_robotiq(p, q, n_p, n_q):
    p, q = np.asarray(p), np.asarray(q)
    n_p, n_q = np.asarray(n_p), np.asarray(n_q)

    # x: closing
    x = q - p
    x = x / np.linalg.norm(x)

    # z: approach
    # normals point inward
    # z follow pointing inward
    z = -(n_p + n_q)
    if np.linalg.norm(z) < 1e-6:
        z = np.array([0, 0, 1])
        if abs(np.dot(z, x)) > 0.9:
            z = np.array([1, 0, 0])

    z = z - np.dot(z, x) * x
    z = z / np.linalg.norm(z)

    # y
    y = np.cross(z, x)
    y = y / np.linalg.norm(y)

    # fix z
    z = np.cross(x, y)

    # rotation
    R = np.column_stack((x, y, z))

    # translation at the midpoint
    # this place palm center, which incorrect
    # we need to shift back along negative z by some offset (e.g. 0.02m) to place the wrist center
    t = 0.5 * (p + q)

    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t

    return T


def correct_grasp_pose(Hgrasp, finger_length=0.088):
    # Shift back along negative z by finger_length to place wrist center
    shift = np.eye(4)
    shift[2, 3] = -finger_length
    return Hgrasp @ shift


p, q = sorted_candidates[i]
n_p, n_q = sorted_normals[i]
Hgrasp = grasp_pose_robotiq(p, q, n_p, n_q)
Hgrasp = correct_grasp_pose(Hgrasp)


# scene setup
scene = trimesh.Scene()
axis = trimesh.creation.axis(origin_size=0.001, axis_length=0.5)
box = trimesh.creation.box(extents=(1, 1, 1))
box.visual.face_colors = [100, 150, 255, 40]
scene.add_geometry(box)
scene.add_geometry(axis)
pc = trimesh.PointCloud(point3d_cylinder_world_frame, colors=[0, 0, 255])
scene.add_geometry(pc)
for i in range(len(point3d_cylinder_world_frame)):
    arrow = create_normal_arrow(
        point3d_cylinder_world_frame[i], normals[i], length=0.005, radius=0.00005
    )
    scene.add_geometry(arrow)

for i in range(min(10, len(sorted_candidates))):
    p, q = sorted_candidates[i]
    n_p, n_q = sorted_normals[i]
    s = sorted_scores[i]
    print(f"Candidate {i+1}:")
    print(f"  P1: {p}, Normal: {n_p}")
    print(f"  P2: {q}, Normal: {n_q}")
    print(f"  Score: {s:.4f}")
    vertcolor = np.array([[255, 0, 0, 255]], dtype=np.uint8)
    path = trimesh.load_path([p, q])
    scene.add_geometry(path)

grsp = trimesh.creation.axis(origin_size=0.001, axis_length=0.1)
grsp.apply_transform(Hgrasp)
scene.add_geometry(grsp)
scene.show(line_settings={"point_size": 5})


ax = make_3d_axis(1.0, unit="m")
plot_transform(ax, np.eye(4), s=0.1, name="world")
ax.scatter(
    point3d_cylinder_world_frame[:, 0],
    point3d_cylinder_world_frame[:, 1],
    point3d_cylinder_world_frame[:, 2],
    c="b",
    s=5,
)
plot_parallel_gripper(ax, Hgrasp)
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")
ax.set_box_aspect([1.0, 1.0, 1.0])
plt.show()
