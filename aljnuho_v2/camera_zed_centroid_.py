import yaml
import cv2
from ultralytics import YOLO
from center_traker import Center
import numpy as np
from multiprocessing import shared_memory
from camera_zed import ZedCamera

zedcam = ZedCamera()
modelyolo = YOLO("yolov8x-seg.pt")
ct = Center()
classidyolo = [40, 41]


yaml_path = "eye_to_hand.yaml"
with open(yaml_path, "r", encoding="utf-8") as f:
    data = yaml.safe_load(f)
HCamleftToBaselink = np.array(data["Result in Matrix form (row major)"]).reshape(
    4, 4
)
HBaselinkToCamleft = np.linalg.inv(HCamleftToBaselink)


def triangulate(point1, point2):
    if point1.dtype != "float64":
        point1 = point1.astype(np.float64)

    if point2.dtype != "float64":
        point2 = point2.astype(np.float64)

    point3d = cv2.triangulatePoints(
        zedcam.infoleft["p"],
        zedcam.inforight["p"],
        point1.reshape(2, -1),
        point2.reshape(2, -1),
        None,
    ).flatten()
    point3d /= point3d[-1]
    return point3d


def transform_point_camera_to_base(point_camera):
    point_camera_h = np.array(
        [point_camera[0], point_camera[1], point_camera[2], 1.0],
        dtype=np.float64,
    )
    point_base_h = HCamleftToBaselink @ point_camera_h
    return point_base_h[:3]


def transform_point_base_to_camera(point_base):
    point_base_h = np.array(
        [point_base[0], point_base[1], point_base[2], 1.0],
        dtype=np.float64,
    )
    point_camera_h = HBaselinkToCamleft @ point_base_h
    return point_camera_h[:3]


def project_camera_point_to_left_image(point_camera):
    z = point_camera[2]
    if z <= 1e-6:
        return None

    k = zedcam.infoleft["k"]
    uvw = k @ point_camera.reshape(3, 1)
    uv = (uvw[:2] / uvw[2]).flatten()
    return np.array([uv[0], uv[1]], dtype=np.float64)


def project_base_point_to_left_image(point_base):
    point_camera = transform_point_base_to_camera(point_base)
    return project_camera_point_to_left_image(point_camera)


def draw_base_axes_on_left_image(image, axis_len_m=0.10):
    base_origin = np.array([0.0, 0.0, 0.0], dtype=np.float64)
    base_x = np.array([axis_len_m, 0.0, 0.0], dtype=np.float64)
    base_y = np.array([0.0, axis_len_m, 0.0], dtype=np.float64)
    base_z = np.array([0.0, 0.0, axis_len_m], dtype=np.float64)

    cam_origin = transform_point_base_to_camera(base_origin)

    pix_o = project_base_point_to_left_image(base_origin)
    pix_x = project_base_point_to_left_image(base_x)
    pix_y = project_base_point_to_left_image(base_y)
    pix_z = project_base_point_to_left_image(base_z)

    if pix_o is None:
        cv2.putText(
            image,
            "Base origin behind camera",
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 0, 255),
            2,
            cv2.LINE_AA,
        )
        return

    cv2.putText(
        image,
        f"base in cam: [{cam_origin[0]:.3f}, {cam_origin[1]:.3f}, {cam_origin[2]:.3f}] m",
        (20, 40),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (255, 255, 255),
        2,
        cv2.LINE_AA,
    )

    o = tuple(np.round(pix_o).astype(int))
    cv2.circle(image, o, 5, (255, 255, 255), -1)
    cv2.putText(
        image,
        "base",
        (o[0] + 8, o[1] - 8),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (255, 255, 255),
        2,
        cv2.LINE_AA,
    )

    if pix_x is not None:
        x = tuple(np.round(pix_x).astype(int))
        cv2.arrowedLine(image, o, x, (0, 0, 255), 2, tipLength=0.18)
        cv2.putText(
            image,
            "X",
            (x[0] + 6, x[1]),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 0, 255),
            2,
            cv2.LINE_AA,
        )

    if pix_y is not None:
        y = tuple(np.round(pix_y).astype(int))
        cv2.arrowedLine(image, o, y, (0, 255, 0), 2, tipLength=0.18)
        cv2.putText(
            image,
            "Y",
            (y[0] + 6, y[1]),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )

    if pix_z is not None:
        z = tuple(np.round(pix_z).astype(int))
        cv2.arrowedLine(image, o, z, (255, 0, 0), 2, tipLength=0.18)
        cv2.putText(
            image,
            "Z",
            (z[0] + 6, z[1]),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 0, 0),
            2,
            cv2.LINE_AA,
        )


# create shared memory
shm = shared_memory.SharedMemory(
    name="camera_zed_centroid", create=True, size=8 * 4
)  # 4 float64: x, y, z, status
pointscentroid = np.ndarray((4,), dtype=np.float64, buffer=shm.buf)
# Status convention: 1.0 = updating, 0.0 = stuck or no fresh update.
pointscentroid[:] = [np.inf, np.inf, np.inf, 0.0]

prev_point_base = None
update_epsilon = 1e-4
print("SHM name:", shm.name)  # pass this to other script

# Hplace
place_cam_left_px = (665, 593)
place_cam_right_px = (602, 594)
point_place_3d = triangulate(
    np.array(place_cam_left_px, dtype=np.float64),
    np.array(place_cam_right_px, dtype=np.float64),
)
point_place_3d_inbase = transform_point_camera_to_base(point_place_3d[:3])
print(
    f"Place point in base frame: [{point_place_3d_inbase[0]:.3f}, {point_place_3d_inbase[1]:.3f}, {point_place_3d_inbase[2]:.3f}] m"
)
# [0.149, -0.660, -0.027] m

try:
    while True:
        imgl, imgr = zedcam.read()
        if imgl is None or imgr is None:
            pointscentroid[3] = 0.0
            continue

        left_bgr = zedcam.zed_img_to_cv2(imgl.get_data())
        right_bgr = zedcam.zed_img_to_cv2(imgr.get_data())
        if left_bgr is None or right_bgr is None:
            pointscentroid[3] = 0.0
            continue

        mean_left, mean_right, left_mask, right_mask = ct.trackedCenterShow(
            modelyolo, classidyolo, left_bgr, right_bgr
        )
        draw_base_axes_on_left_image(left_bgr, axis_len_m=0.10)

        if (mean_left is not None) and (mean_right is not None):
            point3d = triangulate(np.array(mean_left), np.array(mean_right))
            point3dinbase = transform_point_camera_to_base(point3d[:3])
            if prev_point_base is None:
                is_updating = 1.0
            else:
                moved = np.linalg.norm(point3dinbase - prev_point_base)
                is_updating = 1.0 if moved > update_epsilon else 0.0

            # point3dinbase[0] -= 0.025  # 5cm offset in x in base frame
            pointscentroid[:3] = point3dinbase  # write xyz to shared memory
            pointscentroid[3] = is_updating  # write status to shared memory
            prev_point_base = point3dinbase.copy()
        else:
            pointscentroid[3] = 0.0

        print(
            f"Centroid in base frame: [{pointscentroid[0]:.3f}, {pointscentroid[1]:.3f}, {pointscentroid[2]:.3f}] m)"
        )

        cv2.circle(left_bgr, place_cam_left_px, 5, (255, 0, 0), -1)
        cv2.imshow("left", left_bgr)
        cv2.imshow("right", right_bgr)
        if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
            break
except KeyboardInterrupt:
    shm.close()
    shm.unlink()
finally:
    print("Cleaning up shared memory...")
