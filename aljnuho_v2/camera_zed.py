import pyzed.sl as sl
import cv2
import numpy as np


class ZedCamera:

    def __init__(self):
        self.zed = sl.Camera()

        init_params = sl.InitParameters()
        init_params.camera_resolution = sl.RESOLUTION.HD720
        init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE
        init_params.coordinate_units = sl.UNIT.METER
        init_params.camera_fps = 60

        # Open the camera
        err = self.zed.open(init_params)
        if err > sl.ERROR_CODE.SUCCESS:
            exit(-1)

        self.zed.disable_spatial_mapping()
        self.zed.disable_body_tracking()
        self.zed.disable_positional_tracking()
        self.zed.disable_object_detection()
        self.zed.disable_recording()

        self.infoleft = {
            "height": 720,
            "width": 1280,
            "distm": "plumb_bob",
            "d": None,
            "k": None,
            "r": None,
            "p": None,
        }
        self.inforight = {
            "height": 720,
            "width": 1280,
            "distm": "plumb_bob",
            "d": None,
            "k": None,
            "r": None,
            "p": None,
        }
        self.get_camera_info()
        self.show_camera_info()
        self.info = self.infoleft  # hack to use left only for now

    def get_camera_info(self):
        cam_info = self.zed.get_camera_information()
        lc = cam_info.camera_configuration.calibration_parameters.left_cam
        rc = cam_info.camera_configuration.calibration_parameters.right_cam
        Kl = (lc.fx, 0, lc.cx, 0, lc.fy, lc.cy, 0, 0, 1)
        Kr = (rc.fx, 0, rc.cx, 0, rc.fy, rc.cy, 0, 0, 1)
        self.infoleft["k"] = np.array(Kl).reshape(3, 3)
        self.inforight["k"] = np.array(Kr).reshape(3, 3)
        self.infoleft["d"] = np.array(lc.disto)
        self.inforight["d"] = np.array(rc.disto)

        Hd = cam_info.camera_configuration.calibration_parameters.stereo_transform
        # Hdt = Hd.get_translation()
        # HdR = Hd.get_rotation_matrix()
        # HCamLeftToCamRight = np.eye(4)
        # HCamLeftToCamRight[:3, :3] = HdR
        # HCamLeftToCamRight[:3, 3] = Hdt
        print(Hd)

        # # RT matrix for C1 is identity.
        # # RT matrix for C2 is the R and T obtained from stereo calibration.
        # RT1 = np.concatenate([np.eye(3), [[0], [0], [0]]], axis=-1)
        # RT2 = HCamLeftToCamRight[0:3, :]
        # # projection matrix for C1
        # # projection matrix for C2
        # self.infoleft["p"] = self.infoleft["k"] @ RT1
        # self.inforight["p"] = self.inforight["k"] @ RT2

    def show_camera_info(self):
        print("Camera Info:")
        print("Left Camera:")
        print(f"  Resolution: {self.infoleft['width']}x{self.infoleft['height']}")
        print(f"  Distortion Model: {self.infoleft['distm']}")
        print(f"  Distortion Coefficients: {self.infoleft['d']}")
        print(f"  Intrinsic Matrix:\n{self.infoleft['k']}")
        print(f"  Projection Matrix:\n{self.infoleft['p']}")
        print("Right Camera:")
        print(
            f"  Resolution: {self.inforight['width']}x{self.inforight['height']}"
        )
        print(f"  Distortion Model: {self.inforight['distm']}")
        print(f"  Distortion Coefficients: {self.inforight['d']}")
        print(f"  Intrinsic Matrix:\n{self.inforight['k']}")
        print(f"  Projection Matrix:\n{self.inforight['p']}")

    def read(self):
        imageleft = sl.Mat()
        imageright = sl.Mat()
        if self.zed.grab() <= sl.ERROR_CODE.SUCCESS:
            self.zed.retrieve_image(imageleft, sl.VIEW.LEFT)
            self.zed.retrieve_image(imageright, sl.VIEW.RIGHT)
            return imageleft, imageright
        else:
            return None, None

    def release(self):
        self.zed.close()

    def zed_img_to_cv2(self, zedimg):
        # OpenCV drawing APIs require a writable, contiguous Mat-compatible buffer.
        if zedimg is None:
            return None
        if zedimg.ndim == 3 and zedimg.shape[2] == 4:
            zedimg = cv2.cvtColor(zedimg, cv2.COLOR_BGRA2BGR)
        zedimg = np.ascontiguousarray(zedimg, dtype=np.uint8)
        return zedimg


if __name__ == "__main__":
    zed = ZedCamera()
    while True:
        imageleft, imageright = zed.read()
        if imageleft is None or imageright is None:
            continue
        cv2.imshow("LeftImage", imageleft.get_data())
        cv2.imshow("Right Image", imageright.get_data())
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    zed.release()
    cv2.destroyAllWindows()
