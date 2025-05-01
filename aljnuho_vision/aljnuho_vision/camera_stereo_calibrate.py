from camera_system import StereoProcess
from ament_index_python.packages import get_package_share_directory

camleftid = 10
camrightid = 4
cam_left_preconfig = get_package_share_directory("aljnuho_vision") + "/config/" + "/left.yaml"
cam_right_preconfig = get_package_share_directory("aljnuho_vision") + "/config/" + "/right.yaml"
cam_left_postconfig = get_package_share_directory("aljnuho_vision") + "/config/" + "/streo_left.yaml"
cam_right_postconfig = get_package_share_directory("aljnuho_vision") + "/config/" + "/streo_right.yaml"

stereo = StereoProcess(camleftid, camrightid, cam_left_preconfig, cam_right_preconfig)

stereo.calibrate_stereo(cam_left_postconfig, cam_right_postconfig)

print("Stereo calibration finished")