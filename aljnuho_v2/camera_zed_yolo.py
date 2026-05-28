from ultralytics import YOLO
from center_traker import Center
import numpy as np
from camera_zed import ZedCamera
import cv2

model = YOLO("yolov8x-seg.pt")
ct = Center()
zedcam = ZedCamera()
classid = [40, 41]

while True:
    imgl, imgr = zedcam.read()
    if imgl is None or imgr is None:
        continue

    left_bgr = zedcam.zed_img_to_cv2(imgl.get_data())
    right_bgr = zedcam.zed_img_to_cv2(imgr.get_data())
    if left_bgr is None or right_bgr is None:
        continue

    ct.trackedCenterShow(model, classid, left_bgr, right_bgr)

    cv2.imshow("left", left_bgr)
    cv2.imshow("right", right_bgr)

    if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
        break
