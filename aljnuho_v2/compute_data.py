import numpy as np
import pandas as pd
np.set_printoptions(precision=3, suppress=True)
HdeliveryToBase = np.array([
    [1.0, 0.0, 0.0, 0.149],
    [0.0, 1.0, 0.0, -0.660],
    [0.0, 0.0, 1.0, -0.027],
    [0.0, 0.0, 0.0, 1.0]
])

HdeliveryToBase_mm = HdeliveryToBase.copy()
HdeliveryToBase_mm[0:3, 3] *= 1000.0
print(f"==>> HdeliveryToBase_mm: \n{HdeliveryToBase_mm}")

df = pd.read_csv("aljnuho_v2/rgmc-icra2026.csv", header=0)
print(f"==>> df: \n{df}")

delivery_location_est_x_mm = df["delivery_location_est_x_mm"]
print(f"==>> delivery_location_est_x_mm: \n{delivery_location_est_x_mm}")
delivery_location_est_y_mm = df["delivery_location_est_y_mm"]
print(f"==>> delivery_location_est_y_mm: \n{delivery_location_est_y_mm}")
delivery_location_est_z_mm = df["delivery_location_est_z_mm"]
print(f"==>> delivery_location_est_z_mm: \n{delivery_location_est_z_mm}")

xyz_est_mm = np.stack((delivery_location_est_x_mm, delivery_location_est_y_mm, delivery_location_est_z_mm), axis=1)
# print(f"==>> xyz_est_mm: \n{xyz_est_mm}")

results = []

for i in range(xyz_est_mm.shape[0]):
    xyzi = xyz_est_mm[i]
    xyzi_str = np.asarray(xyzi, dtype=str)

    is_fail = False
    pxyz_in_base_mm = None
    if np.any(xyzi_str == "X"):
        is_fail = True
        results.append({"idx": i + 1, "is_fail": is_fail, "px_mm": -1.0, "py_mm": -1.0, "pz_mm": -1.0})
        print(f"idx: {i+1}, is_fail: {is_fail}, pxyz_in_base_mm: {pxyz_in_base_mm}")
        continue

    pxyz_in_delivery_mm = xyzi_str.astype(float)
    pxyz_in_delivery_mm = np.hstack((pxyz_in_delivery_mm, np.array([1.0])))
    pxyz_in_base_mm = HdeliveryToBase_mm @ pxyz_in_delivery_mm
    pxyz = pxyz_in_base_mm[:3]
    results.append({"idx": i + 1, "is_fail": is_fail, "px_mm": pxyz[0], "py_mm": pxyz[1], "pz_mm": pxyz[2]})

    print(f"idx: {i+1}, is_fail: {is_fail}, pxyz_in_base_mm: {pxyz_in_base_mm}")

output_csv = "aljnuho_v2/compute_data_output.csv"
result_df = pd.DataFrame(results, columns=["idx", "is_fail", "px_mm", "py_mm", "pz_mm"])
result_df.to_csv(output_csv, index=False)
print(f"==>> saved: {output_csv}")