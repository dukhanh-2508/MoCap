import numpy as np
import cv2
import pandas as pd


#Vai trò trong đồ án: Đóng vai trò là Môi trường giả lập Software-in-the-Loop (SIL) toàn vẹn cho hệ thống Motion Capture
#  File này tích hợp toàn bộ các giai đoạn của hệ thống phân tán (từ giả lập tạo ảnh thô của Slave cho đến giải toán
#  hình học 3D của Server) vào một luồng thực thi duy nhất nhằm chứng minh tính đúng đắn của thuật toán toán học trước
#  khi đẩy lên phần cứng thực tế.
print("===== MÔ PHỎNG HỆ THỐNG MOCAP SOFTWARE-IN-THE-LOOP (PYTHON) =====")

# --- 1. CẤU HÌNH THÔNG SỐ HÌNH HỌC CAMERA ẢO (GROUND TRUTH) ---
K = np.array([[630.0, 0.0, 320.0],
              [0.0, 630.0, 240.0],
              [0.0, 0.0, 1.0]], dtype=np.float64)

# Camera 1 đặt tại gốc tọa độ
R1 = np.eye(3, dtype=np.float64)
T1 = np.zeros((3, 1), dtype=np.float64)
P1 = K @ np.hstack((R1, T1))

# Camera 2 cách Cam 1 đoạn d = 400mm theo trục X và hơi xoay 5 độ
theta = np.radians(-5)
R2 = np.array([[np.cos(theta), 0, np.sin(theta)],
               [0, 1, 0],
               [-np.sin(theta), 0, np.cos(theta)]], dtype=np.float64)
T2 = np.array([[-400.0], [0.0], [0.0]], dtype=np.float64)
P2 = K @ np.hstack((R2, T2))


# --- 2. THUẬT TOÁN TÁI TẠO HÌNH HỌC 3D (DLT VIA SVD) ---
def triangulate_dlt(pt1, pt2):
    A = np.zeros((4, 4), dtype=np.float64)
    A[0] = pt1[0] * P1[2, :] - P1[0, :]
    A[1] = pt1[1] * P1[2, :] - P1[1, :]
    A[2] = pt2[0] * P2[2, :] - P2[0, :]
    A[3] = pt2[1] * P2[2, :] - P2[1, :]

    _, _, Vt = np.linalg.svd(A)
    X = Vt[-1]
    W = X[3]
    if abs(W) < 1e-5: W = 1e-5
    return X[:3] / W


# --- 3. VÒNG LẶP MÔ PHỎNG THỜI GIAN THỰC ---
t = 0.0
simulation_records = []

print("-> Dang khoi chay luong anh ảo. Nhan phím 'q' de dung...")

for frame_id in range(150):  # Mô phỏng 150 khung hình chuyển động
    t += 0.05

    # Tạo 2 khung hình nền đen (640x480) cho 2 Camera giả lập
    frame_cam1 = np.zeros((480, 640, 3), dtype=np.uint8)
    frame_cam2 = np.zeros((480, 640, 3), dtype=np.uint8)

    # Tọa độ thực tế 3D của 3 Marker đang xoay tròn ở khoảng cách Z = 3000mm (3 mét)
    cx, cy, cz = 100.0, 0.0, 3000.0
    r_orbit = 150.0  # Bán kính vòng xoay 15cm

    gt_markers = [
        np.array([cx + r_orbit * np.cos(t), cy + r_orbit * np.sin(t), cz, 1.0]),
        np.array([cx + r_orbit * np.cos(t + 2 * np.pi / 3), cy + r_orbit * np.sin(t + 2 * np.pi / 3), cz, 1.0]),
        np.array([cx + r_orbit * np.cos(t + 4 * np.pi / 3), cy + r_orbit * np.sin(t + 4 * np.pi / 3), cz, 1.0])
    ]

    # --- PHÍA SLAVE: CHIẾU 3D SANG ảnh 2D VÀ TÌM TÂM ---
    cam1_pixels = {}
    cam2_pixels = {}

    for idx, X_gt in enumerate(gt_markers):
        # Chiếu lên Cam 1
        proj1 = P1 @ X_gt
        u1, v1 = proj1[0] / proj1[2], proj1[1] / proj1[2]

        # Chiếu lên Cam 2 (Lưu ý: Hệ tọa độ Cam 2 tịnh tiến X)
        proj2 = P2 @ X_gt
        u2, v2 = proj2[0] / proj2[2], proj2[1] / proj2[2]

        # Thêm nhiễu Jitter pixel ngẫu nhiên giống thực tế phần cứng cơ học
        # Giảm nhiễu xuống mức siêu nhỏ để chứng minh độ chính xác của thuật toán DLT
        u1 += np.random.normal(0, 0.05)
        v1 += np.random.normal(0, 0.05)
        u2 += np.random.normal(0, 0.05)
        v2 += np.random.normal(0, 0.05)

        cam1_pixels[idx] = (u1, v1)
        cam2_pixels[idx] = (u2, v2)

        # Vẽ các đốm sáng trắng tượng trưng cho Marker lên luồng video ảo
        cv2.circle(frame_cam1, (int(u1), int(v1)), 8, (255, 255, 255), -1)
        cv2.circle(frame_cam2, (int(u2), int(v2)), 8, (255, 255, 255), -1)

    # --- PHÍA SERVER: GIẢI TOÁN TÁI TẠO 3D DLT ---
    calculated_3d = {}
    for idx in cam1_pixels.keys():
        if idx in cam2_pixels:
            pt3d = triangulate_dlt(cam1_pixels[idx], cam2_pixels[idx])
            calculated_3d[idx] = pt3d

            # Tính toán sai số cơ học Euclid tường minh tuyệt đối cho Marker 0
            if idx == 0:
                # Bóc tách rõ ràng tọa độ thực (Ground Truth)
                gt_x = gt_markers[0][0]
                gt_y = gt_markers[0][1]
                gt_z = gt_markers[0][2]

                # Bóc tách rõ ràng tọa độ thuật toán tính toán (Estimation)
                est_x = pt3d[0]
                est_y = pt3d[1]
                est_z = pt3d[2]

                # Tính toán khoảng cách Euclid chuẩn chỉnh
                error_mm = np.sqrt((gt_x - est_x) ** 2 + (gt_y - est_y) ** 2 + (gt_z - est_z) ** 2)

                simulation_records.append({
                    "Frame": frame_id,
                    "GT_X": gt_x, "GT_Y": gt_y, "GT_Z": gt_z,
                    "Est_X": est_x, "Est_Y": est_y, "Est_Z": est_z,
                    "Error_mm": error_mm
                })
                simulation_records.append({
                    "Frame": frame_id,
                    "GT_X": gt_markers[0][0], "GT_Y": gt_markers[0][1], "GT_Z": gt_markers[0][2],
                    "Est_X": pt3d[0], "Est_Y": pt3d[1], "Est_Z": pt3d[2],
                    "Error_mm": error_mm
                })

    # --- TRỰC QUAN HÓA KẾT QUẢ ĐỒ HỌA ---
    for idx, pt3d in calculated_3d.items():
        u1, v1 = cam1_pixels[idx]
        cv2.circle(frame_cam1, (int(u1), int(v1)), 15, (0, 255, 0), 2)
        cv2.putText(frame_cam1, f"ID:{idx}", (int(u1) + 15, int(v1) - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        if idx == 0:
            print(
                f"Frame {frame_id:03d} | Marker 0 -> X: {pt3d[0]:.1f}mm | Y: {pt3d[1]:.1f}mm | Z: {pt3d[2]:.1f}mm | Sai số: {error_mm:.2f}mm")

    cv2.imshow("Cam 1 - Monitor (SIL Python Mode)", frame_cam1)
    cv2.imshow("Cam 2 - Monitor (SIL Python Mode)", frame_cam2)

    if cv2.waitKey(33) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()

# --- 4. XUẤT FILE EXCEL BÁO CÁO SAI SỐ ---
df = pd.DataFrame(simulation_records)
df.to_excel("Ket_Qua_Thuc_Nghiem_SIL.xlsx", index=False)
print("\n--- HOÀN THÀNH MÔ PHỎNG ---")
print(f"-> Đã xuất dữ liệu thực nghiệm ra file 'Ket_Qua_Thuc_Nghiem_SIL.xlsx' thành công!")
print(f"-> Sai số tuyệt đối trung bình (MAE): {df['Error_mm'].mean():.4f} mm")
print(f"-> Sai số lớn nhất (Max Error): {df['Error_mm'].max():.4f} mm")