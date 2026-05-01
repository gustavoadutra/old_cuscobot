import cv2
import cv2.aruco as aruco
import numpy as np
import os
import csv
import glob
from scipy.spatial.transform import Rotation as R

# ==========================================
# CONFIGURAÇÕES
# ==========================================
DATASET_FOLDER = "dataset_20260321_115430" # <-- ALTERE AQUI
RTSP_IMAGES_FOLDER = os.path.join(DATASET_FOLDER, "rtsp_images_fixed")
SENSOR_DATA_FOLDER = os.path.join(DATASET_FOLDER, "sensor_data")
CSV_FILENAME = os.path.join(SENSOR_DATA_FOLDER, "vrs_gps_offline.csv")

TAMANHO_REAL_MARKER = 0.15  # Metros
ARUCO_ID = 42

def main():
    if not os.path.exists(RTSP_IMAGES_FOLDER):
        print(f"[ERRO] Pasta não encontrada: {RTSP_IMAGES_FOLDER}")
        return

    # --- CALIBRAÇÃO DA SUA CÂMERA (Dados do MATLAB) ---
    width = 1920
    height = 1080

    K_raw = np.array([
        [1078.79585,           0.0,  988.796493],
        [         0.0, 1085.59661,  547.254318],
        [         0.0,           0.0,           1.0]
    ], dtype=np.float64)

    D = np.array([-0.27300998, 0.0579501, 0.0, 0.0, 0.0], dtype=np.float64)

    # Calcula a matriz sem distorção (K_new) e gera os mapas de remapeamento
    K_new, roi = cv2.getOptimalNewCameraMatrix(K_raw, D, (width, height), 0, (width, height))
    map1, map2 = cv2.initUndistortRectifyMap(K_raw, D, None, K_new, (width, height), cv2.CV_32FC1)

    # --- SETUP PARA SOLVEPNP ---
    # Importante: A pose deve ser calculada usando a K_new porque a imagem
    # onde o ArUco será detectado JÁ ESTARÁ retificada.
    obj_points = np.array([
        [-TAMANHO_REAL_MARKER/2,  TAMANHO_REAL_MARKER/2, 0],
        [ TAMANHO_REAL_MARKER/2,  TAMANHO_REAL_MARKER/2, 0],
        [ TAMANHO_REAL_MARKER/2, -TAMANHO_REAL_MARKER/2, 0],
        [-TAMANHO_REAL_MARKER/2, -TAMANHO_REAL_MARKER/2, 0],
    ], dtype=np.float32)

    # --- SETUP ARUCO ---
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    parameters = aruco.DetectorParameters()
    detector = aruco.ArucoDetector(aruco_dict, parameters)

    image_paths = sorted(glob.glob(os.path.join(RTSP_IMAGES_FOLDER, "*.png")))
    total_images = len(image_paths)
    
    if total_images == 0:
        print("[ERRO] Nenhuma imagem .png encontrada.")
        return

    print(f"Processando {total_images} imagens...")

    with open(CSV_FILENAME, mode="w", newline="") as csv_file:
        csv_writer = csv.writer(csv_file)

        for i, img_path in enumerate(image_paths):
            timestamp_ns = int(os.path.basename(img_path).split('.')[0])
            
            # 1. Lê a imagem original (Com distorção barril)
            frame_raw = cv2.imread(img_path)
            if frame_raw is None:
                continue

            # 2. Aplica a transformação (Retificação)
            frame_rectified = cv2.remap(frame_raw, map1, map2, cv2.INTER_LINEAR)

            # Cópia para desenhar os marcadores sem alterar o cálculo
            display_rectified = frame_rectified.copy()

            # 3. Detecta ArUco na imagem RETIFICADA
            gray = cv2.cvtColor(frame_rectified, cv2.COLOR_BGR2GRAY)
            corners, ids, rejected = detector.detectMarkers(gray)

            pose_text = "ArUco nao detectado"
            color = (0, 0, 255)

            if ids is not None:
                aruco.drawDetectedMarkers(display_rectified, corners, ids)

                if ARUCO_ID in ids:
                    idx = np.where(ids == ARUCO_ID)[0][0]
                    marker_corners = corners[idx][0]

                    # Calcula Pose usando a K_new (pois a imagem já foi remapeada)
                    success, rvec, tvec = cv2.solvePnP(
                        obj_points, marker_corners, K_new, np.zeros((4,1)), flags=cv2.SOLVEPNP_IPPE_SQUARE
                    )

                    if success:
                        cv2.drawFrameAxes(display_rectified, K_new, np.zeros((4,1)), rvec, tvec, 0.1)

                        rmat, _ = cv2.Rodrigues(rvec)
                        r = R.from_matrix(rmat)
                        heading_degrees = r.as_euler("zyx", degrees=True)[0]

                        new_row = [
                            timestamp_ns, 0, 0, 
                            tvec[0][0], tvec[1][0], tvec[2][0], 
                            0, 0, 0, 0, 0, 0, 1, heading_degrees
                        ]
                        csv_writer.writerow(new_row)
                        
                        pose_text = f"X:{tvec[0][0]:.2f} Z:{tvec[2][0]:.2f} Th:{heading_degrees:.1f}"
                        color = (0, 255, 0)

            # --- VISUALIZAÇÃO LADO A LADO ---
            
            # Reduz o tamanho de ambas para caberem juntas na tela (ex: 640x360 cada)
            disp_raw = cv2.resize(frame_raw, (640, 360))
            disp_rect = cv2.resize(display_rectified, (640, 360))

            # Adiciona os títulos
            cv2.putText(disp_raw, "1. Original (Distortion)", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            cv2.putText(disp_rect, f"2. Rectified -> {pose_text}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

            # Junta as duas imagens horizontalmente
            combined_view = np.hstack((disp_raw, disp_rect))

            cv2.imshow("Ground Truth Offline Gen", combined_view)

            # Delay ajustado para 30ms para parecer um vídeo fluido. 
            # Pressione 'q' para sair.
            if cv2.waitKey(30) & 0xFF == ord('q'):
                break

    cv2.destroyAllWindows()
    print("[CONCLUÍDO]")

if __name__ == "__main__":
    main()