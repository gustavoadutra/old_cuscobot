import cv2
import cv2.aruco as aruco
import numpy as np
import os
import glob
import csv
from scipy.spatial.transform import Rotation as R

# --- MESMAS CONFIGURAÇÕES DO SEU CÓDIGO ORIGINAL ---
TAMANHO_REAL_MARKER = 0.15
ARUCO_ID = 42

big_dataset = "/home/aki/Desktop/old_cuscobot/dataset_20260126_084725(W)/"
RTSP_IMAGES_FOLDER = big_dataset + "rtsp_images"

CSV_OUTPUT = big_dataset + "sensor_data/vrs_gps1.csv"

# Garante que a pasta de saída existe
os.makedirs(os.path.dirname(CSV_OUTPUT), exist_ok=True)

# --- CALIBRAÇÃO (Aproximada) ---
c_w, c_h = 1280, 720
focal_length = c_w
camera_matrix = np.array(
    [[focal_length, 0, c_w / 2], [0, focal_length, c_h / 2], [0, 0, 1]], dtype=float
)
dist_coeffs = np.zeros((4, 1))

# Pontos 3D do Marker (Centralizado no 0,0,0 do próprio marker)
half = TAMANHO_REAL_MARKER / 2
obj_points = np.array(
    [[-half, half, 0], [half, half, 0], [half, -half, 0], [-half, -half, 0]],
    dtype=np.float32,
)


def main():
    # Setup Aruco
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    parameters = aruco.DetectorParameters()
    detector = aruco.ArucoDetector(aruco_dict, parameters)

    # Pegar imagens ordenadas
    image_files = sorted(glob.glob(os.path.join(RTSP_IMAGES_FOLDER, "*.png")))
    if not image_files:
        print("Nenhuma imagem encontrada!")
        return

    # Preparar CSV
    csv_file = open(CSV_OUTPUT, mode="w", newline="")
    csv_writer = csv.writer(csv_file)
    # Cabeçalho opcional (comente se seu leitor não suportar cabeçalho)
    # csv_writer.writerow(["timestamp", "x", "y", "z", "roll", "pitch", "yaw"])

    print(f"Processando {len(image_files)} imagens...")

    path_points = []  # Para desenho na tela

    for img_path in image_files:
        frame = cv2.imread(img_path)
        if frame is None:
            continue

        # Extrair timestamp do nome do arquivo (ex: 17000000.png -> 17000000)
        try:
            timestamp_ns = int(os.path.basename(img_path).split(".")[0])
        except ValueError:
            timestamp_ns = 0

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = detector.detectMarkers(gray)

        row_data = None

        if ids is not None and ARUCO_ID in ids:
            idx = np.where(ids == ARUCO_ID)[0][0]
            marker_corners = corners[idx][0]

            # 1. SolvePnP (Obtemos tvec direto)
            success, rvec, tvec = cv2.solvePnP(
                obj_points,
                marker_corners,
                camera_matrix,
                dist_coeffs,
                flags=cv2.SOLVEPNP_IPPE_SQUARE,
            )

            if success:
                # --- EXTRAÇÃO DE DADOS (SEM INVERSÃO DE RODRIGUES) ---
                # Posição direta em relação à câmera
                pos_x = tvec[0][0]
                pos_y = tvec[1][0]  # Altura (se a câmera estiver reta)
                pos_z = tvec[2][0]  # Profundidade

                # --- CÁLCULO DA ROTAÇÃO (HEADING) ---
                # Convertemos o vetor de rotação para matriz e depois para Euler
                rmat, _ = cv2.Rodrigues(rvec)
                r = R.from_matrix(rmat)
                # Yaw, Pitch, Roll em graus
                euler_angles = r.as_euler("zyx", degrees=True)
                yaw = euler_angles[0]

                # Salvar no formato do seu CSV original
                # Estrutura: [timestamp, ign, ign, X, Y, Z, ..., heading]
                # Nota: Usei Z como Y no CSV se o seu visualizador esperar Y como profundidade,
                # mas o padrão PnP é Z=frente. Mantive o padrão PnP abaixo:
                row_data = [
                    timestamp_ns,  # 0
                    0,
                    0,  # 1, 2
                    pos_x,  # 3 (X)
                    pos_y,  # 4 (Y - Altura)
                    pos_z,  # 5 (Z - Profundidade)
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,  # Covariâncias
                    1,  # 12: Status
                    yaw,  # 13: Heading (Rotação do robô)
                ]

                csv_writer.writerow(row_data)

                # --- VISUALIZAÇÃO ---
                cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvec, 0.1)

                # Desenhar trajeto (Escala 300px por metro)
                draw_x = int(c_w / 2 + (pos_x * 300))
                # Z cresce para longe, então na tela Y diminui (sobe) ou aumenta (desce)
                # Vamos fazer Z crescer para baixo na tela visualmente
                draw_y = int(100 + (pos_y * 100))

                path_points.append((draw_x, draw_y))

                # Texto informativo
                cv2.putText(
                    frame,
                    f"X:{pos_x:.2f} Z:{pos_y:.2f} Yaw:{yaw:.1f}",
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 0),
                    2,
                )

        # Desenhar linha do tempo
        for i in range(1, len(path_points)):
            cv2.line(frame, path_points[i - 1], path_points[i], (0, 0, 255), 2)

        cv2.imshow("Processamento e CSV", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    csv_file.close()
    cv2.destroyAllWindows()
    print(f"Concluído. Dados salvos em {CSV_OUTPUT}")


if __name__ == "__main__":
    main()
