import cv2
import cv2.aruco as aruco
import numpy as np
import time
import pandas as pd
import os
import math

# CONFIGURAÇÃO
# Meça isso no chão real: Quantos metros tem a largura da imagem vista pela câmera?
TAMANHO_REAL_MARKER = 0.15  # 15 cm em metros
ARUCO_ID = 42  # ID do marcador que você imprimiu

rtsp_url = "rtsp://admin:nupedee7@192.168.1.4:554/cam/realmonitor?channel=1&subtype=0&proto=Onvif"

# Configuração Opcional: Forçar TCP ao invés de UDP
# Isso ajuda a evitar "smearing" (borrões cinzas) se a rede oscilar,
# o que é CRUCIAL para Odometria Visual.
os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "rtsp_transport;tcp"
os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "rtsp_transport;tcp|fflags;nobuffer|flags;low_delay"
# Inicia a captura
cap = cv2.VideoCapture(rtsp_url)

# Define resolução menor
#cap.set(cv2.CAP_PROP_FRAME_WIDTH, 240)   # Metade da largura
#cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 360)  # Metade da altura
#cap.set(cv2.CAP_PROP_FPS, 15)            # Reduza FPS se possível (padrão é 30)
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)      # Reduz buffer (menos lag)
# Configuração do ArUco (Nova API)
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
parameters = aruco.DetectorParameters()
detector = aruco.ArucoDetector(aruco_dict, parameters)

ground_truth_data = []
start_time = time.time()

print("Gravando Ground Truth... Pressione 'q' para parar.")

while True:
    ret, frame = cap.read()
    if not ret:
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, rejected = detector.detectMarkers(gray)

    if ids is not None and ARUCO_ID in ids:
        idx = np.where(ids == ARUCO_ID)[0][0]
        c = corners[idx][0]

        # 1. CALCULAR LARGURA EM PIXELS
        # Pegamos o canto superior esquerdo (pt0) e superior direito (pt1)
        # c[0] = canto sup esq, c[1] = canto sup dir
        p1 = c[0]
        p2 = c[1]

        # Distância Euclidiana (Teorema de Pitágoras) entre os cantos
        largura_pixels = math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)

        # 2. CALCULAR A ESCALA (Metros por Pixel)
        if largura_pixels > 0:
            scale = TAMANHO_REAL_MARKER / largura_pixels
        else:
            continue

        # 3. PEGAR O CENTRO DO ROBÔ
        center_x_px = int((c[0][0] + c[2][0]) / 2)
        center_y_px = int((c[0][1] + c[2][1]) / 2)

        # 4. CONVERTER POSIÇÃO PARA METROS
        # Aqui assumimos que o canto (0,0) da imagem é a posição (0,0) metros do mundo
        real_x = center_x_px * scale

        # Invertemos Y e usamos a altura total para a origem ficar no canto inferior esquerdo (opcional)
        height_img, _, _ = frame.shape
        real_y = (height_img - center_y_px) * scale

        # Salva dados
        timestamp = time.time() - start_time
        ground_truth_data.append([timestamp, real_x, real_y])

        # --- VISUALIZAÇÃO ---
        # Desenha o marcador e o centro
        cv2.polylines(frame, [c.astype(int)], True, (0, 255, 0), 2)
        cv2.circle(frame, (center_x_px, center_y_px), 5, (0, 0, 255), -1)

        # Escreve a posição na tela
        texto = f"X: {real_x:.2f}m | Y: {real_y:.2f}m"
        cv2.putText(
            frame, texto, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2
        )

        # Escreve a escala atual (para debug)
        cv2.putText(
            frame,
            f"Scale: {scale:.5f} m/px",
            (10, 60),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 0, 0),
            1,
        )

    cv2.imshow("Ground Truth Recorder", frame)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()

if len(ground_truth_data) > 0:
    df = pd.DataFrame(ground_truth_data, columns=["timestamp", "x", "y"])
    df.to_csv("gt_quadrado_15cm.csv", index=False)
    print("Arquivo salvo: gt_quadrado_15cm.csv")
