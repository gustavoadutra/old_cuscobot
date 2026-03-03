import threading
import cv2
import cv2.aruco as aruco
from cv2.gapi.streaming import timestamp
import numpy as np
import time
import pandas as pd
import os
import math
import csv
import matplotlib.pyplot as plt
import pygame
import plotly.graph_objects as go
from datetime import datetime
from scipy.spatial.transform import Rotation as R


# --- IMPORT SEUS MODULOS EXISTENTES ---
import encoder
import odometry
import serialCom
import goToGoal

# ==========================================
# VARIÁVEIS GLOBAIS COMPARTILHADAS
# ==========================================
program_running = True
global_start_time = time.time()

# Locks para segurança entre threads
data_lock = threading.Lock()  # z
pose_lock = threading.Lock()  # Para odometria e trajetória

# Estado do Sistema
VISUAL_READY = False  # Só inicia o robô quando a câmera pegar o ArUco
ROBOT_FINISHED = False

# Dados Compartilhados (Imagens)
shared_frame_rtsp = None
shared_frame_webcam = None

# Dados Compartilhados (Pose e Trajetória para visualização)
shared_robot_pose = {"x": 0, "y": 0, "theta": 0}
shared_trajectory_points = []  # Lista de tuplas (x, y)
shared_current_goal = (0, 0)

# ==========================================
# CRIAR ESTRUTURA DE PASTAS
# ==========================================
# Nome da pasta principal com timestamp
TIMESTAMP_STR = datetime.fromtimestamp(global_start_time).strftime("%Y%m%d_%H%M%S")
GLOBAL_FOLDER = f"dataset_{TIMESTAMP_STR}"

# Criar estrutura de pastas
SENSOR_DATA_FOLDER = os.path.join(GLOBAL_FOLDER, "sensor_data")
STEREO_LEFT_FOLDER = os.path.join(GLOBAL_FOLDER, "stereo_left")

os.makedirs(SENSOR_DATA_FOLDER, exist_ok=True)
os.makedirs(STEREO_LEFT_FOLDER, exist_ok=True)

print(f"[SETUP] Pasta principal criada: {GLOBAL_FOLDER}")
print(f"[SETUP] Sensor data: {SENSOR_DATA_FOLDER}")
print(f"[SETUP] Stereo left: {STEREO_LEFT_FOLDER}")


def run_visual_odometry():
    global \
        program_running, \
        global_start_time, \
        shared_frame_rtsp, \
        VISUAL_READY, \
        data_lock

    # --- CONFIG ---
    TAMANHO_REAL_MARKER = 0.15  # Metros
    ARUCO_ID = 42
    RTSP_URL = "rtsp://admin:nupedee7@192.168.1.6:554/cam/realmonitor?channel=1&subtype=0&proto=Onvif"

    # Nome do arquivo de saída (dentro de sensor_data)
    CSV_FILENAME = os.path.join(SENSOR_DATA_FOLDER, "vrs_gps.csv")

    # --- NOVO: Pasta para imagens RTSP ---
    RTSP_IMAGES_FOLDER = os.path.join(GLOBAL_FOLDER, "rtsp_images")
    os.makedirs(RTSP_IMAGES_FOLDER, exist_ok=True)
    print(f"[THREAD VISUAL] Imagens RTSP serão salvas em: {RTSP_IMAGES_FOLDER}")

    # Arquivo de timestamp para imagens RTSP
    RTSP_STAMP_FILENAME = os.path.join(SENSOR_DATA_FOLDER, "rtsp_stamp.csv")
    rtsp_stamp_file = open(RTSP_STAMP_FILENAME, mode="w", newline="")
    rtsp_stamp_writer = csv.writer(rtsp_stamp_file)

    # --- CALIBRAÇÃO--- Implementar
    c_w, c_h = 1280, 720
    focal_length = c_w
    center = (c_w / 2, c_h / 2)
    camera_matrix = np.array(
        [[focal_length, 0, center[0]], [0, focal_length, center[1]], [0, 0, 1]],
        dtype="double",
    )
    dist_coeffs = np.zeros((4, 1))

    # Pontos 3D do Marker
    half_size = TAMANHO_REAL_MARKER / 2
    obj_points = np.array(
        [
            [-half_size, half_size, 0],
            [half_size, half_size, 0],
            [half_size, -half_size, 0],
            [-half_size, -half_size, 0],
        ],
        dtype=np.float32,
    )

    os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "rtsp_transport;tcp"

    print("[THREAD VISUAL] Conectando ao RTSP...")
    cap = cv2.VideoCapture(RTSP_URL)

    # Cria/Limpa o arquivo CSV e prepara o writer
    csv_file = open(CSV_FILENAME, mode="w", newline="")
    csv_writer = csv.writer(csv_file)

    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    parameters = aruco.DetectorParameters()
    detector = aruco.ArucoDetector(aruco_dict, parameters)

    print(f"[THREAD VISUAL] Salvando Ground Truth em: {CSV_FILENAME}")

    while program_running:
        ret, frame = cap.read()
        if not ret:
            time.sleep(0.1)
            continue

        # Timestamp KAIST: Nanosegundos do sistema
        timestamp_ns = time.time_ns()

        # --- NOVO: Salvar imagem raw ---
        filename = f"{timestamp_ns}.png"
        save_path = os.path.join(RTSP_IMAGES_FOLDER, filename)
        cv2.imwrite(save_path, frame)

        # Salvar timestamp
        rtsp_stamp_writer.writerow([timestamp_ns])
        rtsp_stamp_file.flush()

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = detector.detectMarkers(gray)

        # Variáveis para display
        display_x, display_y = 0.0, 0.0

        if ids is not None:
            aruco.drawDetectedMarkers(frame, corners, ids)

            if ARUCO_ID in ids:
                idx = np.where(ids == ARUCO_ID)[0][0]
                marker_corners = corners[idx][0]

                # 1. SolvePnP (Marker no referencial da Câmera)
                success, rvec, tvec = cv2.solvePnP(
                    obj_points,
                    marker_corners,
                    camera_matrix,
                    dist_coeffs,
                    flags=cv2.SOLVEPNP_IPPE_SQUARE,
                )

                if success:
                    cv2.drawFrameAxes(
                        frame, camera_matrix, dist_coeffs, rvec, tvec, 0.1
                    )

                    # --- MATEMÁTICA DA ODOMETRIA (Inversão) ---
                    # Queremos a Pose da Câmera no Mundo (Referencial do Marker)

                    # 1. Rotação (Marker -> Câmera)
                    rmat, _ = cv2.Rodrigues(rvec)
                    r = R.from_matrix(rmat)
                    # Yaw, Pitch, Roll em graus
                    euler_angles = r.as_euler("zyx", degrees=True)
                    heading_degrees = euler_angles[0]  # Yaw em graus

                    new_row = [
                        timestamp_ns,  # 0: Timestamp
                        0,  # 1: Ignorado pelo loader
                        0,  # 2: Ignorado pelo loader
                        tvec[0][0],  # 3: X (Loader lê gps_x_raw aqui)
                        tvec[1][0],  # 4: Y (Loader lê gps_y_raw aqui)
                        tvec[2][0],  # 5: Z (Loader lê gps_z_raw aqui)
                        0,
                        0,
                        0,  # 6, 7, 8: Covariância (Ignorado)
                        0,
                        0,
                        0,  # 9, 10, 11: Covariância (Ignorado)
                        1,  # 12: Status Flag (OBRIGATÓRIO SER 1 para ler rotação)
                        heading_degrees,  # 13: Heading em graus
                    ]

                    csv_writer.writerow(new_row)
                    csv_file.flush()  # Garante gravação imediata no disco

                    # --- Extração para Display (Visualização apenas) ---
                    display_x = float(tvec[0])
                    # Dependendo do eixo (assumindo Z para frente no PnP padrão)
                    display_y = float(tvec[2])

                    # Lógica de Inicialização do Robô
                    if not VISUAL_READY:
                        print(f"[THREAD VISUAL] Inicializado! Pose salva no CSV.")
                        with data_lock:
                            VISUAL_READY = True

        # Atualiza GUI
        frame_resized = cv2.resize(frame, (320, 240))
        cv2.putText(
            frame_resized,
            f"GT X:{display_x:.2f} Z:{display_y:.2f}",
            (10, 230),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 0),
            2,
        )

        frame_rgb = cv2.cvtColor(frame_resized, cv2.COLOR_BGR2RGB)
        with data_lock:
            shared_frame_rtsp = frame_rgb

    # Limpeza final
    csv_file.close()
    rtsp_stamp_file.close()
    cap.release()
    print("[THREAD VISUAL] Finalizada e arquivos salvos.")


def run_robot_control():
    global \
        program_running, \
        shared_robot_pose, \
        shared_trajectory_points, \
        shared_current_goal, \
        ROBOT_FINISHED, \
        pose_lock

    # 1. Aguarda a Visual Odometry estar pronta (Sincronia de Threads)
    print("[THREAD CONTROL] Aguardando Câmera Visual...")
    while program_running and not VISUAL_READY:
        time.sleep(0.1)

    if not program_running:
        return

    print("[THREAD CONTROL] Câmera OK. Iniciando controle do robô!")

    # 2. Config Hardware
    PORT = "/dev/ttyACM0"
    BAUDRATE = 9600
    TIMEOUT = 0.05
    WHEEL_RADIUS = 0.06
    WHEEL_BASE = 0.335
    TICKS_PER_REVOLUTION = 980

    # Parâmetros PID / Controle
    MAX_PWM = 10
    MAX_PWM_STEP = 10
    MAX_SPEED_DISTANCE = 1

    # --- SETUP ARQUIVO ENCODER (KAIST FORMAT) ---
    # Formato: timestamp_ns, left_ticks, right_ticks
    ENC_FILENAME = os.path.join(SENSOR_DATA_FOLDER, "encoder.csv")

    enc_file = open(ENC_FILENAME, mode="w", newline="")
    enc_writer = csv.writer(enc_file)

    # Cabeçalho opcional (Datasets puros geralmente não tem, mas ajuda no debug)
    # enc_writer.writerow(["timestamp", "left", "right"])
    print(f"[THREAD CONTROL] Salvando dados de encoder em: {ENC_FILENAME}")

    # Inicializa Objetos
    left_wheel_encoder = encoder.encoder(TICKS_PER_REVOLUTION, WHEEL_RADIUS)
    right_wheel_encoder = encoder.encoder(TICKS_PER_REVOLUTION, WHEEL_RADIUS)

    try:
        arduino = serialCom.Communication(PORT, BAUDRATE, TIMEOUT)
        arduino.open()
        time.sleep(2)  # Reset Arduino
    except Exception as e:
        print(f"[ERROR] Não foi possível conectar ao Arduino: {e}")
        return

    odom = odometry.odometry(left_wheel_encoder, right_wheel_encoder, WHEEL_BASE)
    controller = goToGoal.GoToGoal()

    # Variáveis de Controle
    start_time_ns = time.time_ns()
    last_left_pwm, last_right_pwm = 128, 128

    # Trajetória
    PATH = [[0, 1], [1, 1], [1, 0], [0, 0]]
    # PATH = [[0, 1]]
    step = 0
    goal = PATH[step]
    last_w = 0

    # Loop de Controle
    while program_running:
        iter_start = time.time()

        # A. Leitura Serial & Gravação de Dataset
        if arduino:
            try:
                le_raw = arduino.get_encoder_left()
                ld_raw = arduino.get_encoder_right()

                # Timestamp exato da leitura (Nanosegundos)
                read_time_ns = time.time_ns()

                # SÓ PROCESSA SE OS DADOS FOREM VÁLIDOS (DIFERENTES DE NONE)
                if le_raw is not None and ld_raw is not None:
                    # Converte para int aqui dentro do try para evitar erro de conversão
                    le_val = int(le_raw)
                    ld_val = int(ld_raw)

                    # Atualiza os objetos encoder para a odometria
                    left_wheel_encoder.counter = le_val
                    right_wheel_encoder.counter = ld_val

                    # --- SALVAR NO CSV ---
                    enc_writer.writerow([read_time_ns, le_val, ld_val])
                    enc_file.flush()
            except ValueError:
                # Se chegou uma string suja tipo "12a4", ignora este ciclo
                pass

        # B. Odometria Math (Dead Reckoning)
        current_time_ns = time.time_ns()
        dt = current_time_ns - start_time_ns
        start_time_ns = current_time_ns

        odom.step()
        x, y, theta = odom.getPose()

        # C. Atualiza Globais (para a interface gráfica)
        with pose_lock:
            shared_robot_pose["x"] = x
            shared_robot_pose["y"] = y
            shared_robot_pose["theta"] = theta
            shared_current_goal = goal

            # Adiciona ponto à trajetoria visual apenas se o robô se moveu significativamente
            # (Opcional: evitar encher a memória se estiver parado)
            if (
                len(shared_trajectory_points) == 0
                or (
                    (x - shared_trajectory_points[-1][0]) ** 2
                    + (y - shared_trajectory_points[-1][1]) ** 2
                )
                > 0.0001
            ):
                shared_trajectory_points.append((x, y))

        # D. Controle (Go To Goal)
        if not ROBOT_FINISHED:
            # Algoritmo de controle retorna velocidade angular (w) necessária
            w = controller.step(goal[0], goal[1], x, y, theta, dt, precision=0.05)

            if w is not None:
                last_w = w
            else:
                # Chegou no waypoint
                if step + 1 < len(PATH):
                    step += 1
                    goal = PATH[step]
                    last_w = 0
                    print(f"[THREAD CONTROL] Waypoint atingido! Próximo: {goal}")
                else:
                    print("[THREAD CONTROL] Trajetória Finalizada!")
                    ROBOT_FINISHED = True
                    last_w = 0

            # Calcula velocidades lineares das rodas (Modelo Diferencial)
            left_vel, right_vel = odometry.uni_to_diff(
                3,  # Velocidade linear alvo (m/s) - Ajuste conforme necessário
                last_w,
                left_wheel_encoder,
                right_wheel_encoder,
                WHEEL_BASE,
            )

            # Normalização e Limitação de PWM
            max_val = max(abs(left_vel), abs(right_vel))
            # Evita divisão por zero
            if max_val == 0:
                left_norm, right_norm = 0, 0
            else:
                left_norm = left_vel / max_val
                right_norm = right_vel / max_val

            # Reduz velocidade quando chega perto do alvo
            spd_ctrl = controller.speed_limit_by_distance(
                MAX_SPEED_DISTANCE, MAX_PWM, goal[0], goal[1], x, y
            )

            # Converte para PWM (0-255, onde 128 é parado)
            target_l = (left_norm * spd_ctrl) + 128
            target_r = (right_norm * spd_ctrl) + 128

            # Rampa de Aceleração (Suavização para não dar tranco nos motores)
            last_left_pwm += max(
                min(target_l - last_left_pwm, MAX_PWM_STEP), -MAX_PWM_STEP
            )
            last_right_pwm += max(
                min(target_r - last_right_pwm, MAX_PWM_STEP), -MAX_PWM_STEP
            )

            if arduino:
                arduino.set_speed_left(int(last_left_pwm))
                arduino.set_speed_right(int(last_right_pwm))
        else:
            # Parar robô (128 = 0V no driver comum tipo Sabertooth/BTS com offset)
            if arduino:
                arduino.set_speed_left(128)
                arduino.set_speed_right(128)

    # Cleanup
    if arduino:
        arduino.close()

    # Fecha arquivo CSV
    enc_file.close()
    print("[THREAD CONTROL] Arquivo de encoder salvo e thread finalizada.")


def run_webcam_recorder():
    global program_running, shared_frame_webcam, data_lock

    # --- CONFIG ---
    TARGET_FPS = 30
    FRAME_INTERVAL = 1.0 / TARGET_FPS

    # As imagens vão direto para stereo_left
    print(f"[WEBCAM] Salvando imagens em: {STEREO_LEFT_FOLDER}")

    # --- SETUP ARQUIVO ENCODER (KAIST FORMAT) ---
    # Formato: timestamp_ns, left_ticks, right_ticks
    STEREO_STAMP_FILENAME = os.path.join(SENSOR_DATA_FOLDER, "stereo_stamp.csv")

    stereo_stamp = open(STEREO_STAMP_FILENAME, mode="w", newline="")
    stereo_stamp_writer = csv.writer(stereo_stamp)
    cap_web = cv2.VideoCapture(2)

    # 1. Aguarda a Visual Odometry estar pronta
    print("[THREAD WEBCAM] Aguardando Câmera Visual...")
    while program_running and not VISUAL_READY:
        time.sleep(0.1)

    # Tenta definir o FPS no hardware para evitar buffer
    cap_web.set(cv2.CAP_PROP_FPS, TARGET_FPS)

    print("[WEBCAM] Gravando... ")

    while program_running:
        start_time = time.time()

        ret, frame = cap_web.read()

        if ret:
            # --- TIMESTAMP KAIST (Nanosegundos) ---
            # time.time_ns() retorna o tempo exato do sistema em int (nanosegundos)
            # Exemplo de saída: 1698421055123456789.jpg
            timestamp_ns = time.time_ns()

            filename = f"{timestamp_ns}.png"
            save_path = os.path.join(STEREO_LEFT_FOLDER, filename)

            # Salva a imagem
            # Camera foi colocada ao contrario
            frame = cv2.rotate(frame, cv2.ROTATE_180)
            cv2.imwrite(save_path, frame)

            # --- DISPLAY ---
            frame_resized = cv2.resize(frame, (320, 240))
            frame_rgb = cv2.cvtColor(frame_resized, cv2.COLOR_BGR2RGB)
            with data_lock:
                shared_frame_webcam = frame_rgb

            stereo_stamp_writer.writerow([timestamp_ns])

            stereo_stamp.flush()
        else:
            time.sleep(0.01)

        # --- CONTROLE DE 10HZ ---
        processing_time = time.time() - start_time
        sleep_duration = FRAME_INTERVAL - processing_time
        if sleep_duration > 0:
            time.sleep(sleep_duration)

    cap_web.release()
    stereo_stamp.close()

    print("[WEBCAM] Gravação finalizada.")


# ==========================================
# UTIL: World to Screen
# ==========================================
def world_to_screen(x, y, center_x, center_y, scale=100):
    screen_x = center_x + int(x * scale)
    screen_y = center_y - int(y * scale)  # Inverte Y
    return (screen_x, screen_y)


# ==========================================
# MAIN THREAD: VISUALIZATION ONLY
# ==========================================
def main():
    global program_running, VISUAL_READY

    print("=" * 60)
    print(f"DATASET COLLECTION SESSION")
    print(f"Pasta principal: {GLOBAL_FOLDER}")
    print(f"  - sensor_data/encoder.csv")
    print(f"  - sensor_data/vrs_gps.csv")
    print(f"  - stereo_left/*.png")
    print("=" * 60)

    # 1. Iniciar Threads
    t_visual = threading.Thread(target=run_visual_odometry)
    t_control = threading.Thread(target=run_robot_control)
    t_webcam = threading.Thread(target=run_webcam_recorder)

    t_visual.start()
    t_control.start()
    t_webcam.start()

    # 2. Config Pygame
    pygame.init()
    W_WIDTH, W_HEIGHT = 1000, 600
    screen = pygame.display.set_mode((W_WIDTH, W_HEIGHT))
    pygame.display.set_caption("ROBOT DASHBOARD - MULTITHREADED")

    font = pygame.font.SysFont("Arial", 18)
    clock = pygame.time.Clock()

    # Config Mapa
    MAP_CX, MAP_CY = 300, 400
    MAP_SCALE = 150

    while program_running:
        # Eventos
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                program_running = False
            if event.type == pygame.KEYDOWN and event.key == pygame.K_q:
                program_running = False

        screen.fill((30, 30, 30))

        # --- LER DADOS DAS THREADS (De forma thread-safe) ---
        with pose_lock:
            # Faz uma cópia rápida dos dados para não bloquear a outra thread desenhando
            curr_x = shared_robot_pose["x"]
            curr_y = shared_robot_pose["y"]
            curr_th = shared_robot_pose["theta"]
            # Copia a lista de pontos (slice [:] cria copia)
            pts_draw = shared_trajectory_points[::5]  # Pega 1 a cada 5 para ficar leve
            curr_goal = shared_current_goal

        with data_lock:
            img_rtsp = shared_frame_rtsp
            img_web = shared_frame_webcam
            is_ready = VISUAL_READY

        # --- 1. DESENHAR MAPA ---
        pygame.draw.rect(screen, (10, 10, 10), (20, 50, 600, 530))  # Fundo

        # Grid/Eixos
        cx_s, cy_s = world_to_screen(0, 0, MAP_CX, MAP_CY, MAP_SCALE)
        pygame.draw.line(screen, (50, 50, 50), (20, cy_s), (620, cy_s))  # X
        pygame.draw.line(screen, (50, 50, 50), (cx_s, 50), (cx_s, 580))  # Y

        # Trajetória
        if len(pts_draw) > 1:
            scr_pts = [
                world_to_screen(p[0], p[1], MAP_CX, MAP_CY, MAP_SCALE) for p in pts_draw
            ]
            pygame.draw.lines(screen, (0, 255, 255), False, scr_pts, 2)

        # Robô
        rx, ry = world_to_screen(curr_x, curr_y, MAP_CX, MAP_CY, MAP_SCALE)
        pygame.draw.circle(screen, (255, 50, 50), (rx, ry), 12)
        # Direção
        hx = rx + math.cos(curr_th) * 20
        hy = ry - math.sin(curr_th) * 20
        pygame.draw.line(screen, (255, 255, 0), (rx, ry), (hx, hy), 3)

        # Goal
        gx, gy = world_to_screen(curr_goal[0], curr_goal[1], MAP_CX, MAP_CY, MAP_SCALE)
        pygame.draw.circle(screen, (0, 255, 0), (gx, gy), 6)

        # --- 2. DESENHAR CAMERAS ---

        # RTSP View
        title_rtsp = font.render(
            f"Visual Odometry (Ready: {is_ready})", True, (200, 200, 200)
        )
        screen.blit(title_rtsp, (650, 20))

        if img_rtsp is not None:
            surf = pygame.surfarray.make_surface(np.rot90(img_rtsp))
            surf = pygame.transform.flip(surf, True, False)
            surf = pygame.transform.rotate(surf, 90)
            screen.blit(surf, (650, 50))
        else:
            pygame.draw.rect(screen, (50, 0, 0), (650, 50, 320, 240))
            screen.blit(font.render("Connecting...", True, (255, 255, 255)), (720, 150))

        # Webcam View
        screen.blit(font.render("Webcam Dataset", True, (200, 200, 200)), (650, 310))
        if img_web is not None:
            surf2 = pygame.surfarray.make_surface(np.rot90(img_web))
            surf2 = pygame.transform.flip(surf2, True, False)
            surf2 = pygame.transform.rotate(surf2, 90)
            screen.blit(surf2, (650, 340))

        # --- INFO ---
        info_txt = font.render(
            f"X: {curr_x:.2f}  Y: {curr_y:.2f}  Th: {math.degrees(curr_th):.1f}°",
            True,
            (255, 255, 255),
        )
        screen.blit(info_txt, (30, 20))

        # Informação da pasta
        folder_txt = font.render(
            f"Saving to: {GLOBAL_FOLDER}",
            True,
            (150, 150, 150),
        )
        screen.blit(folder_txt, (30, 560))

        pygame.display.flip()
        clock.tick(60)  # 60 FPS na tela (o robô roda na velocidade da thread dele)

    # Cleanup Global
    print("Desligando main...")
    t_visual.join()
    t_control.join()
    t_webcam.join()
    pygame.quit()

    print("=" * 60)
    print(f"DATASET SALVO COM SUCESSO!")
    print(f"Localização: {GLOBAL_FOLDER}")
    print("=" * 60)


if __name__ == "__main__":
    main()
