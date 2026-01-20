import threading
import cv2
import cv2.aruco as aruco
import numpy as np
import time
import pandas as pd
import os
import math
import csv
import matplotlib.pyplot as plt
import pygame
import plotly.graph_objects as go

# --- IMPORT SEUS MODULOS EXISTENTES ---
# Certifique-se que esses arquivos estao na mesma pasta
import encoder
import odometry
import serialCom
import goToGoal

# ==========================================
# VARIÁVEIS GLOBAIS DE CONTROLE E SINCRONIA
# ==========================================
program_running = True  # Flag mestre para parar todas as threads
global_start_time = time.time()  # Tempo zero compartilhado
data_lock = threading.Lock()  # Para evitar conflito de escrita se necessário


# ==========================================
# THREAD 1: VISUAL ODOMETRY (RTSP + ARUCO)
# ==========================================
def run_visual_odometry():
    global program_running, global_start_time

    # CONFIGURAÇÃO
    TAMANHO_REAL_MARKER = 0.15  # 15 cm
    ARUCO_ID = 42
    RTSP_URL = "rtsp://admin:nupedee7@192.168.0.108:554/cam/realmonitor?channel=1&subtype=0&proto=Onvif"

    os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "rtsp_transport;tcp"

    cap = cv2.VideoCapture(RTSP_URL)

    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    parameters = aruco.DetectorParameters()
    detector = aruco.ArucoDetector(aruco_dict, parameters)

    ground_truth_data = []

    # Variáveis para armazenar a pose inicial (First Frame)
    initial_pose = None  # Vai guardar {'x': val, 'y': val, 'theta': val}

    print("[THREAD 1] Visual Odometry Iniciada...")

    while program_running:
        ret, frame = cap.read()
        if not ret:
            print("[THREAD 1] Erro ao ler frame RTSP.")
            time.sleep(0.1)
            continue

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = detector.detectMarkers(gray)

        current_time = time.time() - global_start_time

        if ids is not None and ARUCO_ID in ids:
            idx = np.where(ids == ARUCO_ID)[0][0]
            c = corners[idx][0]

            # --- 1. CÁLCULO GEOMÉTRICO (Sem matriz de calibração) ---

            # Cantos: 0=TopLeft, 1=TopRight, 2=BottomRight, 3=BottomLeft
            p0, p1 = c[0], c[1]

            # Distância em pixels (largura)
            largura_px = math.sqrt((p0[0] - p1[0]) ** 2 + (p0[1] - p1[1]) ** 2)

            if largura_px > 0:
                scale = TAMANHO_REAL_MARKER / largura_px

                # Centro em Pixels
                cx_px = (c[0][0] + c[2][0]) / 2
                cy_px = (c[0][1] + c[2][1]) / 2

                # Posição Absoluta no Mundo (Baseada no canto 0,0 da imagem)
                abs_x = cx_px * scale
                # Invertendo Y para corresponder ao plano cartesiano padrão (fundo pra cima)
                height_img, _, _ = frame.shape
                abs_y = (height_img - cy_px) * scale

                # --- CÁLCULO DO THETA (Ângulo) ---
                # Calculamos o arco tangente da linha superior do marcador
                # Delta Y e Delta X entre TopRight e TopLeft
                delta_y = p1[1] - p0[1]
                delta_x = p1[0] - p0[0]

                # O sinal de Y no OpenCV cresce para baixo, então invertemos delta_y para o math.atan2
                # funcionar como cartesiano padrão, ou mantemos e ajustamos.
                # Vamos assumir padrão imagem:
                abs_theta = math.atan2(delta_y, delta_x)

                # --- RELATIVO AO PRIMEIRO FRAME ---
                if initial_pose is None:
                    initial_pose = {"x": abs_x, "y": abs_y, "theta": abs_theta}
                    rel_x, rel_y, rel_theta = 0, 0, 0
                    print(f"[THREAD 1] Origem definida: {initial_pose}")
                else:
                    # Diferença simples
                    dx = abs_x - initial_pose["x"]
                    dy = abs_y - initial_pose["y"]

                    # Para rotacionar as coordenadas para alinhar com a orientação inicial do robô:
                    # x_rel = dx * cos(-th0) - dy * sin(-th0)
                    # y_rel = dx * sin(-th0) + dy * cos(-th0)
                    # Mas se você quer apenas o deslocamento relativo à câmera:
                    rel_x = dx
                    rel_y = dy

                    rel_theta = abs_theta - initial_pose["theta"]

                    # Normalizar theta entre -pi e pi
                    rel_theta = (rel_theta + np.pi) % (2 * np.pi) - np.pi

                ground_truth_data.append([current_time, rel_x, rel_y, rel_theta])

    cap.release()

    # Salvar CSV ao final
    if ground_truth_data:
        df = pd.DataFrame(ground_truth_data, columns=["timestamp", "x", "y", "theta"])
        df.to_csv("gt_visual_relativo.csv", index=False)
        print("[THREAD 1] CSV Visual salvo.")


# ==========================================
# THREAD 2: WEBCAM RECORDER (DATASET)
# ==========================================
def run_webcam_recorder():
    global program_running, global_start_time

    SAVE_DIR = "dataset_images"
    if not os.path.exists(SAVE_DIR):
        os.makedirs(SAVE_DIR)

    # Tenta índice 0 (webcam integrada) ou 2 (se 1 for virtual/outro)
    # Se der erro, verifique qual ID é sua webcam USB
    cap_web = cv2.VideoCapture(0)

    print("[THREAD 2] Webcam Recording Iniciada...")

    while program_running:
        ret, frame = cap_web.read()
        if ret:
            # Timestamp preciso para nome do arquivo
            ts = time.time() - global_start_time
            filename = f"{SAVE_DIR}/img_{ts:.4f}.jpg"
            cv2.imwrite(filename, frame)

            # Pequeno sleep para não explodir o HD (ex: 10 FPS)
            time.sleep(0.1)
        else:
            time.sleep(0.1)

    cap_web.release()
    print("[THREAD 2] Webcam Finalizada.")


# ==========================================
# MAIN THREAD: ROBOT CONTROL & PYGAME
# ==========================================
def main():
    global program_running, global_start_time
    # 1. Iniciar Threads Secundárias
    t_visual = threading.Thread(target=run_visual_odometry)
    t_webcam = threading.Thread(target=run_webcam_recorder)

    t_visual.start()
    t_webcam.start()

    # 2. Configuração do Robô (Código Original Adaptado)
    PORT = "/dev/ttyACM0"
    BAUDRATE = 9600
    TIMEOUT = 0.1
    WHEEL_RADIUS = 0.06
    WHEEL_BASE = 0.335
    TICKS_PER_REVOLUTION = 980
    MAX_PWM = 48
    MAX_PWM_STEP = 10
    MAX_SPEED_DISTANCE = 1

    pygame.init()
    screen = pygame.display.set_mode((200, 100))
    pygame.display.set_caption("ROBÔ CONTROLLER - Main")
    font = pygame.font.SysFont("Arial", 18)

    left_wheel_encoder = encoder.encoder(TICKS_PER_REVOLUTION, WHEEL_RADIUS)
    right_wheel_encoder = encoder.encoder(TICKS_PER_REVOLUTION, WHEEL_RADIUS)
    arduino = serialCom.Communication(PORT, BAUDRATE, TIMEOUT)
    arduino.open()
    time.sleep(3)  # Aguarda reset do arduino

    clock = pygame.time.Clock()
    FPS = 100

    odom = odometry.odometry(left_wheel_encoder, right_wheel_encoder, WHEEL_BASE)
    controller = goToGoal.GoToGoal()

    # Variáveis de Controle
    start_time_ns = time.time_ns()
    last_read = time.time()
    last_left_pwm = 128
    last_right_pwm = 128

    pose_log = {"timestamp": [], "x": [], "y": [], "theta": []}

    # Path Planning
    PATH = [[0, 1], [1, 1]]
    step = 0
    goal = PATH[step]

    last_w = 0

    print("SISTEMA RODANDO. Pressione 'Q' na janela do Pygame para parar TUDO.")

    while program_running:
        # --- Pygame Events ---
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                program_running = False
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_q:
                    program_running = False

        # --- Leitura Arduino ---
        le = arduino.get_encoder_left()
        ld = arduino.get_encoder_right()

        if le and ld:
            right_wheel_encoder.counter = int(ld)
            left_wheel_encoder.counter = int(le)
            last_read = time.time()
        elif time.time() - last_read > 10:
            print("Timeout Arduino!")
            program_running = False
            break

        # --- Odometria ---
        current_time_ns = time.time_ns()
        dt = current_time_ns - start_time_ns  # Convertendo para segundos

        start_time_ns = current_time_ns

        odom.step()
        x, y, theta = odom.getPose()

        # Log
        pose_log["timestamp"].append(time.time() - global_start_time)
        pose_log["x"].append(x)
        pose_log["y"].append(y)
        pose_log["theta"].append(theta)

        # --- Controle ---
        w = controller.step(goal[0], goal[1], x, y, theta, dt, precision=0.05)

        if w is not None:
            last_w = w
        else:
            # Chegou ao ponto atual da trajetória
            if step + 1 == len(PATH):
                print("Trajetória concluída!")
                break
            step += 1
            goal = PATH[step]
            plt.scatter(goal[0], goal[1], marker="x", color="r")
            w = last_w

        left_vel, right_vel = odometry.uni_to_diff(
            5, w, left_wheel_encoder, right_wheel_encoder, WHEEL_BASE
        )

        # --- Conversão PWM ---
        # (Simplificada para brevidade, mantendo lógica do usuário)
        max_val = max(abs(left_vel), abs(right_vel))
        left_norm = left_vel / max_val if max_val > 0 else 0
        right_norm = right_vel / max_val if max_val > 0 else 0

        # Speed limit logic do usuario
        max_speed_control = controller.speed_limit_by_distance(
            MAX_SPEED_DISTANCE, MAX_PWM, goal[0], goal[1], x, y
        )

        left_pwm = (left_norm * max_speed_control) + 128
        right_pwm = (right_norm * max_speed_control) + 128

        # Rampa
        last_left_pwm += max(min(left_pwm - last_left_pwm, MAX_PWM_STEP), -MAX_PWM_STEP)
        last_right_pwm += max(
            min(right_pwm - last_right_pwm, MAX_PWM_STEP), -MAX_PWM_STEP
        )

        arduino.set_speed_left(int(last_left_pwm))
        arduino.set_speed_right(int(last_right_pwm))

        # Atualiza tela pygame (mostra status)
        screen.fill((0, 0, 0))
        text_surf = font.render(
            f"X: {x:.2f} Y: {y:.2f} Th: {theta:.2f}", True, (255, 255, 255)
        )
        screen.blit(text_surf, (10, 10))
        pygame.display.flip()

        clock.tick(FPS)

    # --- CLEANUP ---
    print("Encerrando sistema...")
    program_running = False  # Garante que threads parem

    # Parar Robô
    arduino.set_speed_left(128)
    arduino.set_speed_right(128)
    time.sleep(0.1)
    arduino.close()

    # Aguarda threads
    t_visual.join()
    t_webcam.join()

    pygame.quit()
    plt.close()

    # Salvar CSV Odometria
    filename = "trajetoria_odom.csv"
    with open(filename, mode="w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(["timestamp", "x", "y", "theta"])
        for i in range(len(pose_log["timestamp"])):
            writer.writerow(
                [
                    pose_log["timestamp"][i],
                    pose_log["x"][i],
                    pose_log["y"][i],
                    pose_log["theta"][i],
                ]
            )
    print(f"CSV Odom salvo: {filename}")

    # Plotly Final
    fig_go = go.Figure(
        go.Scatter(
            x=pose_log["x"], y=pose_log["y"], mode="lines+markers", name="Odometria"
        )
    )
    fig_go.update_layout(title="Trajetória Final", xaxis_title="X", yaxis_title="Y")
    fig_go.write_html("trajetoria_final.html")  # Salva em HTML para não bloquear script
    print("Plot final salvo em trajetoria_final.html")


if __name__ == "__main__":
    main()
