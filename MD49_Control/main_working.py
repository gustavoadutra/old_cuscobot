import time
import matplotlib.pyplot as plt
import numpy as np
import pygame
import encoder, odometry, serialCom, goToGoal
import plotly.graph_objects as go

# --- Configurações Iniciais ---
PORT = "/dev/ttyACM0"
BAUDRATE = 9600
TIMEOUT = 0.1

WHEEL_RADIUS = 0.06
WHEEL_BASE = 0.335
TICKS_PER_REVOLUTION = 980
MAX_PWM = 48
MAX_PWM_STEP = 10
MAX_SPEED_DISTANCE = 1

# --- Inicialização do Pygame (Substitui o keyboard) ---
pygame.init()
# Criamos uma janela pequena para capturar os eventos de teclado sem sudo
screen = pygame.display.set_mode((200, 100))
pygame.display.set_caption("Controle Robô")

# --- Objetos e Comunicação ---
left_wheel_encoder = encoder.encoder(TICKS_PER_REVOLUTION, WHEEL_RADIUS)
right_wheel_encoder = encoder.encoder(TICKS_PER_REVOLUTION, WHEEL_RADIUS)

arduino = serialCom.Communication(PORT, BAUDRATE, TIMEOUT)
arduino.open()
time.sleep(3)

clock = pygame.time.Clock()
FPS = 100

odom = odometry.odometry(left_wheel_encoder, right_wheel_encoder, WHEEL_BASE)
controller = goToGoal.GoToGoal()

# --- Variáveis Auxiliares ---
start_time = time.time_ns()
last_read = time.time()
last_left_pwm = 128
last_right_pwm = 128
pose_log = {"x": [], "y": [], "theta": []}

# --- Configuração do Plot Matplotlib ---
fig_plt, ax = plt.subplots()
line = ax.scatter(pose_log["x"], pose_log["y"])
plt.axis([-2, 2, -2, 2])
plt.show(block=False)
plt.pause(0.1)

# --- Caminho (Path Planning) ---
PATH = [[0, 1], [1, 1], [1, 0], [0, 0]]
step = 0
goal = PATH[step]
plt.scatter(goal[0], goal[1], marker="x", color="r")

# --- Loop Principal ---
end = False
last_w = 0

print("Robô iniciado. Pressione 'Q' na janela do Pygame para parar.")

while not end:
    # 1. Captura de Eventos (Substitui keyboard.is_pressed)
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            end = True
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_q:
                print("Interrupção via teclado detectada.")
                end = True

    if end:
        break

    # 2. Leitura dos Encoders
    le = arduino.get_encoder_left()
    ld = arduino.get_encoder_right()

    if le and ld:
        right_wheel_encoder.counter = int(ld)
        left_wheel_encoder.counter = int(le)
        last_read = time.time()
    elif time.time() - last_read > 10:
        print("Muito tempo sem ler arduino, encerrando...")
        break

    # 3. Odometria e Tempo
    t = time.time_ns()
    dt = t - start_time
    start_time = t

    odom.step()
    x, y, theta = odom.getPose()
    pose_log["x"].append(x)
    pose_log["y"].append(y)
    pose_log["theta"].append(theta)

    # 4. Controle PID (Go to Goal)
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

    # 5. Cinemática Diferencial e PWM
    left, right = odometry.uni_to_diff(
        5, w, left_wheel_encoder, right_wheel_encoder, WHEEL_BASE
    )

    # Normalização
    if left > right:
        left_norm = left / left if left != 0 else 0
        right_norm = right / left if left != 0 else 0
    else:
        left_norm = left / right if right != 0 else 0
        right_norm = right / right if right != 0 else 0

    max_speed = controller.speed_limit_by_distance(
        MAX_SPEED_DISTANCE, MAX_PWM, goal[0], goal[1], x, y
    )
    left_pwm = (left_norm * max_speed) + 128
    right_pwm = (right_norm * max_speed) + 128

    # Suavização do PWM (Rampa)
    left_dif = left_pwm - last_left_pwm
    right_dif = right_pwm - last_right_pwm

    last_left_pwm += max(min(left_dif, MAX_PWM_STEP), -MAX_PWM_STEP)
    last_right_pwm += max(min(right_dif, MAX_PWM_STEP), -MAX_PWM_STEP)

    # Envio para o Arduino
    arduino.set_speed_left(int(last_left_pwm))
    arduino.set_speed_right(int(last_right_pwm))

    # 6. Atualização do Gráfico em Tempo Real
    line.set_offsets(np.c_[pose_log["x"], pose_log["y"]])
    fig_plt.canvas.draw()
    fig_plt.canvas.flush_events()

    clock.tick(FPS)

# --- Finalização ---
arduino.set_speed_left(128)
arduino.set_speed_right(128)
arduino.close()

# Gráfico Final (Plotly)
fig_go = go.Figure()
fig_go.add_trace(
    go.Scatter(
        x=pose_log["x"], y=pose_log["y"], mode="lines+markers", name="Trajetória"
    )
)
fig_go.update_layout(
    title="Trajetória Final do Robô", xaxis_title="X (m)", yaxis_title="Y (m)"
)
fig_go.show()

pygame.quit()
plt.show()
