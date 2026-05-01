import math
import csv
import matplotlib.pyplot as plt

# =================================================================
# SUAS CLASSES ORIGINAIS
# =================================================================
class odometry():
    def __init__(self, el, er, L):
        self.el = el
        self.er = er
        self.L = L
        self.wl_last_counter = 0
        self.wr_last_counter = 0
        
        self.x = 0
        self.y = 0
        self.theta = 0
        
        self.meters_per_tick_left = (2 * math.pi * self.el.radius) / (el.ticks_p_revol)
        self.meters_per_tick_right = (2 * math.pi * self.er.radius) / (er.ticks_p_revol)

    def step(self):
        delta_ticks_left = (self.el.counter - self.wl_last_counter)
        delta_ticks_right = (self.er.counter - self.wr_last_counter)
        
        self.wl_last_counter = self.el.counter
        self.wr_last_counter = self.er.counter
        
        Dl = self.meters_per_tick_left * delta_ticks_left
        Dr = self.meters_per_tick_right * delta_ticks_right
        Dc = (Dr + Dl) / 2
        
        x_dt = Dc * math.cos(self.theta)
        y_dt = Dc * math.sin(self.theta)
        theta_dt = (Dr - Dl) / self.L

        self.x = self.x + x_dt
        self.y = self.y + y_dt
        self.theta = self.theta + theta_dt
        
        return self.x, self.y, self.theta
        
    def getPose(self):
        return self.x, self.y, self.theta

# =================================================================
# CLASSE AUXILIAR PARA O PLOT
# =================================================================
class EncoderSimulado:
    def __init__(self, ticks_p_revol, radius):
        self.ticks_p_revol = ticks_p_revol
        self.radius = radius
        self.counter = 0

# =================================================================
# SCRIPT DE PROCESSAMENTO E PLOTAGEM
# =================================================================
def plot_odometry_from_csv(csv_filepath):
    # Parâmetros físicos do robô
    WHEEL_RADIUS = 0.06
    WHEEL_BASE = 0.335
    TICKS_PER_REVOLUTION = 980

    # Inicializa os encoders simulados e a odometria
    el = EncoderSimulado(TICKS_PER_REVOLUTION, WHEEL_RADIUS)
    er = EncoderSimulado(TICKS_PER_REVOLUTION, WHEEL_RADIUS)
    odom = odometry(el, er, WHEEL_BASE)

    # Listas para armazenar a trajetória
    x_history = []
    y_history = []

    print(f"Lendo dados de: {csv_filepath}")
    
    try:
        with open(csv_filepath, mode='r') as file:
            reader = csv.reader(file)
            
            # Se o seu CSV tiver cabeçalho (ex: timestamp, left, right), descomente a linha abaixo:
            # next(reader) 

            for row in reader:
                if not row: continue # Pula linhas vazias
                
                # Formato esperado: timestamp_ns, left_ticks, right_ticks
                timestamp_ns = int(row[0])
                left_ticks = int(row[1])
                right_ticks = int(row[2])

                # Atualiza os contadores dos encoders
                el.counter = left_ticks
                er.counter = right_ticks

                # Calcula o próximo passo da odometria
                odom.step()
                x, y, theta = odom.getPose()

                # Salva a posição para o plot
                x_history.append(x)
                y_history.append(y)

    except FileNotFoundError:
        print(f"Erro: Arquivo '{csv_filepath}' não encontrado.")
        return

    # Gerando o gráfico
    plt.figure(figsize=(10, 8))
    plt.plot(x_history, y_history, marker='.', linestyle='-', markersize=2, label="Trajetória (Wheel Odometry)", color='b')
    
    # Marca o ponto de início e fim
    if x_history:
        plt.scatter(x_history[0], y_history[0], color='green', s=100, label='Início', zorder=5)
        plt.scatter(x_history[-1], y_history[-1], color='red', s=100, label='Fim', zorder=5)

    plt.title('Odometria do Robô baseada no encoder.csv')
    plt.xlabel('Posição X (metros)')
    plt.ylabel('Posição Y (metros)')
    plt.axis('equal') # Mantém a proporção dos eixos X e Y igual para não distorcer a curva
    plt.grid(True, linestyle='--', alpha=0.7)
    plt.legend()
    plt.show()

if __name__ == "__main__":
    # Ajuste o caminho para a pasta onde o seu dataset foi salvo
    CAMINHO_DO_CSV = "/home/aki/Desktop/old_cuscobot/dataset_20260321_115430/sensor_data/encoder.csv" 
    plot_odometry_from_csv(CAMINHO_DO_CSV)