import pandas as pd
import numpy as np
from scipy.spatial.transform import Rotation as R


def convert_to_loader_format(input_file, output_file):
    print(f"Lendo: {input_file}")
    try:
        # Lê global_pose.csv (sem cabeçalho)
        # Formato: timestamp, r00, r01, r02, tx, r10, r11, r12, ty, r20, r21, r22, tz
        df = pd.read_csv(input_file, header=None)
    except Exception as e:
        print(f"Erro: {e}")
        return

    output_data = []
    print("Convertendo dados e calculando Heading (Yaw)...")

    for index, row in df.iterrows():
        # --- 1. Timestamp ---
        timestamp = int(row[0])

        # --- 2. Posição (tx, ty, tz) ---
        tx = row[4]
        ty = row[8]
        tz = row[12]

        # --- 3. Calcular Heading (Yaw) em Graus ---
        # Reconstrói a matriz de rotação 3x3
        rot_matrix = np.array(
            [
                [row[1], row[2], row[3]],
                [row[5], row[6], row[7]],
                [row[9], row[10], row[11]],
            ]
        )

        # Converte Matriz -> Euler (Yaw/Heading é geralmente o eixo Z ou Y dependendo do frame)
        # O dataset KAIST/Urban geralmente usa Z-up ou Y-down.
        # Vamos extrair Euler no padrão ZYX e pegar o primeiro ângulo (Z) como Heading.
        r = R.from_matrix(rot_matrix)
        euler = r.as_euler("zyx", degrees=True)
        heading_degrees = euler[0]  # Yaw em graus

        # --- 4. Montar a linha para o formato vrs_gps (14 colunas) ---
        # Índices baseados no seu código ComplexUrbanDatasetLoader:
        # [0]=Timestamp
        # [3]=X, [4]=Y, [5]=Z
        # [12]=Flag (tem que ser 1), [13]=Heading(Graus)

        new_row = [
            timestamp,  # 0: Timestamp
            0,  # 1: Ignorado pelo loader
            0,  # 2: Ignorado pelo loader
            tx,  # 3: X (Loader lê gps_x_raw aqui)
            ty,  # 4: Y (Loader lê gps_y_raw aqui)
            tz,  # 5: Z (Loader lê gps_z_raw aqui)
            0,
            0,
            0,  # 6, 7, 8: Covariância (Ignorado)
            0,
            0,
            0,  # 9, 10, 11: Covariância (Ignorado)
            1,  # 12: Status Flag (OBRIGATÓRIO SER 1 para ler rotação)
            heading_degrees,  # 13: Heading em graus
        ]

        output_data.append(new_row)

    # Cria DataFrame e Salva
    # Header=None pois o loader usa header=None
    df_out = pd.DataFrame(output_data)
    df_out.to_csv(output_file, index=False, header=False)

    print(f"Salvo em: {output_file}")
    print("Exemplo da primeira linha (verifique colunas 3, 4, 5, 12 e 13):")
    # Mostra com formatação de lista para facilitar contagem
    print(list(df_out.iloc[0].values))


# --- Executar ---
convert_to_loader_format(
    "/home/aki/Desktop/Datasets/test_dataset/global_pose.csv",
    "/home/aki/Desktop/Datasets/test_dataset/vrs_gps.csv",
)
