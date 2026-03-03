import cv2


def listar_cameras_disponiveis(max_testes=10):
    """
    Testa os primeiros 'max_testes' índices para encontrar câmeras conectadas.
    Retorna uma lista com os índices que funcionaram.
    """
    indices_disponiveis = []

    print(f"Verificando os primeiros {max_testes} índices de câmera...\n")

    for index in range(max_testes):
        # Tenta abrir a câmera com o índice atual
        cap = cv2.VideoCapture(index)

        if cap.isOpened():
            # Tenta ler um frame para garantir que está funcionando mesmo
            ret, _ = cap.read()
            if ret:
                print(f"[SUCESSO] Câmera encontrada no índice: {index}")
                indices_disponiveis.append(index)
            else:
                print(f"[ALERTA] Índice {index} abriu, mas não conseguiu ler o frame.")

            # É importante liberar a câmera após o teste
            cap.release()
        else:
            # Opcional: imprimir falhas (muitas vezes desnecessário poluir o log)
            # print(f"[FALHA] Nenhuma câmera no índice: {index}")
            pass

    print("\n--- Resumo ---")
    if indices_disponiveis:
        print(f"Câmeras disponíveis nos índices: {indices_disponiveis}")
    else:
        print("Nenhuma câmera foi encontrada.")

    return indices_disponiveis


if __name__ == "__main__":
    listar_cameras_disponiveis()
