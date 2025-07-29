import pandas as pd
import matplotlib.pyplot as plt
import os

FILE_NAME = 'datalog.csv'

def plotar_dados(file_path):
    if not os.path.exists(file_path):
        print(f"Erro: Arquivo '{file_path}' não encontrado!")
        return

    print(f"Lendo dados de '{file_path}'...")
    try:
        df = pd.read_csv(file_path)
    except Exception as e:
        print(f"Erro ao ler o arquivo CSV: {e}")
        return
    df['tempo'] = pd.to_datetime(df['data_hora'], format='%d/%m/%Y-%H:%M:%S')


    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
    fig.suptitle('Análise de Dados do Datalogger IMU', fontsize=16)

    # Gráfico Acelerômetro
    ax1.plot(df['tempo'], df['accel_x'], label='Eixo X', color='r')
    ax1.plot(df['tempo'], df['accel_y'], label='Eixo Y', color='g')
    ax1.plot(df['tempo'], df['accel_z'], label='Eixo Z', color='b')
    ax1.set_title('Dados do Acelerômetro')
    ax1.set_ylabel('Aceleração (g)')
    ax1.legend()
    ax1.grid(True)

    # Gráfico Giroscópio 
    ax2.plot(df['tempo'], df['giro_x'], label='Eixo X', color='r')
    ax2.plot(df['tempo'], df['giro_y'], label='Eixo Y', color='g')
    ax2.plot(df['tempo'], df['giro_z'], label='Eixo Z', color='b')
    ax2.set_title('Dados do Giroscópio')
    ax2.set_ylabel('Velocidade Angular (°/s)')
    ax2.set_xlabel('Tempo')
    ax2.legend()
    ax2.grid(True)

    plt.gcf().autofmt_xdate()

    plt.tight_layout(rect=[0, 0, 1, 0.96])
    print("Gráficos gerados com sucesso! Exibindo a janela de plotagem.")
    plt.show()

if __name__ == "__main__":
    plotar_dados(FILE_NAME)