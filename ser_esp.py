import csv
import serial  # Importa a biblioteca pyserial

ser = serial.Serial('COM6', 9600)  # Conecta à porta serial

# Abre o arquivo CSV para escrita
with open('dados_arduino.csv', mode='w', newline='') as file:
    writer = csv.writer(file)
    # Escreve o cabeçalho do arquivo CSV
    writer.writerow(['Motor1', 'Motor2', 'Motor3', 'Motor4'])

    try:
        while True:
            # Lê uma linha de dados da serial
            line = ser.readline().decode('utf-8').strip()
            print(line)  # Exibe o dado recebido

            # Verifica se a linha recebida contém os valores dos motores
            if "Motor1:" in line:
                # Divide a linha recebida por vírgula para separar os valores dos motores
                values = line.split(',')
                
                # Extrai os valores de cada motor
                motor1 = values[0].split(':')[1]
                motor2 = values[1].split(':')[1]
                motor3 = values[2].split(':')[1]
                motor4 = values[3].split(':')[1]
                
                # Grava os valores dos motores no arquivo CSV
                writer.writerow([motor1, motor2, motor3, motor4])

    except KeyboardInterrupt:
        print("Interrupção pelo usuário")
    finally:
        ser.close()
