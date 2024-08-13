#import leitura_ser_arduino
import csv
import serial  # Importa a biblioteca pyserial

ser = serial.Serial('COM6', 9600)  # Conecta à porta serial
# Configura a porta serial (substitua 'COM6' pelo nome da sua porta)


# Abre o arquivo CSV para escrita
with open('dados_arduino.csv', mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(['Dado'])  # Cabeçalho do arquivo CSV

    try:
        while True:
            # Lê uma linha de dados da serial
            line = ser.readline().decode('utf-8').strip()
            print(line)  # Exibe o dado recebido

            # Grava o dado no arquivo CSV
            writer.writerow([line])

    except KeyboardInterrupt:
        print("Interrupção pelo usuário")
    finally:
        ser.close()
