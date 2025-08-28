import zmq
import msgpack
import time
import serial
import struct

PROXY_ADDRESS = "tcp://localhost:5559"
DATA_TOPIC = b'sensor/imu'
STATUS_TOPIC = b'flags/imu'
SERIAL_PORT = "/dev/imu"
BAUD_RATE = 19200
RETRY_DELAY = 0.2

def initialize_imu():

    try:
        print(f"Tentando conectar ao IMU na porta {SERIAL_PORT}...")
        serial_conn = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=None)
        print("Conectado ao IMU com sucesso.")
        return serial_conn
    except serial.SerialException as e:
        print(f"ERRO CRÍTICO ao abrir a porta serial do IMU: {e}")
        return None

def publish_status(socket, topic, current_status, previous_status):

    if current_status != previous_status:
        status_message = {
            'timestamp_ns': time.time_ns(),
            'status': current_status
        }
        socket.send_multipart([topic, msgpack.packb(status_message)])
        return current_status

def get_imu_packet_blocking(serial_conn):
    
    while True:
        if serial_conn.read(1) == b'\x55':
            if serial_conn.read(1) == b'\x53':
                payload = serial_conn.read(9)
                leitura_timestamp_ns = time.time_ns()
                packet = b'\x55\x53' + payload
                if sum(packet[:10]) & 0xFF == packet[10]:
                    roll_raw, pitch_raw, yaw_raw = struct.unpack('<hhh', packet[2:8])
                    euler_angles = {
                        "roll": roll_raw / 32768.0 * 180.0,
                        "pitch": pitch_raw / 32768.0 * 180.0,
                        "yaw": yaw_raw / 32768.0 * 180.0,
                    }
                    return euler_angles, leitura_timestamp_ns

def main():
    
    context = zmq.Context()
    publisher_socket = context.socket(zmq.PUB)
    publisher_socket.connect(PROXY_ADDRESS)
    
    previous_status = None
    imu_connection = None
    
    print("Serviço do Publisher IMU iniciado.")

    try:
        while True:
            if imu_connection is None or not imu_connection.is_open:
                if imu_connection:
                    imu_connection.close()
                imu_connection = initialize_imu()

            if imu_connection:
                previous_status = publish_status(publisher_socket, STATUS_TOPIC, 'active', previous_status)
                try:
                    while True:
                        euler_data, timestamp_data = get_imu_packet_blocking(imu_connection)
                        message_package = {
                            'timestamp_ns': timestamp_data,
                            'data': euler_data
                        }
                        publisher_socket.send_multipart([DATA_TOPIC, msgpack.packb(message_package)])
                        
                except serial.SerialException as e:
                    print(f"ERRO DE CONEXÃO: {e}. O sensor foi desconectado.")
                    imu_connection.close()
                    imu_connection = None
                    previous_status = publish_status(publisher_socket, STATUS_TOPIC, 'disabled', previous_status)
            else:
                previous_status = publish_status(publisher_socket, STATUS_TOPIC, 'disabled', previous_status)
                print(f"Falha ao conectar ao IMU. Tentando novamente em {RETRY_DELAY} segundos...")
                time.sleep(RETRY_DELAY)

    except KeyboardInterrupt:
        print("\nServiço encerrado pelo usuário.")
    finally:
        if imu_connection and imu_connection.is_open:
            imu_connection.close()
        publisher_socket.close()
        context.term()

if __name__ == "__main__":
    main()
