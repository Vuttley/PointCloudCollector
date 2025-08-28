import zmq
import msgpack
import time
import subprocess
import re

PROXY_ADDRESS = "tcp://localhost:5559"
DATA_TOPIC = b'sensor/lidar/chunk'
STATUS_TOPIC = b'flags/lidar'
SERIAL_PORT = "/dev/lidar"
CHUNK_SIZE = 360
RETRY_DELAY = 0.2

def initialize_lidar():
    try:
        print("Tentando iniciar o processo do LiDAR...")
        process = subprocess.Popen([
            "/home/admin/Documents/stl27lsdk/sdk_ldrobotsensorteam_stl_v3.0.5_stable_20230203-18-10/linux_app/build/ldlidar_stl",
            "STL27L", "serialcom", SERIAL_PORT
        ], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, bufsize=1)
        print("Processo do LiDAR iniciado com sucesso.")
        return process
    except (FileNotFoundError, Exception) as e:
        print(f"ERRO CRÍTICO ao iniciar o LiDAR: {e}")
        return None

def publish_status(socket, topic, current_status, previous_status):
    if current_status != previous_status:
        status_message = {
            'timestamp_ns': time.time_ns(),
            'status': current_status
        }
        socket.send_multipart([topic, msgpack.packb(status_message)])
        return current_status

def get_point_chunk_blocking(lidar_process, chunk_size):
    points_chunk = []
    for line in lidar_process.stdout:
        if match := re.search(r"angle:([\d.]+),distance\(mm\):(\d+),intensity:(\d+)", line):
            point_data = {
                'angle': float(match.group(1)),
                'distance': int(match.group(2)),
                'intensity': int(match.group(3)),
                'timestamp_ns': time.time_ns()
            }
            points_chunk.append(point_data)
            if len(points_chunk) == chunk_size:
                return points_chunk
    return None

def main():
    context = zmq.Context()
    publisher_socket = context.socket(zmq.PUB)
    publisher_socket.connect(PROXY_ADDRESS)
    
    previous_status = None
    lidar_process = None
    
    print("Serviço do Publisher Lidar iniciado.")

    try:
        while True:
            if lidar_process is None or lidar_process.poll() is not None:
                lidar_process = initialize_lidar()

            if lidar_process:
                previous_status = publish_status(publisher_socket, STATUS_TOPIC, 'active', previous_status)

                chunk = get_point_chunk_blocking(lidar_process, CHUNK_SIZE)
                if chunk:
                    message_package = {'points': chunk}
                    publisher_socket.send_multipart([DATA_TOPIC, msgpack.packb(message_package)])
                else:
                    print("Processo do LiDAR foi encerrado inesperadamente. Tentando reiniciar...")
                    lidar_process = None
                    previous_status = publish_status(publisher_socket, STATUS_TOPIC, 'disabled', previous_status)
            else:
                previous_status = publish_status(publisher_socket, STATUS_TOPIC, 'disabled', previous_status)
                print(f"Falha ao iniciar o LiDAR. Tentando novamente em {RETRY_DELAY} segundos...")
                time.sleep(RETRY_DELAY)

    except KeyboardInterrupt:
        print("\nServiço encerrado pelo usuário.")
    finally:
        if lidar_process:
            lidar_process.kill()
        publisher_socket.close()
        context.term()

if __name__ == "__main__":
    main()
