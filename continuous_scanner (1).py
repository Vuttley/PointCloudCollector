import zmq
import msgpack
import time
import json
from collections import deque
from adafruit_pca9685 import PCA9685
from board import SCL, SDA
import busio
import numpy as np
import threading
from datetime import datetime
import os
import sys

class ContinuousScanner:
    
    def __init__(self, movement_plan, output_folder):

        self.movement_plan = deque(movement_plan)
        self.output_folder = output_folder
        self.data_queue = deque(maxlen=50000)
        self.running = True
        self.pause = False
        self.last_data_timestamp = time.time()
        self.WATCHDOG_TIMEOUT_S = 15.0

        try:
            self.i2c = busio.I2C(SCL, SDA)
            self.pca = PCA9685(self.i2c)
            self.pca.frequency = 50
        except Exception as e:
            print(f"ERRO CRÍTICO: Falha ao inicializar hardware: {e}")
            self.running = False
            return
            
        self.context = zmq.Context()
        self.subscriber = self.context.socket(zmq.SUB)
        self.subscriber.connect("tcp://localhost:5560")
        self.subscriber.setsockopt(zmq.SUBSCRIBE, b'sensor/')
        self.subscriber.setsockopt(zmq.SUBSCRIBE, b'flags/')
        
        self.current_base_pos = 1.0
        self.current_head_pos = 0.5
        
        self.data_thread = threading.Thread(target=self._data_collection_loop, daemon=True)

    def _data_collection_loop(self):

        while self.running:
            try:
                topic, payload = self.subscriber.recv_multipart()
                msg = msgpack.unpackb(payload)

                if topic.startswith(b'flags/'):
                    status = msg.get('status')
                    sensor_name = topic.split(b'/')[1].decode('utf-8')
                    if status == 'disabled':
                        print(f"WARN: Publisher '{sensor_name}' reportou status 'disabled'. Pausando o sistema.")
                        self.pause = True
                    if self.pause == True:
                        if status == 'active':
                            print(f"INFO: Publisher '{sensor_name}' reportou status 'active'. Retomando operações.")
                            self.pause = False
                else:
                    self.last_data_timestamp = time.time()
                    self.data_queue.append((topic, msg))

            except zmq.ZMQError as e:
                if not self.running or e.errno == zmq.ETERM:
                    break 
                else:
                    print(f"ERRO INESPERADO: Erro na thread de dados ZMQ: {e}")
                    break
        print("Thread de coleta de dados encerrada.")
        
    def _process_data_queue(self, f_imu, f_lidar):

        if not f_imu or not f_lidar:
            self.data_queue.clear()
            return
        
        try:
            while True:
                topic, msg = self.data_queue.popleft()
                if topic == b'sensor/imu':
                    f_imu.write(json.dumps(msg) + '\n')
                elif topic == b'sensor/lidar/chunk':
                    for point in msg.get('points', []):
                        f_lidar.write(json.dumps(point) + '\n')
        except IndexError:
            pass 
        
    def run(self, time_per_move_s=0.6):

        if not self.running:
            print("Scanner em estado de erro. Encerrando.")
            return

        print("Iniciando a captura de dados brutos...")
        self.data_thread.start()
        time.sleep(1)

        f_imu = None
        f_lidar = None
        pan_count = 0
        
        try:
            print(f"Iniciando plano de movimento... Salvando resultados em '{self.output_folder}'")
            for move in self.movement_plan:
                
                while self.pause:
                    print("Sistema pausado. Aguardando sinal 'active' dos publishers...")
                    self.last_data_timestamp = time.time()
                    time.sleep(2)

                if time.time() - self.last_data_timestamp > self.WATCHDOG_TIMEOUT_S:
                    print(f"\nERRO CRÍTICO (WATCHDOG): Nenhum dado recebido por mais de {self.WATCHDOG_TIMEOUT_S} segundos.")
                    print("Isso pode indicar que os publishers travaram ou foram encerrados.")
                    print("Encerrando com código de erro para acionar o reinício pelo systemd.")
                    sys.exit(1)

                base_target, head_target = move

                if base_target is not None:
                    if f_imu: f_imu.close()
                    if f_lidar: f_lidar.close()
                    
                    self._process_data_queue(f_imu, f_lidar)
                    pan_count += 1
                    
                    imu_filename = os.path.join(self.output_folder, f"raw_imu_pan_{pan_count}.txt")
                    lidar_filename = os.path.join(self.output_folder, f"raw_lidar_pan_{pan_count}.txt")
                    
                    print(f"\n--- INICIANDO PAN #{pan_count} ---")
                    print(f"Salvando dados em '{imu_filename}' e '{lidar_filename}'")
                    
                    f_imu = open(imu_filename, 'w')
                    f_lidar = open(lidar_filename, 'w')
                    
                    self._move_base(base_target)

                if head_target is not None:
                    self._move_head(head_target)
                
                time.sleep(time_per_move_s)
                self._process_data_queue(f_imu, f_lidar)

            print("\nPlano de movimento finalizado.")

        except KeyboardInterrupt:
            print("\nInterrupção pelo usuário. Encerrando.")
        finally:
            print("Encerrando captura...")
            self.running = False
            self.data_thread.join(timeout=2.0)
            
            self._process_data_queue(f_imu, f_lidar)

            if f_imu: f_imu.close()
            if f_lidar: f_lidar.close()

            time.sleep(1)
            self._move_base(1)
            time.sleep(3)
            self._move_head(0.5)

            self.subscriber.close()
            self.context.term()
            print(f"Processo finalizado. {pan_count} scans de PAN foram salvos em '{self.output_folder}'.")

    def _set_servo_pulse(self, channel, pulse_width):

        duty_cycle = int((pulse_width / 20000) * 0xFFFF)
        self.pca.channels[channel].duty_cycle = duty_cycle
        
    def _stop_servo(self, channel):

        self.pca.channels[channel].duty_cycle = 0
        
    def _move_base(self, value, move_time_s=0.4):

        pulse = 500 + value * 2000
        self._set_servo_pulse(1, pulse)
        time.sleep(move_time_s)
        self._stop_servo(1)
        
    def _move_head(self, value, move_time_s=0.4):

        pulse = 1200 + value * 1000
        self._set_servo_pulse(0, pulse)
        time.sleep(move_time_s)
        self._stop_servo(0)


def gerar_plano_de_scan_dinamico(num_pontos_desejado, tipo, passo_min=0.1, passo_max=0.25):

    if num_pontos_desejado <= 0: 
        return []
    posicao_atual = 1.0 
    direcao = -1 
    sequencia_pontos = [posicao_atual]
    while len(sequencia_pontos) < num_pontos_desejado:
        passo = np.random.uniform(passo_min, passo_max)
        proxima_posicao = posicao_atual + direcao * passo
        if proxima_posicao >= 1.0: 
            direcao = -1
        elif proxima_posicao <= 0.0: 
            direcao = 1
        else: 
            posicao_atual = proxima_posicao
            sequencia_pontos.append(round(posicao_atual, 2))
    if tipo=='tilt': 
        return [[None, p] for p in sequencia_pontos]
    else: 
        return [[p, None] for p in sequencia_pontos]

def gerar_plano_com_marcadores(num_pontos_pan, num_pontos_tilt):
    
    sequencia_tilt = gerar_plano_de_scan_dinamico(num_pontos_tilt, 'tilt')
    sequencia_pan = gerar_plano_de_scan_dinamico(num_pontos_pan, 'pan')
    plano_final = []
    for pan in sequencia_pan:
      plano_final.append(pan)
      plano_final.extend(sequencia_tilt)
    return plano_final

if __name__ == '__main__':

    timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    output_directory = f"scan_results_{timestamp}"
    
    os.makedirs(output_directory, exist_ok=True)
    print(f"Diretório de saída criado: {output_directory}")
    
    scan_plan = gerar_plano_com_marcadores(num_pontos_pan=10, num_pontos_tilt=300)
    
    scanner = ContinuousScanner(movement_plan=scan_plan, output_folder=output_directory)
    scanner.run(time_per_move_s=0.4)
