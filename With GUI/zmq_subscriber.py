import zmq
import msgpack
import time
from collections import deque
from adafruit_pca9685 import PCA9685
from board import SCL, SDA
import busio
import numpy as np
import csv
import threading
from datetime import datetime
import queue

class ScanController:
    
    def __init__(self, movement_plan, output_filename="scan_data.csv"):
        self.state = 'IDLE'
        self.is_first_run = True
        self.idle_angles = {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}
        self.moving_angles = None
        #self.drift_rate_yaw = 0.0 -> sensor parece estável agora
        self.movement_plan = deque(movement_plan)
        self.current_move = None
        self.buffer_time = 4
        self.imu_freq = 98
        self.imu_buffer = deque(maxlen=self.buffer_time*self.imu_freq)
        self.lidar_buffer = deque(maxlen=self.buffer_time*21600)
        self.final_point_cloud = []
        self.pause = False
        self.output_filename = output_filename
        self.MAX_IMU_GAP_NS = 100_000_000

        self.processing_queue = queue.Queue() 
        self.point_cloud_lock = threading.Lock() 

        try:
            self.i2c = busio.I2C(SCL, SDA)
            self.pca = PCA9685(self.i2c)
            self.pca.frequency = 50
        except Exception as e:
            print(f"ERRO CRÍTICO: Falha ao inicializar hardware: {e}")
            self.state = 'ERROR'
            return
            
        self.context = zmq.Context()
        self.subscriber = self.context.socket(zmq.SUB)
        self.subscriber.connect("tcp://localhost:5560")
        self.subscriber.setsockopt(zmq.SUBSCRIBE, b'sensor/')
        self.subscriber.setsockopt(zmq.SUBSCRIBE, b'flags/')
        self.running = True
        self.data_thread = threading.Thread(target=self._data_ingestion_loop, daemon=True)
        self.processing_thread = threading.Thread(target=self._processing_worker_loop, daemon=True)

    def _processing_worker_loop(self):
        while self.running or not self.processing_queue.empty():
            try:
                job = self.processing_queue.get(timeout=1.0)
                if job is None:
                    break
                
                processed_points = self._process_moving_data_vectorized(
                    lidar_data=job['lidar_data'],
                    imu_data=job['imu_data'],
                    current_move=job['current_move'],
                    moving_angles=job['moving_angles'],
                    idle_angles=job['idle_angles']
                )
                
                if processed_points:
                    with self.point_cloud_lock:
                        self.final_point_cloud.extend(processed_points)
                
                self.processing_queue.task_done()

            except queue.Empty:
                continue

    def _process_moving_data_vectorized(self, lidar_data, imu_data, current_move, moving_angles, idle_angles):
        if not lidar_data or not imu_data or len(imu_data) < 2:
            return []

        lidar_timestamps_ns = np.array([p['timestamp_ns'] for p in lidar_data])
        lidar_distances = np.array([p['distance'] for p in lidar_data])
        lidar_angles = np.array([p['angle'] for p in lidar_data])
        lidar_intensities = np.array([p['intensity'] for p in lidar_data])

        imu_timestamps_ns = np.array([m['timestamp_ns'] for m in imu_data])
        imu_pitches = np.array([m['data']['pitch'] for m in imu_data])
        imu_yaws_unwrapped = np.unwrap([m['data']['yaw'] for m in imu_data], period=360)

        indices = np.searchsorted(imu_timestamps_ns, lidar_timestamps_ns, side='right')
        valid_indices_mask = (indices > 0) & (indices < len(imu_timestamps_ns))
        
        valid_time_gap_mask = np.zeros_like(lidar_timestamps_ns, dtype=bool)

        time_gaps = (imu_timestamps_ns[indices[valid_indices_mask]] - 
                     imu_timestamps_ns[indices[valid_indices_mask] - 1])

        valid_time_gap_mask[valid_indices_mask] = time_gaps < self.MAX_IMU_GAP_NS

        final_valid_mask = valid_time_gap_mask
        
        if not np.any(final_valid_mask):
            return []

        lidar_timestamps_ns = lidar_timestamps_ns[final_valid_mask]
        lidar_distances = lidar_distances[final_valid_mask]
        lidar_angles = lidar_angles[final_valid_mask]
        lidar_intensities = lidar_intensities[final_valid_mask]

        final_pitches = np.interp(lidar_timestamps_ns, imu_timestamps_ns, imu_pitches)
        final_yaws = np.interp(lidar_timestamps_ns, imu_timestamps_ns, imu_yaws_unwrapped)
        final_rolls = np.full_like(lidar_timestamps_ns, idle_angles['roll'])

        local_results = []
        for i in range(len(lidar_timestamps_ns)):
            csv_row = {
                'distancia_lidar': lidar_distances[i],
                'angulo_lidar': lidar_angles[i],
                'intensidade_lidar': lidar_intensities[i],
                'roll': final_rolls[i],
                'pitch': final_pitches[i],
                'yaw': (final_yaws[i] + 180) % 360 - 180,
                'estado': 'movimento',
                'timestamp': lidar_timestamps_ns[i],
                'timestamp_imu': lidar_timestamps_ns[i],
                'movimento_base': current_move[0],
                'movimento_cabeca': current_move[1],
            }
            local_results.append(csv_row)
            
        return local_results

    def _handle_moving_state(self):
        stop_angles, stop_time_ns = self._determine_move_stop(2.0)

        if stop_angles:
            job_data = {
                'imu_data': list(self.imu_buffer),
                'lidar_data': list(self.lidar_buffer),
                'current_move': self.current_move,
                'moving_angles': self.moving_angles,
                'idle_angles': self.idle_angles.copy()
            }
            self.imu_buffer.clear()
            self.lidar_buffer.clear()
            
            self.processing_queue.put(job_data)
            
            self.state = 'IDLE'

    def _process_stopped_data(self, imu_snapshot, lidar_snapshot):
        MAX_POINTS_PER_STOP = 4320
        
        if not imu_snapshot or not lidar_snapshot:
            return

        base_move_target = self.current_move[0] if self.current_move else None
        head_move_target = self.current_move[1] if self.current_move else None
        
        all_processed_points = []
        for lidar_point in lidar_snapshot:
            lidar_ts = lidar_point['timestamp_ns']
            closest_imu = min(imu_snapshot, key=lambda m: abs(m['timestamp_ns'] - lidar_ts))
            point_orientation = closest_imu['data']
            
            csv_row = {
                'distancia_lidar': lidar_point['distance'], 'angulo_lidar': lidar_point['angle'],
                'intensidade_lidar': lidar_point['intensity'], 'roll': point_orientation['roll'],
                'pitch': point_orientation['pitch'], 'yaw': point_orientation['yaw'],
                'estado': 'parado', 'timestamp': lidar_point['timestamp_ns'],
                'timestamp_imu': closest_imu['timestamp_ns'], 'movimento_base': base_move_target,
                'movimento_cabeca': head_move_target,
            }
            all_processed_points.append(csv_row)
            
        final_points_to_add = all_processed_points[-MAX_POINTS_PER_STOP:]
        
        if final_points_to_add:
            with self.point_cloud_lock:
                self.final_point_cloud.extend(final_points_to_add)

    def run(self):
        if self.state == 'ERROR':
            print("Controlador em estado de erro. Encerrando.")
            return
            
        self.data_thread.start()
        self.processing_thread.start()

        try:
            while self.state != 'FINISHED' and self.running:
                self._run_state_machine()
                time.sleep(0.01)
        except KeyboardInterrupt:
            print("\nInterrupção pelo usuário. Encerrando.")
        except Exception as e:
            print(f"ERRO INESPERADO no loop principal: {e}")
            self.state = 'ERROR'
        finally:
            print("Loop principal encerrado. Aguardando processamento final...")
            self.running = False
            
            print("Aguardando a conclusão do processamento em background...")
            self.processing_queue.join() # <--- MODIFICADO: Garante que a fila está vazia antes de tentar salvar
            self.processing_queue.put(None)

            self.data_thread.join(timeout=2.0)
            self.processing_thread.join(timeout=10.0)
            
            print("Iniciando limpeza e salvamento final.")
            self._move_base(1)
            time.sleep(3)
            self._move_head(0.5)
            self._save_point_cloud() # <--- MODIFICADO: Agora este é o salvamento final e completo
            self.subscriber.close()
            self.context.term()

    def _data_ingestion_loop(self):
        while self.running:
            try:
                topic, payload = self.subscriber.recv_multipart()
                msg = msgpack.unpackb(payload)
                
                if topic == b'sensor/imu':
                    self.imu_buffer.append(msg)
                elif topic == b'sensor/lidar/chunk':
                    self.lidar_buffer.extend(msg.get('points', []))
                elif topic.startswith(b'flags/'):
                    status = msg.get('status')
                    if status == 'disabled':
                        self.pause = True
                        print("WARN: Comunicação com sensores perdida. Sistema pausado.")
                    elif status == 'active':
                        self.pause = False
            except zmq.ZMQError:
                if not self.running:
                    break 
                else:
                    print("ERRO INESPERADO: Erro na thread de dados ZMQ.")
                    self.state = 'ERROR'
                    break
                    
    def _set_servo_pulse(self, channel, pulse_width):
        duty_cycle = int((pulse_width / 20000) * 0xFFFF)
        self.pca.channels[channel].duty_cycle = duty_cycle
        
    def _move_base(self, value):
        pulse = 500 + value * 2000
        self._set_servo_pulse(1, pulse)
        
    def _move_head(self, value):
        pulse = 1200 + value * 1000
        self._set_servo_pulse(0, pulse)
        
    def _get_blocking_imu_message(self, timeout_s=1.0):
        start_time = time.time()
        initial_len = len(self.imu_buffer)
        while time.time() - start_time < timeout_s:
            if len(self.imu_buffer) > initial_len:
                return self.imu_buffer[-1]
            time.sleep(0.001)
        print("WARN: Timeout ao esperar por mensagem da IMU.")
        return self.imu_buffer[-1] if self.imu_buffer else None
        
    def _run_state_machine(self):
        if self.pause:
            time.sleep(1)
            return
        if self.state == 'IDLE':
            self._handle_idle_state()
        elif self.state == 'MOVING':
            self._handle_moving_state()
            
    def _handle_idle_state(self):
        # <--- LÓGICA DE SALVAMENTO PROGRESSIVO ADICIONADA AQUI ---
        
        # Primeiro, processa os dados coletados enquanto estava parado
        self._collect_idle_lidar_data()

        # Se o plano de movimento acabou, finaliza o processo
        if not self.movement_plan:
            self.state = 'FINISHED'
            return

        next_move = self.movement_plan[0]
        base_target, _ = next_move

        if base_target is not None and not self.is_first_run:
            self.processing_queue.join()
            self._save_point_cloud()

        # Agora, continua com a lógica original
        if self.is_first_run:
            self.is_first_run = False
        
        self.current_move = self.movement_plan.popleft()
        self.moving_angles = self._get_blocking_imu_message()
        if not self.moving_angles:
            self.pause = True
            self.movement_plan.appendleft(self.current_move)
            return
        
        base_target, head_target = self.current_move
        if base_target is not None: self._move_base(base_target)
        if head_target is not None: self._move_head(head_target)
        
        self.move_start_time_ns = time.time_ns()
        self.state = 'MOVING'
        self.imu_buffer.clear()
        self.lidar_buffer.clear()

    def _collect_idle_lidar_data(self):
        time.sleep(1)
        imu_snapshot = list(self.imu_buffer)
        lidar_snapshot = list(self.lidar_buffer)
        self.imu_buffer.clear() 
        self.lidar_buffer.clear() 
        self._process_stopped_data(imu_snapshot, lidar_snapshot)

    def _calibrate_drift_rate(self, duration_s=5):
        print("Calibrando drift da IMU...")
        self._move_base(0.5)
        time.sleep(2)
        self.imu_buffer.clear()
        time.sleep(duration_s)
        imu_readings = list(self.imu_buffer)
        yaws = np.unwrap([m['data']['yaw'] for m in imu_readings], period=360)
        timestamps_s = np.array([m['timestamp_ns'] / 1e9 for m in imu_readings])
        A = np.vstack([timestamps_s, np.ones(len(timestamps_s))]).T
        slope, _ = np.linalg.lstsq(A, yaws, rcond=None)[0]
        time.sleep(2)
        self._move_base(1)
        return slope
        
    def _determine_move_stop(self, settling_time_s=2.0):
        if (time.time_ns() - self.move_start_time_ns) / 1e9 < settling_time_s:
            return None, None
        stop_time_ns = self.move_start_time_ns + int(settling_time_s * 1e9)
        imu_snapshot = list(self.imu_buffer)
        if not imu_snapshot: return None, None
        stop_msg = min(imu_snapshot, key=lambda m: abs(m['timestamp_ns'] - stop_time_ns))
        return stop_msg, stop_time_ns
        
    def _save_point_cloud(self):
        # <--- NENHUMA MUDANÇA NECESSÁRIA AQUI ---
        # A função já salva a nuvem de pontos completa e sobrescreve o arquivo (modo 'w')
        with self.point_cloud_lock:
            if not self.final_point_cloud:
                print("Nenhum ponto foi coletado ainda. Nenhum arquivo foi salvo.")
                return
            
            points_to_save = self.final_point_cloud.copy()

        print(f"Salvando {len(points_to_save)} pontos em '{self.output_filename}'...")
        fieldnames = [
            'distancia_lidar', 'angulo_lidar', 'intensidade_lidar',
            'roll', 'pitch', 'yaw', 'estado', 'timestamp', 'timestamp_imu',
            'movimento_base', 'movimento_cabeca'
        ]
        try:
            with open(self.output_filename, 'w', newline='', encoding='utf-8') as f:
                writer = csv.DictWriter(f, fieldnames=fieldnames)
                writer.writeheader()
                writer.writerows(points_to_save)
            print("Arquivo salvo com sucesso.")
        except Exception as e:
            print(f"ERRO: Falha ao salvar o arquivo CSV: {e}")
            
def gerar_plano_de_scan_dinamico(num_pontos_desejado, tipo, passo_min=0.15, passo_max=0.40):
    if num_pontos_desejado <= 0:
        return []

    posicao_atual = 1.0
    direcao = -1
    sequencia_pontos = [posicao_atual]

    while len(sequencia_pontos) < num_pontos_desejado:
        
        passo = np.random.uniform(passo_min, passo_max)
        
        proxima_posicao = posicao_atual + direcao * passo

        if proxima_posicao >= 1.0:
            posicao_atual = 1.0
            direcao = -1
        elif proxima_posicao <= 0.0:
            posicao_atual = 0.0
            direcao = 1
        else:
            posicao_atual = proxima_posicao
            
        sequencia_pontos.append(round(posicao_atual, 2))

    if tipo=='tilt':
      plano_final = [[None, p] for p in sequencia_pontos]
    else:
      plano_final = [[p, None] for p in sequencia_pontos]

    return plano_final


def gerar_plano_com_marcadores(num_pontos_pan, num_pontos_tilt):
    sequencia_tilt = gerar_plano_de_scan_dinamico(num_pontos_tilt, 'tilt')
    sequencia_pan = gerar_plano_de_scan_dinamico(num_pontos_pan, 'pan')

    plano_final = []

    for pan in sequencia_pan:
      plano_final.append(pan)
      plano_final.extend(sequencia_tilt)

    return plano_final

if __name__ == '__main__':
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    nome_arquivo_saida = f"scan_{timestamp}.csv"
    print(f"Salvando dados no arquivo: {nome_arquivo_saida}")
    numero_de_pontos_tilt = 300
    numero_de_pontos_pan = 10
    scan_plan = gerar_plano_com_marcadores(num_pontos_pan=numero_de_pontos_pan, num_pontos_tilt=numero_de_pontos_tilt)
    controller = ScanController(movement_plan=scan_plan, output_filename=nome_arquivo_saida)
    controller.run()
