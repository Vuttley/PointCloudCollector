from adafruit_pca9685 import PCA9685
from board import SCL, SDA
import busio
import time
import subprocess
import re
import numpy as np
import open3d as o3d
from pathlib import Path
from math import cos, sin, radians
import serial
import struct
from scipy.spatial.transform import Rotation as R
import threading
import socket
import pandas as pd
import os

i2c = busio.I2C(SCL, SDA)
pca = PCA9685(i2c)
pca.frequency = 50
stop_const = True

def set_servo_pulse(channel, pulse_width):
    duty_cycle = int((pulse_width / 20000) * 0xFFFF)
    pca.channels[channel].duty_cycle = duty_cycle

# 500 a 2500 para servo 1

#1350 para servo 0 reto
#set_servo_pulse(0, 1350)
#1000 a 1700, servo 0  não pode dar 360

servo_1 = 0
servo_2 = 1

dist_sensor_eixo = 0.155

def move_head(value):
	pulse = 1050 + value*600
	set_servo_pulse(servo_1, pulse)
	expected_angle = 0.18*(pulse-350) - 180
	return round(expected_angle, 2)
    
def move_base(value):
	pulse = 500 + value*2000
	set_servo_pulse(servo_2, pulse)
	expected_angle = 0.18*(pulse-500)
	return round(expected_angle, 2)
    
class HeadMovement(threading.Thread):
    def __init__(self):
        super().__init__()
        self.running = True
        self.head_expected = None
        self.base_expected = None
        self.lock = threading.Lock()

    def run(self):
        while self.running:
            for k in range(19):
                value = (k + 1) / 20.0
                self.base_expected = move_base(value)
                time.sleep(2)
                for i in range(20):
                    value = i / 20.0
                    self.head_expected = move_head(value)
                    time.sleep(1)
                for i in range(20):
                    value = 1 - (i / 20.0)
                    self.head_expected = move_head(value)
                    time.sleep(1)
            self.stop()
            
    def get_expected_angles(self):
        with self.lock:
            return self.head_expected, self.base_expected
            
    def get_running(self):
        return self.running

    def stop(self):
        self.running = False

HOST = '0.0.0.0'
PORT = 5000

server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.bind((HOST, PORT))
server.listen(1)
server.settimeout(10)
print("Aguardando conexão...")

conn = None
try:
    conn, addr = server.accept()
    print(f"Conectado a {addr}")
except socket.timeout:
    print("Nenhuma conexão estabelecida, executando sem TCP/IP.")

head_movement_thread = HeadMovement()
head_movement_thread.start()

class IMUReader(threading.Thread):
    def __init__(self, port, baudrate):
        super().__init__()
        self.port = port
        self.baudrate = baudrate
        self.ser = serial.Serial(port, baudrate, timeout=1)
        self.roll = None
        self.pitch = None
        self.yaw = None
        self.running = True
        self.lock = threading.Lock()
        self.last_packet_time = time.time()

    def run(self):
        threshold = 1
        while self.running:
            try:
                packet = self.read_packet()
            except Exception as e:
                print("Erro ao ler pacote:", e)
                self.reconnect()
                continue

            if packet is None:
                self.roll = None
                self.pitch = None
                self.yaw = None
                if time.time() - self.last_packet_time > threshold:
                    print("Nenhum pacote recebido por mais de {} segundos, tentando reconexão.".format(threshold))
                    self.reconnect()
                continue

            if packet[0] != 0x55 or packet[1] != 0x53:
                continue

            if not self.verify_checksum(packet):
                print("Erro de checksum:", packet.hex())
                continue

            self.last_packet_time = time.time()

            roll = struct.unpack("<h", packet[2:4])[0] / 32768.0 * 180
            pitch = struct.unpack("<h", packet[4:6])[0] / 32768.0 * 180
            yaw = struct.unpack("<h", packet[6:8])[0] / 32768.0 * 180

            with self.lock:
                self.roll, self.pitch, self.yaw = roll, pitch, yaw

    def read_packet(self):
        data = self.ser.read(11)
        return data if len(data) == 11 else None

    def verify_checksum(self, packet):
        return (sum(packet[:10]) & 0xFF) == packet[10]

    def get_angles(self):
        with self.lock:
            return self.roll, self.pitch, self.yaw

    def reconnect(self):
        print("Tentando reconectar ao IMU...")
        while self.running:
            try:
                self.ser.close()
            except Exception:
                pass
            time.sleep(2)
            try:
                self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
                print("Reconexão bem-sucedida.")
                self.last_packet_time = time.time()
                return
            except Exception as e:
                print("Falha na reconexão:", e)
                time.sleep(2)

    def stop(self):
        """Encerra a thread"""
        self.running = False
        self.ser.close()

def send_data():
    global df, indx, conn
    command = [
        "/home/admin/Documents/stl27lsdk/sdk_ldrobotsensorteam_stl_v3.0.5_stable_20230203-18-10/linux_app/build/ldlidar_stl",
        "STL27L", "serialcom", "/dev/ttyUSB0"
    ]
    process = subprocess.Popen(command, stdout=subprocess.PIPE, text=True)
    angles = []
    turning_point = 0
    try:
        status = head_movement_thread.get_running()
        while status:
            line = process.stdout.readline()
            if not line and process.poll() is not None:
                break
            Row, Pitch, Yaw = imu_reader.get_angles()
            if Row is None or Pitch is None or Yaw is None:
                imu_reader.reconnect()
                time.sleep(0.1)
                break
            expected_head, expected_base = head_movement_thread.get_expected_angles()
            match = re.search(r"angle:(\d+\.\d+),distance\(mm\):(\d+),intensity:(\d+)", line)
            if match:
                angle = radians(float(match.group(1)))
                distance = float(match.group(2)) / 1000.0
                intensity = float(match.group(3)) / 255.0
                data = 'distance: ' + str(distance) + ' intensity: ' +  str(intensity) + ' angle: ' +  str(angle) + ' Row: ' +  str(Row) + ' Pitch: ' +  str(Pitch) + ' Yaw: ' +  str(Yaw) + ' Expected_Head: ' + str(expected_head) + ' Expected_Base: ' + str(expected_base)+ '\n' 
                new_data = {
                'index': indx, 
                'distance': distance, 
                'intensity': intensity, 
                'angle': angle, 
                'roll': Row, 
                'pitch': Pitch, 
                'yaw': Yaw, 
                'expected_head': expected_head, 
                'expected_base': expected_base
                }
                df = pd.concat([df, pd.DataFrame([new_data])], ignore_index = True)
                indx+=1
            status = head_movement_thread.get_running()
    except KeyboardInterrupt:
        print("Encerrando visualização...")
        head_movement_thread.stop()
        head_movement_thread.join()
    finally:
        if conn is not None:
            try:
                conn.sendall(data.encode('utf-8'))
            except:
                pass
        
def get_unique_filename(base_name="Experimento", extension=".csv", directory="."):
    x = 1
    while True:
        filename = f"{base_name}{x}{extension}"
        filepath = os.path.join(directory, filename)
        if not os.path.exists(filepath):
            return filepath
        x += 1

if __name__ == "__main__":
    PORT = "/dev/ttyAMA0"
    BAUDRATE = 9600
    imu_reader = IMUReader(PORT, BAUDRATE)
    imu_reader.start()
    columns = ['index', 'distance', 'intensity', 'angle', 'roll', 'pitch', 'yaw', 'expected_head', 'expected_base']
    df = pd.DataFrame(columns = columns)
    indx = 0
    try:
        status = head_movement_thread.get_running()
        while status:
            send_data()
            status = head_movement_thread.get_running()
        unique_filepath = get_unique_filename()
        df.to_csv(unique_filepath, index=False)
    except KeyboardInterrupt:
        print("Encerrando visualização...")
        head_movement_thread.stop()
        head_movement_thread.join()
    finally:
        if process.poll() is None:
            process.terminate()
        
