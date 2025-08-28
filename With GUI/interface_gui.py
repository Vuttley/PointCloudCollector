import customtkinter as ctk
from tkinter import filedialog, messagebox
import os
import re
from pathlib import Path
import threading
import queue
import sys
import multiprocessing
import shutil
from PIL import Image
import paramiko
import numpy as np
import zmq

try:
    from thread import process_scan_pair
except ImportError:
    messagebox.showerror("Erro Crítico", "O arquivo 'thread.py' não foi encontrado.")
    sys.exit(1)

class PiController:
    def __init__(self, hostname, username, password):
        self.hostname = hostname
        self.username = username
        self.password = password
        self.ssh_client = None

    def connect(self):
        if self.ssh_client and self.ssh_client.get_transport().is_active():
            return True, "Já conectado."
        try:
            self.ssh_client = paramiko.SSHClient()
            self.ssh_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            self.ssh_client.connect(self.hostname, username=self.username, password=self.password, timeout=5)
            return True, f"Conectado a {self.hostname}"
        except Exception as e:
            self.ssh_client = None
            return False, f"Falha ao conectar: {e}"

    def disconnect(self):
        if self.ssh_client:
            self.ssh_client.close()
            self.ssh_client = None
            return True, "Desconectado."
        return False, "Já estava desconectado."

    def start_scanner(self, script_path):
        if not self.ssh_client: return False, "Não conectado ao Pi."
        command = f"bash {script_path}"
        try:
            stdin, stdout, stderr = self.ssh_client.exec_command(command, timeout=10)
            exit_status = stdout.channel.recv_exit_status()
            if exit_status == 0:
                return True, "Comando de início enviado."
            else:
                error = stderr.read().decode('utf-8').strip()
                return False, f"Falha no script: {error}"
        except Exception as e:
            return False, f"Exceção: {e}"

    def stop_scanner(self, script_path):
        if not self.ssh_client: return False, "Não conectado ao Pi."
        command = f"bash {script_path}"
        try:
            stdin, stdout, stderr = self.ssh_client.exec_command(command, timeout=10)
            exit_status = stdout.channel.recv_exit_status()
            if exit_status == 0:
                return True, "Sessão de scanner parada."
            else:
                error = stderr.read().decode('utf-8').strip()
                return False, f"Falha ao parar: {error}"
        except Exception as e:
            return False, f"Exceção: {e}"
            
    def shutdown_pi(self):
        if not self.ssh_client: return False, "Não conectado ao Pi."
        command = "sudo shutdown -h now"
        try:
            self.ssh_client.exec_command(command, timeout=5)
            return True, "Comando de desligamento enviado."
        except Exception as e:
            if isinstance(e, paramiko.ssh_exception.SSHException):
                return True, "Comando de desligamento enviado."
            return False, f"Exceção: {e}"

def show_ply_in_new_process(filepath):
    import open3d as o3d
    from pathlib import Path
    if not filepath or not Path(filepath).exists():
        print(f"ERRO (Processo Filho): Arquivo não encontrado - {filepath}"); return
    try:
        pcd = o3d.io.read_point_cloud(str(filepath))
        if not pcd.has_points():
            print(f"ERRO (Processo Filho): O arquivo .ply está vazio - {filepath}"); return
        o3d.visualization.draw_geometries([pcd], window_name=f"Visualizador - {Path(filepath).name}")
    except Exception as e:
        print(f"ERRO (Processo Filho): Ocorreu um erro ao visualizar - {e}")

class QueueIO(queue.Queue):
    def write(self, text): self.put(text)
    def flush(self): sys.__stdout__.flush()

class MainApplication(ctk.CTk):
    def __init__(self):
        super().__init__()
        self.title("Cerise 3D Processor")
        self.after(20, lambda: self.state('zoomed'))
        self.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.grid_rowconfigure(2, weight=1)
        self.grid_columnconfigure(0, weight=1)
        ctk.set_appearance_mode("dark")
        ctk.set_default_color_theme("blue")
        
        header_container = ctk.CTkFrame(self, fg_color="transparent")
        header_container.grid(row=0, column=0, sticky="ew", padx=10, pady=5)
        header_container.grid_columnconfigure(0, weight=1)
        logo_frame = ctk.CTkFrame(header_container, fg_color="transparent")
        logo_frame.grid(row=0, column=0, sticky="ew")
        logo_frame.grid_columnconfigure(0, weight=1)
        try:
            logo_image_data = Image.open("logo_cerise.png")
            logo_image = ctk.CTkImage(light_image=logo_image_data, size=(200, 50))
            ctk.CTkLabel(logo_frame, image=logo_image, text="").grid(row=0, column=0, pady=10)
        except FileNotFoundError:
            ctk.CTkLabel(logo_frame, text="Cerise 3D", font=ctk.CTkFont(size=20, weight="bold")).grid(row=0, column=0, pady=10)
        
        self.nav_frame = ctk.CTkFrame(header_container, fg_color="transparent")
        self.nav_frame.grid(row=1, column=0, pady=(5,10))
        self.btn_scanner_control = ctk.CTkButton(self.nav_frame, text="Controle do Scanner", command=lambda: self.show_frame(ScannerControlFrame))
        self.btn_scanner_control.pack(side="left", padx=5)
        self.btn_data_source = ctk.CTkButton(self.nav_frame, text="Fonte de Dados", command=lambda: self.show_frame(DataSourceFrame))
        self.btn_data_source.pack(side="left", padx=5)
        
        container = ctk.CTkFrame(self)
        container.grid(row=2, column=0, sticky="nsew", padx=10, pady=(0, 10))
        container.grid_rowconfigure(0, weight=1)
        container.grid_columnconfigure(0, weight=1)

        self.frames = {}
        for F in (ScannerControlFrame, DataSourceFrame, ProcessingFrame):
            frame = F(container, self)
            self.frames[F] = frame
            frame.grid(row=0, column=0, sticky="nsew")

        self.show_frame(ScannerControlFrame)

    def show_frame(self, cont_class):
        if cont_class == ProcessingFrame: self.nav_frame.grid_remove()
        else: self.nav_frame.grid(row=1, column=0, pady=(5,10))
        
        self.btn_scanner_control.configure(state="disabled" if cont_class == ScannerControlFrame else "normal")
        self.btn_data_source.configure(state="disabled" if cont_class == DataSourceFrame else "normal")
        
        frame = self.frames[cont_class]
        frame.tkraise()

    def show_processing_frame(self, folder_path):
        self.show_frame(ProcessingFrame)
        self.frames[ProcessingFrame].load_folder_data(folder_path)

    def show_data_source_frame(self):
        self.frames[DataSourceFrame].reset_state()
        self.show_frame(DataSourceFrame)

    def lock_navigation(self):
        self.btn_data_source.configure(state="disabled")

    def unlock_navigation(self):
        self.btn_data_source.configure(state="normal")

    def on_closing(self):
        scanner_frame = self.frames[ScannerControlFrame]
        if scanner_frame.pi_controller: scanner_frame.pi_controller.disconnect()
        if scanner_frame.status_listener_thread and scanner_frame.status_listener_thread.is_alive():
            scanner_frame.stop_status_listener()
        sys.stdout = sys.__stdout__
        self.destroy()


class ScannerControlFrame(ctk.CTkFrame):
    def __init__(self, parent, controller):
        super().__init__(parent)
        self.controller = controller
        
        self.pi_controller = None
        self.grid_columnconfigure(0, weight=1)
        self.grid_rowconfigure(0, weight=1)

        self.status_listener_thread = None
        self.is_listening_status = False
        self.zmq_context = zmq.Context()

        main_frame = ctk.CTkFrame(self, fg_color="transparent"); main_frame.grid(row=0, column=0)
        ctk.CTkLabel(main_frame, text="Controle Remoto do Scanner", font=ctk.CTkFont(size=24, weight="bold")).grid(row=0, column=0, pady=(10, 20))
        
        control_frame = ctk.CTkFrame(main_frame, fg_color="transparent"); control_frame.grid(row=1, column=0, padx=20, pady=10, sticky="ew")
        control_frame.grid_columnconfigure(1, weight=1)
        
        connect_frame = ctk.CTkFrame(control_frame, fg_color="transparent"); connect_frame.grid(row=0, column=0, rowspan=2, padx=(10,5), pady=5, sticky="ns")
        self.pi_connect_button = ctk.CTkButton(connect_frame, text="Conectar ao Pi", command=self.connect_to_pi_thread); self.pi_connect_button.pack(pady=5)
        self.pi_disconnect_button = ctk.CTkButton(connect_frame, text="Desconectar", command=self.disconnect_from_pi_thread); self.pi_disconnect_button.pack(pady=5)
        
        self.pi_status_label = ctk.CTkLabel(control_frame, text="Status: Desconectado", text_color="gray", anchor="w"); self.pi_status_label.grid(row=0, column=1, padx=10, pady=5, sticky="ewns")
        
        self.scan_progress_label = ctk.CTkLabel(control_frame, text="", text_color="cyan", anchor="w"); self.scan_progress_label.grid(row=1, column=1, padx=10, pady=5, sticky="ewns")

        scanner_buttons_frame = ctk.CTkFrame(main_frame, fg_color="transparent"); scanner_buttons_frame.grid(row=2, column=0, pady=(20, 10))
        self.start_scanner_button = ctk.CTkButton(scanner_buttons_frame, text="Iniciar Scanner", command=self.start_scanner_thread); self.start_scanner_button.pack(side="left", padx=5)
        self.stop_scanner_button = ctk.CTkButton(scanner_buttons_frame, text="Parar Scanner", command=self.stop_scanner_thread); self.stop_scanner_button.pack(side="left", padx=5)

        shutdown_frame = ctk.CTkFrame(main_frame, fg_color="transparent"); shutdown_frame.grid(row=3, column=0, pady=(30,10))
        self.shutdown_pi_button = ctk.CTkButton(shutdown_frame, text="Desligar o Raspberry Pi", command=self.shutdown_pi_thread, fg_color="#D9534F", hover_color="#C9302C")
        self.shutdown_pi_button.pack()
        
        self._update_button_states('DISCONNECTED')

    def _update_button_states(self, state):
        if state == 'DISCONNECTED':
            self.pi_connect_button.configure(state="normal", text="Conectar ao Pi")
            self.pi_disconnect_button.configure(state="disabled")
            self.start_scanner_button.configure(state="disabled")
            self.stop_scanner_button.configure(state="disabled")
            self.shutdown_pi_button.configure(state="disabled")
            self.controller.unlock_navigation()
        elif state == 'BUSY':
            self.pi_connect_button.configure(state="disabled")
            self.pi_disconnect_button.configure(state="disabled")
            self.start_scanner_button.configure(state="disabled")
            self.stop_scanner_button.configure(state="disabled")
            self.shutdown_pi_button.configure(state="disabled")
            self.controller.lock_navigation()
        elif state == 'CONNECTED_IDLE':
            self.pi_connect_button.configure(state="disabled", text="Conectado")
            self.pi_disconnect_button.configure(state="normal")
            self.start_scanner_button.configure(state="normal")
            self.stop_scanner_button.configure(state="disabled")
            self.shutdown_pi_button.configure(state="normal")
            self.controller.unlock_navigation()
        elif state == 'SCANNING':
            self.pi_connect_button.configure(state="disabled", text="Conectado")
            self.pi_disconnect_button.configure(state="disabled")
            self.start_scanner_button.configure(state="disabled")
            self.stop_scanner_button.configure(state="normal")
            self.shutdown_pi_button.configure(state="disabled")
            self.controller.lock_navigation()
    
    def connect_to_pi_thread(self):
        self._update_button_states('BUSY')
        self.pi_connect_button.configure(text="Conectando...")
        threading.Thread(target=self.connect_to_pi, daemon=True).start()

    def connect_to_pi(self):
        success, message = False, "Erro desconhecido"
        try:
            self.pi_controller = PiController(PI_HOSTNAME, PI_USERNAME, PI_PASSWORD)
            success, message = self.pi_controller.connect()
        except Exception as e:
            message, success = f"Exceção: {e}", False
        finally:
            self.after(0, self.update_pi_connection_status, success, message)

    def update_pi_connection_status(self, success, message):
        if success:
            self.pi_status_label.configure(text=message, text_color="green")
            self._update_button_states('CONNECTED_IDLE')
        else:
            self.pi_status_label.configure(text=message, text_color="red")
            self._update_button_states('DISCONNECTED')
            self.pi_controller = None

    def disconnect_from_pi_thread(self):
        self._update_button_states('BUSY')
        self.pi_disconnect_button.configure(text="Desconectar")
        threading.Thread(target=self.disconnect_from_pi, daemon=True).start()

    def disconnect_from_pi(self):
        try:
            if self.pi_controller: self.pi_controller.disconnect()
        except Exception as e: print(f"Exceção em disconnect_from_pi: {e}")
        finally: self.after(0, self.update_pi_disconnection_status)

    def update_pi_disconnection_status(self):
        self.pi_status_label.configure(text="Status: Desconectado", text_color="gray")
        self._update_button_states('DISCONNECTED')
        self.pi_controller = None

    def start_scanner_thread(self):
        self._update_button_states('BUSY')
        threading.Thread(target=self.start_scanner, daemon=True).start()

    def start_scanner(self):
        success, message = False, "Controlador do Pi não inicializado."
        try:
            if self.pi_controller:
                success, message = self.pi_controller.start_scanner(START_SCANNER_SCRIPT_PATH)
                if success:
                    self.start_status_listener()
        except Exception as e: message, success = f"Exceção: {e}", False
        finally: self.after(0, self.update_pi_scanner_status, success, message, False)

    def stop_scanner_thread(self):
        self._update_button_states('BUSY')
        threading.Thread(target=self.stop_scanner, daemon=True).start()
        
    def stop_scanner(self):
        success, message = False, "Controlador do Pi não inicializado."
        try:
            if self.pi_controller: 
                success, message = self.pi_controller.stop_scanner(STOP_SCANNER_SCRIPT_PATH)
                self.stop_status_listener()
        except Exception as e: message, success = f"Exceção: {e}", False
        finally: self.after(0, self.update_pi_scanner_status, success, message, True)

    def update_pi_scanner_status(self, success, message, is_stopping):
        if success:
            self.pi_status_label.configure(text=message, text_color="orange" if is_stopping else "green")
            if is_stopping: self._update_button_states('CONNECTED_IDLE')
            else: self._update_button_states('SCANNING')
        else:
            self.pi_status_label.configure(text=message, text_color="red")
            self._update_button_states('CONNECTED_IDLE')

    def start_status_listener(self):
        if self.status_listener_thread and self.status_listener_thread.is_alive():
            print("Listener de status já está ativo.")
            return
        
        print("Iniciando listener de status do scanner...")
        self.is_listening_status = True
        self.status_listener_thread = threading.Thread(target=self._status_listener_loop, daemon=True)
        self.status_listener_thread.start()

    def stop_status_listener(self):
        print("Parando listener de status do scanner...")
        self.is_listening_status = False

        if self.status_listener_thread:
            self.status_listener_thread.join(timeout=1.0)
        self.after(0, self.update_scan_progress_label, "")

    def _status_listener_loop(self):
        socket = self.zmq_context.socket(zmq.SUB)
        socket.setsockopt(zmq.SUBSCRIBE, b"")
        socket.setsockopt(zmq.RCVTIMEO, 2000)
        
        status_address = f"tcp://{PI_HOSTNAME}:5561"
        socket.connect(status_address)
        
        print(f"Listener de status conectado a {status_address}")

        while self.is_listening_status:
            try:
                message = socket.recv_string()
                self.after(0, self.update_scan_progress_label, message)
            except zmq.Again:
                continue
            except zmq.ZMQError as e:
                if e.errno == zmq.ETERM:
                    print("Contexto ZMQ encerrado, listener de status terminando.")
                    break
                else:
                    print(f"Erro ZMQ no listener de status: {e}")
                    break
        
        socket.close()
        print("Listener de status encerrado.")
    
    def update_scan_progress_label(self, message):
        self.scan_progress_label.configure(text=message)

    def shutdown_pi_thread(self):
        if messagebox.askyesno("Confirmar Desligamento", 
                               "Você tem certeza que deseja desligar o Raspberry Pi?\n\nA conexão será perdida permanentemente até que o Pi seja ligado novamente de forma manual.",
                               icon='warning'):
            self._update_button_states('BUSY')
            self.pi_status_label.configure(text="Enviando comando de desligamento...", text_color="orange")
            threading.Thread(target=self.shutdown_pi, daemon=True).start()

    def shutdown_pi(self):
        success, message = False, "Controlador do Pi não inicializado."
        try:
            if self.pi_controller:
                success, message = self.pi_controller.shutdown_pi()
        except Exception as e:
            message, success = f"Exceção: {e}", False
        finally:
            self.after(500, self.update_pi_disconnection_status) 

class DataSourceFrame(ctk.CTkFrame):
    def __init__(self, parent, controller):
        super().__init__(parent)
        self.controller = controller
        
        self.selected_path = None
        self.copy_thread = None

        self.grid_columnconfigure(0, weight=1)
        self.grid_rowconfigure(0, weight=1)
        self.grid_rowconfigure(3, weight=1)

        main_frame = ctk.CTkFrame(self, fg_color="transparent")
        main_frame.grid(row=1, column=0, sticky="n")
        main_frame.grid_columnconfigure(0, weight=1)

        ctk.CTkLabel(main_frame, text="Selecione a Fonte dos Dados", font=ctk.CTkFont(size=24, weight="bold")).grid(row=0, column=0, pady=(20, 10))
        local_frame = ctk.CTkFrame(main_frame, fg_color="transparent")
        local_frame.grid(row=1, column=0, padx=20, pady=10, sticky="ew")
        local_frame.grid_columnconfigure(0, weight=1)
        ctk.CTkLabel(local_frame, text="Opção 1: Processar Pasta Local", font=ctk.CTkFont(family="Roboto", size=18, weight='bold')).grid(row=0, column=0, columnspan=2, padx=10, pady=(10, 5), sticky="w")
        self.local_path_entry = ctk.CTkEntry(local_frame, placeholder_text="Selecione uma pasta no seu PC...", width=400)
        self.local_path_entry.grid(row=1, column=0, padx=10, pady=5, sticky="ew")
        ctk.CTkButton(local_frame, text="Procurar...", command=self.select_local_folder).grid(row=1, column=1, padx=10, pady=5)
        pi_frame = ctk.CTkFrame(main_frame, fg_color="transparent")
        pi_frame.grid(row=2, column=0, padx=20, pady=10, sticky="ew")
        pi_frame.grid_columnconfigure(1, weight=1)
        ctk.CTkLabel(pi_frame, text="Opção 2: Copiar Pasta do Raspberry Pi", font=ctk.CTkFont(family="Roboto", size=18, weight='bold')).grid(row=0, column=0, columnspan=3, padx=10, pady=(10, 5), sticky="w")
        ctk.CTkLabel(pi_frame, text="Origem (Pi):").grid(row=1, column=0, padx=10, pady=5, sticky="w")
        self.pi_source_entry = ctk.CTkEntry(pi_frame, placeholder_text="Selecione a pasta no drive de rede (Z:\\)...")
        self.pi_source_entry.grid(row=1, column=1, padx=5, pady=5, sticky="ew")
        ctk.CTkButton(pi_frame, text="Procurar...", command=self.select_pi_source_folder).grid(row=1, column=2, padx=10, pady=5)
        ctk.CTkLabel(pi_frame, text="Destino (PC):").grid(row=2, column=0, padx=10, pady=5, sticky="w")
        self.pi_dest_entry = ctk.CTkEntry(pi_frame, placeholder_text="Selecione/crie uma pasta no seu PC...")
        self.pi_dest_entry.grid(row=2, column=1, padx=5, pady=5, sticky="ew")
        ctk.CTkButton(pi_frame, text="Procurar...", command=self.select_local_destination_folder).grid(row=2, column=2, padx=10, pady=5)
        self.proceed_button = ctk.CTkButton(main_frame, text="Avançar para Processamento", command=self.proceed, state="disabled")
        self.proceed_button.grid(row=4, column=0, pady=20)

    def reset_state(self):
        self.local_path_entry.delete(0, "end")
        self.pi_source_entry.delete(0, "end")
        self.pi_dest_entry.delete(0, "end")
        self.proceed_button.configure(state="disabled")
        self.selected_path = None

    def select_local_folder(self):
        path = filedialog.askdirectory(title="Selecione uma pasta com dados de scan")
        if path:
            self.local_path_entry.delete(0, "end")
            self.local_path_entry.insert(0, path)
            self.clear_pi_selection()
            self.proceed_button.configure(state="normal")
            self.selected_path = path

    def select_pi_source_folder(self):
        path = filedialog.askdirectory(title="Selecione a pasta de ORIGEM no Raspberry Pi")
        if path:
            self.pi_source_entry.delete(0, "end")
            self.pi_source_entry.insert(0, path)
            self.clear_local_selection()
            self.check_pi_paths()

    def select_local_destination_folder(self):
        path = filedialog.askdirectory(title="Selecione a pasta de DESTINO no seu PC")
        if path:
            self.pi_dest_entry.delete(0, "end")
            self.pi_dest_entry.insert(0, path)
            self.clear_local_selection()
            self.check_pi_paths()

    def check_pi_paths(self):
        if self.pi_source_entry.get() and self.pi_dest_entry.get():
            self.proceed_button.configure(state="normal")
        else:
            self.proceed_button.configure(state="disabled")

    def clear_local_selection(self):
        self.local_path_entry.delete(0, "end")
        self.selected_path = None

    def clear_pi_selection(self):
        self.pi_source_entry.delete(0, "end")
        self.pi_dest_entry.delete(0, "end")
    
    def proceed(self):
        self.proceed_button.configure(state="disabled")
        if self.local_path_entry.get():
            self.selected_path = self.local_path_entry.get()
            self.controller.show_processing_frame(self.selected_path)
        elif self.pi_source_entry.get() and self.pi_dest_entry.get():
            source = self.pi_source_entry.get()
            dest_base = self.pi_dest_entry.get()
            source_folder_name = os.path.basename(source)
            self.selected_path = os.path.join(dest_base, source_folder_name)
            self.show_copy_progress_popup()
            self.copy_thread = threading.Thread(target=self.copy_folder_thread, args=(source, self.selected_path), daemon=True)
            self.copy_thread.start()
            self.after(100, self.check_copy_thread)

    def show_copy_progress_popup(self):
        self.progress_popup = ctk.CTkToplevel(self)
        self.progress_popup.title("Copiando...")
        self.progress_popup.geometry("300x100")
        self.progress_popup.transient(self)
        self.progress_popup.grab_set()
        ctk.CTkLabel(self.progress_popup, text="Copiando arquivos do Raspberry Pi...\nAguarde, isso pode levar um tempo.").pack(pady=10, padx=10, expand=True)
        progress = ctk.CTkProgressBar(self.progress_popup, mode="indeterminate")
        progress.pack(pady=10, padx=10, fill="x", expand=True)
        progress.start()
        
    def copy_folder_thread(self, source, destination):
        try:
            print(f"Iniciando cópia de '{source}' para '{destination}'...")
            shutil.copytree(source, destination, dirs_exist_ok=True)
            print("Cópia concluída com sucesso!")
        except Exception as e:
            print(f"ERRO durante a cópia: {e}")
            self.after(0, lambda: messagebox.showerror("Erro de Cópia", f"Falha ao copiar arquivos: {e}"))
            self.selected_path = None

    def check_copy_thread(self):
        if self.copy_thread.is_alive():
            self.after(100, self.check_copy_thread)
        else:
            self.progress_popup.destroy()
            if self.selected_path:
                self.controller.show_processing_frame(self.selected_path)
            else:
                self.proceed_button.configure(state="normal")


class ProcessingFrame(ctk.CTkFrame):
    def __init__(self, parent, controller):
        super().__init__(parent)
        self.controller = controller
        
        self.grid_columnconfigure(0, weight=1)
        self.grid_rowconfigure(1, weight=1)
        
        self.ply_visualize_buttons = []
        
        path_frame = ctk.CTkFrame(self, fg_color="transparent")
        path_frame.grid(row=0, column=0, padx=10, pady=(10,5), sticky="ew")
        path_frame.grid_columnconfigure(2, weight=1)
        
        self.back_button = ctk.CTkButton(path_frame, text="< Voltar para Fonte de Dados", command=lambda: controller.show_data_source_frame(), width=180)
        self.back_button.grid(row=0, column=0, padx=(10, 5))
        ctk.CTkLabel(path_frame, text="Pasta de Trabalho:").grid(row=0, column=1, padx=(10,5))
        self.path_entry = ctk.CTkEntry(path_frame, state="readonly")
        self.path_entry.grid(row=0, column=2, padx=5, sticky="ew")
        
        main_container = ctk.CTkFrame(self, fg_color="transparent")
        main_container.grid(row=1, column=0, padx=10, pady=(0,5), sticky="nsew")
        main_container.grid_columnconfigure((0, 1), weight=1)
        main_container.grid_rowconfigure(0, weight=1)
        main_container.grid_rowconfigure(1, weight=2)
        
        q1_frame = ctk.CTkFrame(main_container); q1_frame.grid(row=0, column=0, padx=(0, 5), pady=(0, 5), sticky="nsew")
        q1_frame.grid_rowconfigure(1, weight=1); q1_frame.grid_columnconfigure(0, weight=1)
        ctk.CTkLabel(q1_frame, text="Scans Disponíveis", font=ctk.CTkFont(size=18, weight='bold')).grid(row=0, column=0, padx=10, pady=5)
        self.scrollable_scans_frame = ctk.CTkScrollableFrame(q1_frame); self.scrollable_scans_frame.grid(row=1, column=0, padx=5, pady=5, sticky="nsew")
        self.scan_checkboxes = {}

        q2_frame = ctk.CTkFrame(main_container); q2_frame.grid(row=0, column=1, padx=(5, 0), pady=(0, 5), sticky="nsew")
        q2_frame.grid_columnconfigure(1, weight=1)
        ctk.CTkLabel(q2_frame, text="Parâmetros", font=ctk.CTkFont(size=18, weight='bold')).grid(row=0, column=0, columnspan=2, pady=5, padx=10)
        self.param_entries = {}
        parameters = {"Percentil A Manter": "40.0", "Voxel Size": "0.001", "Neighbor Density Factor": "300.0", "Std Ratio": "1.7", "Min Neighbors": "200", "Max Neighbors": "10000"}
        param_keys = {"Percentil A Manter": "percentil_a_manter", "Voxel Size": "voxel_size", "Neighbor Density Factor": "neighbor_density_factor", "Std Ratio": "std_ratio", "Min Neighbors": "min_neighbors", "Max Neighbors": "max_neighbors"}
        for i, (p_label, p_val) in enumerate(parameters.items(), start=1):
            ctk.CTkLabel(q2_frame, text=f"{p_label}:").grid(row=i, column=0, padx=10, pady=5, sticky="w")
            entry = ctk.CTkEntry(q2_frame); entry.insert(0, p_val); entry.grid(row=i, column=1, padx=10, pady=5, sticky="ew")
            self.param_entries[param_keys[p_label]] = entry
        
        q3_frame = ctk.CTkFrame(main_container); q3_frame.grid(row=1, column=0, padx=(0, 5), pady=(5, 0), sticky="nsew")
        q3_frame.grid_rowconfigure(2, weight=1); q3_frame.grid_columnconfigure(0, weight=1)
        ctk.CTkLabel(q3_frame, text="Progresso e Logs", font=ctk.CTkFont(size=18, weight='bold')).grid(row=0, column=0, padx=10, pady=5)
        progress_frame_q3 = ctk.CTkFrame(q3_frame); progress_frame_q3.grid(row=1, column=0, padx=5, pady=5, sticky="ew")
        progress_frame_q3.grid_columnconfigure(0, weight=1)
        ctk.CTkLabel(progress_frame_q3, text="Geral:", width=50).pack(side="left", padx=(5,0))
        self.geral_progressbar = ctk.CTkProgressBar(progress_frame_q3); self.geral_progressbar.set(0); self.geral_progressbar.pack(side="left", fill="x", expand=True, padx=5)
        self.geral_progress_label = ctk.CTkLabel(progress_frame_q3, text="N/A", width=40); self.geral_progress_label.pack(side="left", padx=(0,5))
        ctk.CTkLabel(progress_frame_q3, text="Tarefa:", width=50).pack(side="left", padx=(5,0))
        self.tarefa_progressbar = ctk.CTkProgressBar(progress_frame_q3); self.tarefa_progressbar.set(0); self.tarefa_progressbar.pack(side="left", fill="x", expand=True, padx=5)
        self.tarefa_progress_label = ctk.CTkLabel(progress_frame_q3, text="0%", width=40); self.tarefa_progress_label.pack(side="left", padx=(0,5))
        self.log_textbox = ctk.CTkTextbox(q3_frame, state="disabled", activate_scrollbars=True); self.log_textbox.grid(row=2, column=0, padx=5, pady=5, sticky="nsew")
        
        q4_frame = ctk.CTkFrame(main_container); q4_frame.grid(row=1, column=1, padx=(5, 0), pady=(5, 0), sticky="nsew")
        q4_frame.grid_rowconfigure(1, weight=1); q4_frame.grid_columnconfigure(0, weight=1)
        ctk.CTkLabel(q4_frame, text="Visualizador de .PLY", font=ctk.CTkFont(size=18, weight='bold')).grid(row=0, column=0, padx=10, pady=5)
        self.scrollable_ply_frame = ctk.CTkScrollableFrame(q4_frame); self.scrollable_ply_frame.grid(row=1, column=0, padx=5, pady=5, sticky="nsew")
        
        footer_frame = ctk.CTkFrame(self, fg_color="transparent")
        footer_frame.grid(row=2, column=0, padx=10, pady=10, sticky="sew")
        footer_frame.grid_columnconfigure((0, 1), weight=1)
        self.start_button = ctk.CTkButton(footer_frame, text="Iniciar Processamento", command=self.start_processing_thread)
        self.start_button.grid(row=0, column=0, padx=10, pady=5, sticky="e")
        self.open_folder_button = ctk.CTkButton(footer_frame, text="Abrir Pasta de Trabalho", command=self.open_output_folder, state="disabled")
        self.open_folder_button.grid(row=0, column=1, padx=10, pady=5, sticky="w")
        
        self.log_queue = QueueIO()
        sys.stdout = self.log_queue
        self.after(100, self.update_log_from_queue)

    def load_folder_data(self, folder_path):
        print("\n" + "="*30 + " NOVA PASTA CARREGADA " + "="*30)
        self.log_textbox.configure(state="normal")
        self.log_textbox.delete("1.0", "end")
        self.log_textbox.configure(state="disabled")
        self.geral_progressbar.set(0)
        self.tarefa_progressbar.set(0)
        self.geral_progress_label.configure(text="N/A")
        self.tarefa_progress_label.configure(text="0%")
        self.path_entry.configure(state="normal")
        self.path_entry.delete(0, "end")
        self.path_entry.insert(0, folder_path)
        self.path_entry.configure(state="readonly")
        self.open_folder_button.configure(state="normal")
        
        for widget in self.scrollable_scans_frame.winfo_children(): widget.destroy()
        for widget in self.scrollable_ply_frame.winfo_children(): widget.destroy()
        self.loading_scans_label = ctk.CTkLabel(self.scrollable_scans_frame, text="Carregando...")
        self.loading_scans_label.pack(pady=10)
        self.loading_ply_label = ctk.CTkLabel(self.scrollable_ply_frame, text="Carregando...")
        self.loading_ply_label.pack(pady=10)
        
        threading.Thread(target=self._load_files_thread_target, args=(folder_path,), daemon=True).start()

    def _load_files_thread_target(self, base_path_str):
        base_path = Path(base_path_str)
        if not base_path.is_dir():
            self.after(0, self._populate_ui_from_thread, [], [])
            return
        scan_ids = sorted([m.group(1) for f in base_path.glob("raw_lidar_pan_*.txt") if (m := re.search(r'raw_lidar_pan_(\d+)\.txt', f.name)) and (base_path / f"raw_imu_pan_{m.group(1)}.txt").exists()], key=int)
        ply_files = sorted(list(base_path.glob("*.ply")), key=os.path.getmtime, reverse=True)
        self.after(0, self._populate_ui_from_thread, scan_ids, ply_files)

    def _populate_ui_from_thread(self, scan_ids, ply_files):
        for widget in self.scrollable_scans_frame.winfo_children(): widget.destroy()
        for widget in self.scrollable_ply_frame.winfo_children(): widget.destroy()
        
        self.scan_checkboxes = {}
        if not scan_ids:
            ctk.CTkLabel(self.scrollable_scans_frame, text="Nenhum par de scan (LIDAR/IMU) válido.").pack(pady=10)
        else:
            for scan_id in scan_ids:
                var = ctk.StringVar(value="on")
                cb = ctk.CTkCheckBox(self.scrollable_scans_frame, text=f"Scan ID: {scan_id}", variable=var, onvalue="on", offvalue="off")
                cb.pack(anchor="w", padx=10, pady=2, fill="x")
                self.scan_checkboxes[scan_id] = var

        self.ply_visualize_buttons.clear()
        if not ply_files:
            ctk.CTkLabel(self.scrollable_ply_frame, text="Nenhum arquivo .ply encontrado.").pack(pady=10)
        else:
            for ply_file in ply_files:
                item_frame = ctk.CTkFrame(self.scrollable_ply_frame, fg_color="transparent")
                item_frame.pack(fill="x", expand=True, pady=2)
                item_frame.grid_columnconfigure(0, weight=1)
                ctk.CTkLabel(item_frame, text=ply_file.name).grid(row=0, column=0, padx=5, sticky="w")
                vis_button = ctk.CTkButton(item_frame, text="Visualizar", width=80, command=lambda p=ply_file: self.visualize_ply_thread(p))
                vis_button.grid(row=0, column=1, padx=5)
                self.ply_visualize_buttons.append(vis_button)

    def set_controls_state(self, state):
        self.start_button.configure(state=state)
        self.back_button.configure(state=state)
        for button in self.ply_visualize_buttons:
            button.configure(state=state)

    def start_processing_thread(self):
        self.set_controls_state("disabled")
        self.start_button.configure(text="Processando...")
        self.log_textbox.configure(state="normal")
        self.log_textbox.delete("1.0", "end")
        self.log_textbox.configure(state="disabled")
        threading.Thread(target=self.run_processing_logic, daemon=True).start()

    def run_processing_logic(self):
        base_path_str = self.path_entry.get()
        base_path = Path(base_path_str)
        selected_scans = [scan_id for scan_id, var in self.scan_checkboxes.items() if var.get() == "on"]
        try:
            params = {key: float(entry.get()) for key, entry in self.param_entries.items()}
        except ValueError:
            print("ERRO: Parâmetros devem ser números.")
            self.after(0, lambda: self.set_controls_state("normal"))
            self.after(0, lambda: self.start_button.configure(text="Iniciar Processamento"))
            return
        total_scans = len(selected_scans)
        self.after(0, self.update_geral_progress, 0, total_scans)
        print(f"\nIniciando processamento para {total_scans} scans selecionados...")
        for i, scan_id in enumerate(selected_scans):
            self.after(0, self.update_geral_progress, i, total_scans)
            self.after(0, self.update_tarefa_progress, 0, 1)
            process_scan_pair(base_path, scan_id, params, progress_callback=lambda cur,_max: self.after(0, self.update_tarefa_progress, cur, _max))
            self.after(0, self.update_tarefa_progress, 1, 1)
        self.after(0, self.update_geral_progress, total_scans, total_scans)
        print("\n" + "#"*80 + "\nPROCESSAMENTO CONCLUÍDO!\n" + "#"*80)
        self.after(0, lambda: self.set_controls_state("normal"))
        self.after(0, lambda: self.start_button.configure(text="Iniciar Processamento"))
        self.after(0, self._populate_ui_from_thread, *self._load_files_for_update(base_path_str))

    def _load_files_for_update(self, base_path_str):
        base_path = Path(base_path_str)
        scan_ids = sorted([m.group(1) for f in base_path.glob("raw_lidar_pan_*.txt") if (m := re.search(r'raw_lidar_pan_(\d+)\.txt', f.name)) and (base_path / f"raw_imu_pan_{m.group(1)}.txt").exists()], key=int)
        ply_files = sorted(list(base_path.glob("*.ply")), key=os.path.getmtime, reverse=True)
        return scan_ids, ply_files

    def visualize_ply_thread(self, filepath):
        vis_process = multiprocessing.Process(target=show_ply_in_new_process, args=(filepath,), daemon=True)
        vis_process.start()
        
    def update_tarefa_progress(self, current_value, max_value):
        percentage = current_value / max_value if max_value > 0 else 0
        self.tarefa_progressbar.set(percentage)
        self.tarefa_progress_label.configure(text=f"{int(percentage * 100)}%")

    def update_geral_progress(self, current_scan, total_scans):
        percentage = current_scan / total_scans if total_scans > 0 else 0
        self.geral_progressbar.set(percentage)
        self.geral_progress_label.configure(text=f"{current_scan}/{total_scans}")

    def update_log_from_queue(self):
        while not self.log_queue.empty():
            line = self.log_queue.get_nowait()
            self.log_textbox.configure(state="normal")
            self.log_textbox.insert("end", line)
            self.log_textbox.see("end")
            self.log_textbox.configure(state="disabled")
        self.after(100, self.update_log_from_queue)

    def open_output_folder(self):
        if os.path.isdir(self.path_entry.get()):
            if sys.platform == "win32": os.startfile(self.path_entry.get())
            else: import subprocess; subprocess.Popen(["xdg-open", self.path_entry.get()])

if __name__ == "__main__":
    PI_HOSTNAME = "raspberrypi.local"
    PI_USERNAME = "admin"
    PI_PASSWORD = "admin"
    START_SCANNER_SCRIPT_PATH = "/home/admin/Downloads/zmq/ligar/start_scan_session.sh"
    STOP_SCANNER_SCRIPT_PATH = "/home/admin/Downloads/zmq/ligar/stop_scan_session.sh"
    
    multiprocessing.freeze_support()
    app = MainApplication()
    app.mainloop()