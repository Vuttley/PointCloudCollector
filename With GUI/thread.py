import numpy as np
import json
import csv
import time
import pandas as pd
import matplotlib.colors as colors
import matplotlib.pyplot as plt
import open3d as o3d
from scipy.interpolate import UnivariateSpline
from scipy.spatial import cKDTree
import os
import re
import shutil
from pathlib import Path

try:
    from tqdm import tqdm
except ImportError:
    print("Biblioteca 'tqdm' não encontrada. Para uma barra de progresso, instale com: pip install tqdm")
    def tqdm(iterator, *args, **kwargs):
        return iterator

def gerar_ply_otimizado(df, nome_arquivo="nuvem_de_pontos_otimizada.ply"):
    if df.empty:
        print("DataFrame vazio. Nenhum arquivo PLY será gerado.")
        return
    print(f"Iniciando geração otimizada de PLY para {len(df)} pontos...")
    start_time = time.time()
    distancias_m = df['distancia_lidar'].to_numpy() / 1000.0
    angulos_lidar_rad = np.deg2rad(df['angulo_lidar'].to_numpy())
    pitches_rad = -np.deg2rad(df['pitch'].to_numpy())
    yaws_rad = -np.deg2rad(df['yaw'].to_numpy())
    intensidades = df['intensidade_lidar'].to_numpy()
    normalizer = colors.Normalize(vmin=np.min(intensidades), vmax=np.max(intensidades))
    colormap = plt.get_cmap('viridis')
    cores_normalizadas = normalizer(intensidades)
    cores_rgba = colormap(cores_normalizadas)
    cores_rgb_uchar = (cores_rgba[:, :3] * 255).astype(np.uint8)
    d_cabeca_para_sensor = np.array([0, 0, 0.082])
    p_no_sensor = np.zeros((len(df), 3))
    p_no_sensor[:, 1] = distancias_m
    cos_a = np.cos(angulos_lidar_rad)
    sin_a = np.sin(angulos_lidar_rad)
    p_laser_rotacionado = np.zeros_like(p_no_sensor)
    p_laser_rotacionado[:, 0] = -distancias_m * sin_a
    p_laser_rotacionado[:, 1] =  distancias_m * cos_a
    vetor_total_na_cabeca = p_laser_rotacionado + d_cabeca_para_sensor
    cos_p = np.cos(pitches_rad); sin_p = np.sin(pitches_rad)
    x_cabeca = vetor_total_na_cabeca[:, 0]; y_cabeca = vetor_total_na_cabeca[:, 1]; z_cabeca = vetor_total_na_cabeca[:, 2]
    vetor_apos_pitch = np.zeros_like(vetor_total_na_cabeca)
    vetor_apos_pitch[:, 0] =  x_cabeca * cos_p + z_cabeca * sin_p
    vetor_apos_pitch[:, 1] =  y_cabeca
    vetor_apos_pitch[:, 2] = -x_cabeca * sin_p + z_cabeca * cos_p
    cos_y = np.cos(yaws_rad); sin_y = np.sin(yaws_rad)
    x_pitch = vetor_apos_pitch[:, 0]; y_pitch = vetor_apos_pitch[:, 1]; z_pitch = vetor_apos_pitch[:, 2]
    p_final_no_mundo = np.zeros_like(vetor_apos_pitch)
    p_final_no_mundo[:, 0] = x_pitch * cos_y - y_pitch * sin_y
    p_final_no_mundo[:, 1] = x_pitch * sin_y + y_pitch * cos_y
    p_final_no_mundo[:, 2] = z_pitch
    print("Cálculos finalizados. Escrevendo arquivo PLY...")
    pontos_com_cores = np.hstack((p_final_no_mundo, cores_rgb_uchar))
    num_pontos = len(df)
    header = ["ply", "format ascii 1.0", f"element vertex {num_pontos}", "property float x", "property float y", "property float z", "property uchar red", "property uchar green", "property uchar blue", "end_header"]
    try:
        np.savetxt(nome_arquivo, pontos_com_cores, fmt="%.4f %.4f %.4f %d %d %d", header="\n".join(header), comments='')
        end_time = time.time()
        print(f"Sucesso! Arquivo '{os.path.basename(str(nome_arquivo))}' salvo em {end_time - start_time:.2f} segundos.")
    except Exception as e:
        print(f"ERRO: Falha ao salvar o arquivo PLY: {e}")

def load_raw_data(filename):
    data = []
    print(f"Carregando dados de '{os.path.basename(str(filename))}'...")
    with open(filename, 'r') as f:
        for line in f:
            try:
                data.append(json.loads(line))
            except json.JSONDecodeError:
                print(f"Aviso: linha malformada ignorada em {os.path.basename(str(filename))}")
    print(f"Carregados {len(data)} registros.")
    return data

def process_and_filter_data(lidar_data, imu_data, output_filename, percentile_to_keep=98.0):
    start_time = time.time()
    if not lidar_data or not imu_data or len(imu_data) < 5:
        print("ERRO: Dados insuficientes para processamento.")
        return False
    lidar_timestamps_ns = np.array([p['timestamp_ns'] for p in lidar_data])
    lidar_distances = np.array([p['distance'] for p in lidar_data])
    lidar_angles = np.array([p['angle'] for p in lidar_data])
    lidar_intensities = np.array([p['intensity'] for p in lidar_data])
    imu_timestamps_ns = np.array([m['timestamp_ns'] for m in imu_data])
    imu_pitches = np.array([m['data']['pitch'] for m in imu_data])
    imu_yaws_unwrapped = np.unwrap([m['data']['yaw'] for m in imu_data], period=360)
    indices = np.searchsorted(imu_timestamps_ns, lidar_timestamps_ns, side='right')
    valid_indices_mask = (indices > 0) & (indices < len(imu_timestamps_ns))
    lidar_timestamps_ns = lidar_timestamps_ns[valid_indices_mask]
    lidar_distances = lidar_distances[valid_indices_mask]
    lidar_angles = lidar_angles[valid_indices_mask]
    lidar_intensities = lidar_intensities[valid_indices_mask]
    print(f"   {len(lidar_timestamps_ns)} pontos de LiDAR com timestamps válidos.")
    smoothing_factor = np.cbrt(len(imu_timestamps_ns))
    spline_pitch = UnivariateSpline(imu_timestamps_ns, imu_pitches, k=3, s=smoothing_factor)
    spline_yaw = UnivariateSpline(imu_timestamps_ns, imu_yaws_unwrapped, k=3, s=smoothing_factor)
    final_pitches = spline_pitch(lidar_timestamps_ns)
    final_yaws = spline_yaw(lidar_timestamps_ns)
    final_yaws = np.array([np.mean(final_yaws)]*len(final_yaws))
    final_rolls = np.zeros_like(lidar_timestamps_ns)
    print(f"Filtrando pontos instáveis (mantendo {percentile_to_keep}% dos mais estáveis)...")
    spline_pitch_acc = spline_pitch.derivative(n=2)
    pitch_accels = spline_pitch_acc(lidar_timestamps_ns)
    motion_instability = np.abs(pitch_accels)
    if motion_instability.size > 0:
        instability_threshold = np.percentile(motion_instability, percentile_to_keep)
        valid_motion_mask = motion_instability <= instability_threshold
        num_original = len(lidar_timestamps_ns)
        lidar_timestamps_ns = lidar_timestamps_ns[valid_motion_mask]
        lidar_distances = lidar_distances[valid_motion_mask]
        lidar_angles = lidar_angles[valid_motion_mask]
        lidar_intensities = lidar_intensities[valid_motion_mask]
        final_pitches = final_pitches[valid_motion_mask]
        final_yaws = final_yaws[valid_motion_mask]
        final_rolls = final_rolls[valid_motion_mask]
        num_final = len(lidar_timestamps_ns)
        num_descartados = num_original - num_final
        percent_descartados = (num_descartados / num_original) * 100 if num_original > 0 else 0
        print(f"   Filtro de movimento descartou {num_descartados} pontos ({percent_descartados:.2f}%).")
    else:
        print("   Nenhum ponto para filtrar.")
    print(f"Salvando {len(lidar_timestamps_ns)} pontos finais em '{os.path.basename(str(output_filename))}'...")
    fieldnames = ['distancia_lidar', 'angulo_lidar', 'intensidade_lidar', 'roll', 'pitch', 'yaw', 'timestamp']
    try:
        with open(output_filename, 'w', newline='', encoding='utf-8') as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            for i in range(len(lidar_timestamps_ns)):
                writer.writerow({'distancia_lidar': lidar_distances[i], 'angulo_lidar': lidar_angles[i], 'intensidade_lidar': lidar_intensities[i], 'roll': final_rolls[i], 'pitch': final_pitches[i], 'yaw': (final_yaws[i] + 180) % 360 - 180, 'timestamp': lidar_timestamps_ns[i]})
        processing_time = time.time() - start_time
        print(f"Sucesso! Processamento concluído em {processing_time:.2f} segundos.")
        return True
    except Exception as e:
        print(f"ERRO: Falha ao salvar o arquivo CSV: {e}")
        return False

def remove_statistical_outlier_variable_v2(pcd, df,
                                           distance_col='distancia_lidar', angle_col='angulo_lidar',
                                           neighbor_density_factor=200000.0, std_ratio=2.0,
                                           min_neighbors=10, max_neighbors=200,
                                           progress_callback=None):
    if len(pcd.points) != len(df):
        raise ValueError("A Nuvem de Pontos e o DataFrame devem ter o mesmo número de pontos.")
    if len(pcd.points) == 0:
        return pcd, []

    points = np.asarray(pcd.points)
    tree = cKDTree(points)
    distances = df[distance_col].to_numpy(dtype=float)
    angles = np.abs(np.deg2rad(df[angle_col].to_numpy(dtype=float)))
    distances[distances < 0.01] = 0.01
    angles[angles < 0.01] = 0.01
    k_values = neighbor_density_factor / (distances * angles)
    k_values = np.clip(k_values, min_neighbors, max_neighbors).astype(int)
    avg_neighbor_distances = np.zeros(len(points))

    print("Analisando vizinhanças dos pontos...")
    total_points = len(points)
    for i in range(total_points):
        k = k_values[i]
        neighbor_dists, _ = tree.query(points[i], k=k + 1, p=2)
        avg_neighbor_distances[i] = np.mean(neighbor_dists[1:])
        if progress_callback and (i % 100 == 0 or i == total_points - 1):
            progress_callback(i + 1, total_points)

    mean_of_avg_dists = np.mean(avg_neighbor_distances)
    std_dev_of_avg_dists = np.std(avg_neighbor_distances)
    distance_threshold = mean_of_avg_dists + std_ratio * std_dev_of_avg_dists
    inlier_indices = np.where(avg_neighbor_distances <= distance_threshold)[0]
    pcd_cleaned = pcd.select_by_index(inlier_indices)
    print(f"Filtragem concluída. Limiar de distância: {distance_threshold:.4f}. Pontos mantidos: {len(inlier_indices)} de {len(points)}")
    return pcd_cleaned, inlier_indices

def process_scan_pair(base_path: Path, scan_id: str, params: dict, progress_callback=None):
    print("\n" + "="*80)
    print(f"INICIANDO PROCESSAMENTO PARA O SCAN ID: {scan_id}")
    print("="*80)
    
    temp_dir = base_path / f"processamento_temporario_{scan_id}"
    try:
        temp_dir.mkdir(exist_ok=True, parents=True)
    except Exception as e:
        print(f"ERRO CRÍTICO: Não foi possível criar o diretório de trabalho temporário: {temp_dir}")
        print(f"Verifique as permissões de escrita. Erro: {e}")
        return None

    imu_file = base_path / f"raw_imu_pan_{scan_id}.txt"
    lidar_file = base_path / f"raw_lidar_pan_{scan_id}.txt"
    output_csv = temp_dir / f"{scan_id}_processado.csv"
    intermediate_ply = temp_dir / f"{scan_id}_intermediario.ply"
    final_output_ply = base_path / f"{scan_id}.ply"

    PERCENTIL_A_MANTER = params.get('percentil_a_manter', 60.0)
    NEIGHBOR_DENSITY_FACTOR = params.get('neighbor_density_factor', 300.0)
    STD_RATIO = params.get('std_ratio', 1.7)
    MIN_NEIGHBORS = int(params.get('min_neighbors', 200))
    MAX_NEIGHBORS = int(params.get('max_neighbors', 10000))
    VOXEL_SIZE = params.get('voxel_size', 0.001)

    try:
        imu_data = load_raw_data(imu_file)
        lidar_data = load_raw_data(lidar_file)
        if not process_and_filter_data(lidar_data, imu_data, output_csv, percentile_to_keep=PERCENTIL_A_MANTER):
            raise Exception("Falha na etapa de processamento e filtragem de dados.")

        df = pd.read_csv(output_csv)
        if df.empty:
            print(f"AVISO: Nenhum ponto de dados foi gerado para o scan {scan_id}. Pulando para o próximo.")
            return
        
        gerar_ply_otimizado(df, nome_arquivo=intermediate_ply)
        pcd = o3d.io.read_point_cloud(str(intermediate_ply))
        if not pcd.has_points():
             raise Exception(f"Falha ao ler o arquivo PLY intermediário ou o arquivo está vazio: {intermediate_ply}")

        print(f"Reduzindo a nuvem com Voxel Downsampling (tamanho do voxel: {VOXEL_SIZE})...")
        pcd_downsampled = pcd.voxel_down_sample(VOXEL_SIZE)
        print(f"Pontos após downsampling: {len(pcd_downsampled.points)}")

        print("Mapeando pontos da nuvem reduzida para o DataFrame original...")
        pcd_tree = o3d.geometry.KDTreeFlann(pcd)
        indices_map = [pcd_tree.search_knn_vector_3d(point, 1)[1][0] for point in np.asarray(pcd_downsampled.points)]
        df_downsampled = df.iloc[indices_map].reset_index(drop=True)
        print(f"Mapeamento concluído. A nuvem reduzida tem {len(df_downsampled)} pontos.")

        print("Iniciando remoção de outliers com densidade variável...")
        pcd_cleaned, ind = remove_statistical_outlier_variable_v2(
            pcd=pcd_downsampled, df=df_downsampled,
            neighbor_density_factor=NEIGHBOR_DENSITY_FACTOR, std_ratio=STD_RATIO,
            min_neighbors=MIN_NEIGHBORS, max_neighbors=MAX_NEIGHBORS,
            progress_callback=progress_callback
        )

        print("Estimando normais para a nuvem de pontos limpa...")
        radius_normal = VOXEL_SIZE * 2
        pcd_cleaned.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))
        o3d.io.write_point_cloud(str(final_output_ply), pcd_cleaned, write_ascii=True)
        print("-" * 80)
        print(f"SUCESSO! Nuvem de pontos final salva como '{final_output_ply.name}'")
        print("-" * 80)
        return final_output_ply

    except Exception as e:
        print("\n" + "!"*80)
        print(f"ERRO no processamento do scan ID {scan_id}: {e}")
        print("!"*80 + "\n")
        return None
    finally:
        if temp_dir.exists():
            print(f"Limpando arquivos temporários para o scan ID {scan_id}...")
            shutil.rmtree(temp_dir)
            print("Limpeza concluída.")