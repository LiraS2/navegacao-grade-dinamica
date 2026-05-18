import os
import time
import random
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

from nav_utils import create_base_grid, get_navigable_cells, generate_frame_pedestrians, compute_occupancy, create_base_graph_and_navigable
from dijkstra_nav import build_frame_graph, dijkstra_std

# Constants
MAX_STEPS = 15000
BLOCK_THRESHOLD = 400.0

SCENARIOS = {
    "BR-06": {
        "name": "BR-06",
        "arena_m": (25, 10),
        "grid": (25, 63),
        "navigable_estimate": 639,
        "frames": 400,
        "lambda_poisson": 8.79,
        "start": (0, 0),
        "end": (24, 62),
    },
    "CN-01": {
        "name": "CN-01",
        "arena_m": (15, 20),
        "grid": (50, 38),
        "navigable_estimate": 599,
        "frames": 99,
        "lambda_poisson": 34.32,
        "start": (0, 0),
        "end": (49, 37),
    }
}

def manhattan_distance(a, b):
    return abs(a[0]-b[0]) + abs(a[1]-b[1])

def run_simulation(scenario_config, seed=42):
    random.seed(seed)
    np.random.seed(seed)

    rows, cols = scenario_config['grid']
    n_frames = scenario_config['frames']
    lambda_val = scenario_config['lambda_poisson']
    start = scenario_config['start']

    # 1. Criar grafo base
    base_graph, navigable = create_base_graph_and_navigable(rows, cols, obstacle_ratio=0.05, seed=seed)

    # 2. Estado do robo
    robot = start
    cleaned = {robot}
    passos = 0
    consecutive_fails = 0

    records = []
    last_occ_grid = None
    
    # Loop limitado estritamente aos frames do dataset (spec v2.2)
    for frame in range(1, n_frames + 1):
        # 3. Gerar pedestres deste frame (DETERMINISTICO)
        pedestrians = generate_frame_pedestrians(
            frame_idx=frame,
            lambda_val=lambda_val,
            grid_rows=rows,
            grid_cols=cols,
            seed=seed
        )

        # 4. Calcular ocupacao
        occ_grid = compute_occupancy(rows, cols, pedestrians)
        last_occ_grid = occ_grid

        # 5. Construir grafo dinamico
        frame_graph = build_frame_graph(base_graph, occ_grid, BLOCK_THRESHOLD)

        # 6. Escolher goal
        if passos >= MAX_STEPS or len(cleaned) >= len(navigable):
            break

        not_cleaned = [cell for cell in navigable if cell not in cleaned]
        if not not_cleaned:
            break

        # Goal: nao-limpado mais proximo (Manhattan)
        next_goal = min(not_cleaned, 
                       key=lambda c: manhattan_distance(c, robot))

        # 7. Busca Dijkstra
        result = dijkstra_std(frame_graph, robot, next_goal)

        # 8. Executar acao
        if result['success'] and result['path_length'] >= 1:
            robot = result['path'][1]  # avanca 1 celula
            cleaned.add(robot)
            passos += 1
            action = "ADVANCE"
            consecutive_fails = 0
        else:
            action = "WAIT"
            consecutive_fails += 1

            if consecutive_fails > 5:
                # BACKTRACK: escolher goal aleatorio proximo
                nearby = [c for c in not_cleaned 
                         if manhattan_distance(c, robot) < 20]
                if nearby:
                    next_goal = random.choice(nearby)
                else:
                    next_goal = random.choice(not_cleaned)
                consecutive_fails = 0

        # 9. Registrar metricas
        records.append({
            'scenario': scenario_config['name'],
            'frame': frame,
            'robot_r': robot[0],
            'robot_c': robot[1],
            'goal_r': next_goal[0],
            'goal_c': next_goal[1],
            'nodes_expanded': result['nodes_expanded'],
            'max_queue_size': result['max_queue_size'],
            'time_ms': result['time_ms'],
            'path_cost': result['path_cost'],
            'path_length': result['path_length'],
            'success': result['success'],
            'action': action,
            'pedestrians_count': len(pedestrians),
            'max_occupancy': occ_grid.max(),
            'coverage': len(cleaned) / len(navigable) * 100
        })

    return records, last_occ_grid

def generate_plots(records, last_occ_grid, scenario_name, out_dir):
    df = pd.DataFrame(records)
    if df.empty:
        return
        
    # 1. fig_coverage_over_time.png
    plt.figure(figsize=(10, 6))
    plt.plot(df['frame'], df['coverage'], label='Coverage %')
    plt.title(f'Coverage over Time - {scenario_name}')
    plt.xlabel('Frame')
    plt.ylabel('Coverage (%)')
    plt.grid(True)
    plt.legend()
    plt.savefig(os.path.join(out_dir, f'{scenario_name}_fig_coverage_over_time.png'), dpi=300)
    plt.close()

    # 2. fig_nodes_expanded_dist.png
    plt.figure(figsize=(10, 6))
    df_success = df[df['success'] == True]
    if not df_success.empty:
        plt.hist(df_success['nodes_expanded'], bins=20, color='purple', edgecolor='black')
        plt.title(f'Nodes Expanded Distribution - {scenario_name}')
        plt.xlabel('Nodes Expanded')
        plt.ylabel('Frequency')
        plt.grid(True)
        plt.savefig(os.path.join(out_dir, f'{scenario_name}_fig_nodes_expanded_dist.png'), dpi=300)
    plt.close()

    # 3. fig_time_per_search.png
    plt.figure(figsize=(10, 6))
    plt.plot(df['frame'], df['time_ms'], color='green')
    plt.title(f'Time per Search (ms) - {scenario_name}')
    plt.xlabel('Frame')
    plt.ylabel('Time (ms)')
    plt.grid(True)
    plt.savefig(os.path.join(out_dir, f'{scenario_name}_fig_time_per_search.png'), dpi=300)
    plt.close()

    # 4. fig_queue_size.png
    plt.figure(figsize=(10, 6))
    plt.plot(df['frame'], df['max_queue_size'], color='red')
    plt.title(f'Max Queue Size per Search - {scenario_name}')
    plt.xlabel('Frame')
    plt.ylabel('Max Queue Size')
    plt.grid(True)
    plt.savefig(os.path.join(out_dir, f'{scenario_name}_fig_queue_size.png'), dpi=300)
    plt.close()

    # 5. fig_occupancy_heatmap.png
    plt.figure(figsize=(10, 10))
    plt.imshow(last_occ_grid, cmap='hot', interpolation='nearest')
    plt.colorbar(label='Occupancy Value')
    plt.title(f'Occupancy Heatmap (Last Frame) - {scenario_name}')
    plt.xlabel('Columns')
    plt.ylabel('Rows')
    plt.savefig(os.path.join(out_dir, f'{scenario_name}_fig_occupancy_heatmap.png'), dpi=300)
    plt.close()


def main():
    out_dir = 'output/dijkstra'
    os.makedirs(out_dir, exist_ok=True)

    all_records = []
    summaries = []

    for name, config in SCENARIOS.items():
        print(f"Running Dijkstra for {name}...")
        records, last_occ_grid = run_simulation(config)
        all_records.extend(records)
        
        generate_plots(records, last_occ_grid, name, out_dir)
        
        df = pd.DataFrame(records)
        if not df.empty:
            total_steps = df[df['action'] == 'ADVANCE'].shape[0]
            coverage_pct = df['coverage'].iloc[-1]
            n_frames_val = config['frames']
            summary = {
                'scenario': name,
                'n_frames': n_frames_val,
                'total_steps': total_steps,
                'coverage_percent': coverage_pct,
                'coverage_per_step': (coverage_pct / total_steps) if total_steps > 0 else 0,
                'advance_rate': (total_steps / n_frames_val * 100),
                'avg_nodes_expanded': df[df['success'] == True]['nodes_expanded'].mean() if not df[df['success'] == True].empty else 0,
                'max_nodes_expanded': df['nodes_expanded'].max(),
                'avg_queue_size': df[df['success'] == True]['max_queue_size'].mean() if not df[df['success'] == True].empty else 0,
                'total_time_ms': df['time_ms'].sum(),
                'avg_time_ms': df['time_ms'].mean(),
                'max_time_ms': df['time_ms'].max(),
                'success_rate': df[df['success'] == True].shape[0] / len(df) * 100,
                'wait_count': df[df['action'] == 'WAIT'].shape[0],
                'max_consecutive_waits': (df['action'] == 'WAIT').astype(int).groupby((df['action'] != 'WAIT').cumsum()).sum().max(),
            }
            summaries.append(summary)

    pd.DataFrame(all_records).to_csv(os.path.join(out_dir, 'dijkstra_raw.csv'), index=False)
    pd.DataFrame(summaries).to_csv(os.path.join(out_dir, 'dijkstra_summary.csv'), index=False)
    
    print("Dijkstra simulation finished successfully.")

if __name__ == '__main__':
    main()
