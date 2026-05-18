import os
import time
import random
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

from nav_utils import create_base_grid, get_navigable_cells, generate_frame_pedestrians, compute_occupancy
from antigravity_nav import compute_resultant_force, force_to_direction, is_valid_move, random_valid_move

# Constants
MAX_STEPS = 15000
BLOCK_THRESHOLD = 400.0
K_ATTRACTIVE = 1.0
K_REPULSIVE = 500.0
D0 = 1.5
MAX_FORCE_MAGNITUDE = 10.0

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

def run_antigravity_simulation(scenario_config, seed=42):
    random.seed(seed)
    np.random.seed(seed)

    rows, cols = scenario_config['grid']
    n_frames = scenario_config['frames']
    lambda_val = scenario_config['lambda_poisson']
    start = scenario_config['start']

    base_grid = create_base_grid(rows, cols, obstacle_ratio=0.05, seed=seed)
    navigable = get_navigable_cells(base_grid)

    robot = start
    cleaned = {robot}
    passos = 0
    consecutive_fails = 0
    records = []
    
    # Store path for trajectory plotting
    trajectory = [robot]
    last_occ_grid = None

    for frame in range(1, n_frames + 1):
        pedestrians = generate_frame_pedestrians(
            frame_idx=frame,
            lambda_val=lambda_val,
            grid_rows=rows,
            grid_cols=cols,
            seed=seed
        )

        occ_grid = compute_occupancy(rows, cols, pedestrians)
        last_occ_grid = occ_grid

        if passos >= MAX_STEPS or len(cleaned) >= len(navigable):
            break

        not_cleaned = [cell for cell in navigable if cell not in cleaned]
        if not not_cleaned:
            break

        next_goal = min(not_cleaned,
                       key=lambda c: abs(c[0]-robot[0]) + abs(c[1]-robot[1]))

        t0 = time.perf_counter()

        F_total = compute_resultant_force(
            robot, next_goal, occ_grid,
            k_att=K_ATTRACTIVE,
            k_rep=K_REPULSIVE,
            d0=D0,
            block_threshold=BLOCK_THRESHOLD,
            max_force=MAX_FORCE_MAGNITUDE
        )

        dr, dc = force_to_direction(F_total)

        if not is_valid_move(robot, dr, dc, rows, cols, occ_grid, BLOCK_THRESHOLD):
            dr, dc = random_valid_move(robot, rows, cols, occ_grid, BLOCK_THRESHOLD, seed)

        new_r = robot[0] + dr
        new_c = robot[1] + dc

        time_ms = (time.perf_counter() - t0) * 1000

        if (dr, dc) != (0, 0) and (new_r, new_c) != robot:
            robot = (new_r, new_c)
            cleaned.add(robot)
            trajectory.append(robot)
            passos += 1
            action = "ADVANCE"
            consecutive_fails = 0
            success = True
        else:
            action = "WAIT"
            consecutive_fails += 1
            success = False

            if consecutive_fails > 5:
                nearby = [c for c in not_cleaned
                         if abs(c[0]-robot[0]) + abs(c[1]-robot[1]) < 15]
                if nearby:
                    next_goal = random.choice(nearby)
                consecutive_fails = 0

        records.append({
            'scenario': scenario_config['name'],
            'frame': frame,
            'robot_r': robot[0],
            'robot_c': robot[1],
            'goal_r': next_goal[0],
            'goal_c': next_goal[1],
            'force_r': F_total[0],
            'force_c': F_total[1],
            'dr': dr,
            'dc': dc,
            'time_ms': time_ms,
            'success': success,
            'action': action,
            'pedestrians_count': len(pedestrians),
            'max_occupancy': occ_grid.max(),
            'coverage': len(cleaned) / len(navigable) * 100
        })

    return records, trajectory, last_occ_grid

def generate_plots(records, trajectory, last_occ_grid, scenario_name, out_dir):
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

    # 2. fig_force_magnitude.png
    df['force_magnitude'] = np.sqrt(df['force_r']**2 + df['force_c']**2)
    plt.figure(figsize=(10, 6))
    plt.plot(df['frame'], df['force_magnitude'], color='orange')
    plt.title(f'Force Magnitude per Frame - {scenario_name}')
    plt.xlabel('Frame')
    plt.ylabel('Force Magnitude')
    plt.grid(True)
    plt.savefig(os.path.join(out_dir, f'{scenario_name}_fig_force_magnitude.png'), dpi=300)
    plt.close()

    # 3. fig_time_per_frame.png
    plt.figure(figsize=(10, 6))
    plt.plot(df['frame'], df['time_ms'], color='green')
    plt.title(f'Time per Frame (ms) - {scenario_name}')
    plt.xlabel('Frame')
    plt.ylabel('Time (ms)')
    plt.grid(True)
    plt.savefig(os.path.join(out_dir, f'{scenario_name}_fig_time_per_frame.png'), dpi=300)
    plt.close()

    # 4. fig_trajectory.png
    plt.figure(figsize=(10, 10))
    traj_r = [p[0] for p in trajectory]
    traj_c = [p[1] for p in trajectory]
    plt.plot(traj_c, traj_r, marker='.', color='blue', alpha=0.5, label='Robot Path')
    plt.plot(trajectory[0][1], trajectory[0][0], 'go', label='Start')
    plt.plot(trajectory[-1][1], trajectory[-1][0], 'ro', label='End')
    plt.title(f'Trajectory - {scenario_name}')
    plt.xlabel('Columns')
    plt.ylabel('Rows')
    plt.gca().invert_yaxis()
    plt.legend()
    plt.grid(True)
    plt.savefig(os.path.join(out_dir, f'{scenario_name}_fig_trajectory.png'), dpi=300)
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
    out_dir = 'output/antigravity'
    os.makedirs(out_dir, exist_ok=True)

    all_records = []
    summaries = []
    trajectories_by_scenario = {}  # para overlay com Dijkstra

    for name, config in SCENARIOS.items():
        print(f"Running AntiGravity for {name}...")
        records, trajectory, last_occ_grid = run_antigravity_simulation(config)
        all_records.extend(records)
        trajectories_by_scenario[name] = trajectory
        
        generate_plots(records, trajectory, last_occ_grid, name, out_dir)
        
        df = pd.DataFrame(records)
        if not df.empty:
            df['force_magnitude'] = np.sqrt(df['force_r']**2 + df['force_c']**2)
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
                'avg_force_magnitude': df['force_magnitude'].mean(),
                'max_force_magnitude': df['force_magnitude'].max(),
                'total_time_ms': df['time_ms'].sum(),
                'avg_time_ms': df['time_ms'].mean(),
                'max_time_ms': df['time_ms'].max(),
                'success_rate': df[df['action'] == 'ADVANCE'].shape[0] / len(df) * 100,
                'wait_count': df[df['action'] == 'WAIT'].shape[0],
                'max_consecutive_waits': (df['action'] == 'WAIT').astype(int).groupby((df['action'] != 'WAIT').cumsum()).sum().max(),
            }
            summaries.append(summary)

    pd.DataFrame(all_records).to_csv(os.path.join(out_dir, 'antigravity_raw.csv'), index=False)
    pd.DataFrame(summaries).to_csv(os.path.join(out_dir, 'antigravity_summary.csv'), index=False)

    # Gráfico de trajetórias sobrepostas com Dijkstra (se CSV existir)
    dijk_raw_path = 'output/dijkstra/dijkstra_raw.csv'
    if os.path.exists(dijk_raw_path):
        dijk_df = pd.read_csv(dijk_raw_path)
        for name in SCENARIOS:
            ag_traj = trajectories_by_scenario.get(name, [])
            d_df = dijk_df[dijk_df['scenario'] == name]
            if ag_traj and not d_df.empty:
                rows_grid, cols_grid = SCENARIOS[name]['grid']
                fig, ax = plt.subplots(figsize=(12, 8))
                # AntiGravity trajectory
                ag_c = [p[1] for p in ag_traj]
                ag_r = [p[0] for p in ag_traj]
                ax.plot(ag_c, ag_r, color='blue', alpha=0.6, linewidth=1, label='AntiGravity')
                ax.plot(ag_c[0], ag_r[0], 'bs', markersize=8)
                ax.plot(ag_c[-1], ag_r[-1], 'b^', markersize=8)
                # Dijkstra trajectory
                dijk_r = d_df['robot_r'].tolist()
                dijk_c = d_df['robot_c'].tolist()
                ax.plot(dijk_c, dijk_r, color='red', alpha=0.6, linewidth=1, label='Dijkstra-Std')
                ax.plot(dijk_c[0], dijk_r[0], 'rs', markersize=8)
                ax.plot(dijk_c[-1], dijk_r[-1], 'r^', markersize=8)
                ax.set_xlim(0, cols_grid)
                ax.set_ylim(rows_grid, 0)
                ax.set_title(f'Trajetórias Sobrepostas — {name} (quadrado=início, triângulo=fim)')
                ax.set_xlabel('Coluna')
                ax.set_ylabel('Linha')
                ax.legend()
                ax.grid(True, alpha=0.3)
                fig.savefig(os.path.join(out_dir, f'{name}_fig_trajectory_overlay.png'), dpi=300, bbox_inches='tight')
                plt.close(fig)
                print(f"  Overlay salvo: {name}_fig_trajectory_overlay.png")
    
    print("AntiGravity simulation finished successfully.")

if __name__ == '__main__':
    main()
