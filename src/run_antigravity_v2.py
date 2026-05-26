import os
import time
import random
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from collections import deque

from nav_utils import create_base_grid, get_navigable_cells, generate_frame_pedestrians, compute_occupancy
from antigravity_nav_v2 import compute_resultant_force_v2, force_to_direction, is_valid_move, smart_fallback

# Constants
MAX_STEPS = 15000
BLOCK_THRESHOLD = 400.0
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
        "config": {
            "K_ATTRACTIVE": 1.0,
            "K_REPULSIVE": 100.0,
            "D0": 0.8,
            "GOAL_THRESHOLD": 1.0,
            "CORRIDOR_MODE": True,
            "GOAL_ZONE_RADIUS": 3,
            "HISTORY_LEN": 5,
            "FALLBACK_HP": 10.0,
            "FALLBACK_OF": 0.01,
            "FALLBACK_GF": 10.0,
        }
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
        "config": {
            "K_ATTRACTIVE": 2.0,
            "K_REPULSIVE": 300.0,
            "D0": 1.5,
            "GOAL_THRESHOLD": 2.0,
            "CORRIDOR_MODE": True,
            "GOAL_ZONE_RADIUS": 5,
            "HISTORY_LEN": 3,
            "FALLBACK_HP": 10.0,
            "FALLBACK_OF": 0.01,
            "FALLBACK_GF": 1.0,
        }
    }
}

def run_antigravity_simulation(scenario_config, seed=42):
    random.seed(seed)
    np.random.seed(seed)

    rows, cols = scenario_config['grid']
    n_frames = scenario_config['frames']
    lambda_val = scenario_config['lambda_poisson']
    start = scenario_config['start']
    config = scenario_config['config']

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

    visited_history = deque(maxlen=config.get("HISTORY_LEN", 10))
    visited_history.append(robot)

    corridors_blocked_count = 0
    goal_oscillation_count = 0
    fallback_used_count = 0
    fallback_success_count = 0

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

        F_total, is_in_corridor = compute_resultant_force_v2(
            robot, next_goal, occ_grid, config,
            block_threshold=BLOCK_THRESHOLD,
            max_force=MAX_FORCE_MAGNITUDE
        )

        if is_in_corridor:
            corridors_blocked_count += 1

        dr, dc = force_to_direction(F_total)
        fallback_triggered = False

        if not is_valid_move(robot, dr, dc, rows, cols, occ_grid, BLOCK_THRESHOLD):
            fallback_triggered = True
            fallback_used_count += 1
            hp = config.get("FALLBACK_HP", 50.0)
            of = config.get("FALLBACK_OF", 1.0)
            gf = config.get("FALLBACK_GF", 1.0)
            dr, dc = smart_fallback(robot, next_goal, occ_grid, rows, cols, visited_history, BLOCK_THRESHOLD, hp=hp, of=of, gf=gf)

        new_r = robot[0] + dr
        new_c = robot[1] + dc

        # Check for oscillation (if we move to a cell we've just been in, while near the goal)
        if len(visited_history) >= 2 and (new_r, new_c) == visited_history[-2]:
            dist_to_goal = abs(robot[0]-next_goal[0]) + abs(robot[1]-next_goal[1])
            if dist_to_goal <= config["GOAL_ZONE_RADIUS"]:
                goal_oscillation_count += 1

        time_ms = (time.perf_counter() - t0) * 1000

        if (dr, dc) != (0, 0) and (new_r, new_c) != robot:
            robot = (new_r, new_c)
            cleaned.add(robot)
            trajectory.append(robot)
            visited_history.append(robot)
            passos += 1
            action = "ADVANCE"
            consecutive_fails = 0
            success = True
            if fallback_triggered:
                fallback_success_count += 1
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
            'coverage': len(cleaned) / len(navigable) * 100,
            'is_in_corridor': is_in_corridor,
            'fallback_triggered': fallback_triggered
        })

    extra_metrics = {
        'corridors_blocked': corridors_blocked_count,
        'goal_oscillation_count': goal_oscillation_count,
        'fallback_used': fallback_used_count,
        'fallback_success': fallback_success_count
    }

    return records, trajectory, last_occ_grid, extra_metrics

def generate_plots(records, trajectory, last_occ_grid, scenario_name, out_dir):
    df = pd.DataFrame(records)
    if df.empty:
        return
        
    # 1. fig_coverage_over_time.png
    plt.figure(figsize=(10, 6))
    plt.plot(df['frame'], df['coverage'], label='Coverage % (v2.0)')
    plt.title(f'Coverage over Time - {scenario_name} (AntiGravity v2.0)')
    plt.xlabel('Frame')
    plt.ylabel('Coverage (%)')
    plt.grid(True)
    plt.legend()
    plt.savefig(os.path.join(out_dir, f'{scenario_name}_fig_coverage_over_time.png'), dpi=300)
    plt.close()

    # 4. fig_trajectory.png
    plt.figure(figsize=(10, 10))
    traj_r = [p[0] for p in trajectory]
    traj_c = [p[1] for p in trajectory]
    plt.plot(traj_c, traj_r, marker='.', color='blue', alpha=0.5, label='Robot Path v2.0')
    plt.plot(trajectory[0][1], trajectory[0][0], 'go', label='Start')
    plt.plot(trajectory[-1][1], trajectory[-1][0], 'ro', label='End')
    plt.title(f'Trajectory - {scenario_name} (AntiGravity v2.0)')
    plt.xlabel('Columns')
    plt.ylabel('Rows')
    plt.gca().invert_yaxis()
    plt.legend()
    plt.grid(True)
    plt.savefig(os.path.join(out_dir, f'{scenario_name}_fig_trajectory.png'), dpi=300)
    plt.close()

def main():
    out_dir = '../output/antigravity_v2'
    os.makedirs(out_dir, exist_ok=True)

    all_records = []
    summaries = []
    trajectories_by_scenario = {}

    for name, config in SCENARIOS.items():
        print(f"Running AntiGravity v2.0 for {name}...")
        records, trajectory, last_occ_grid, extra_metrics = run_antigravity_simulation(config)
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
                'avg_time_ms': df['time_ms'].mean(),
                'success_rate': df[df['action'] == 'ADVANCE'].shape[0] / len(df) * 100,
                'wait_count': df[df['action'] == 'WAIT'].shape[0],
                'corridors_blocked': extra_metrics['corridors_blocked'],
                'goal_oscillation_count': extra_metrics['goal_oscillation_count'],
                'fallback_used': extra_metrics['fallback_used'],
                'fallback_success': extra_metrics['fallback_success']
            }
            summaries.append(summary)

    pd.DataFrame(all_records).to_csv(os.path.join(out_dir, 'antigravity_v2_raw.csv'), index=False)
    pd.DataFrame(summaries).to_csv(os.path.join(out_dir, 'antigravity_v2_summary.csv'), index=False)

    print("AntiGravity v2.0 simulation finished successfully.")

if __name__ == '__main__':
    main()
