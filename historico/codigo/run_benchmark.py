import os
import random
import time
import math
from collections import defaultdict
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
import networkx as nx

from antigravity import dijkstra_path

# ─────────────────────────────────────────────────────────────────────────────
# CONFIGURATION
# ─────────────────────────────────────────────────────────────────────────────
CONFIG = {
    "seed": 42,
    "max_steps": 15000,
    "max_wait_replans": 8,
    "cell_size_m": 0.40,
    "wbase": 1.0,
    "pmax": 500.0,
    "influence_radius_cells": 1.5,
    "proximal_factor": 2.0,
    "pedestrian_radius_m": 0.45,
    "block_threshold": 400.0,
    "scenarios": {
        "BR-06": {"dims_m": (25, 10), "frames": 400, "lambda_ped": 8.79},
        "CN-01": {"dims_m": (15, 20), "frames": 99,  "lambda_ped": 34.32},
    }
}

OUTPUT_DIR = "output/benchmark"

# ─────────────────────────────────────────────────────────────────────────────
# HELPER FUNCTIONS
# ─────────────────────────────────────────────────────────────────────────────
def manhattan(u, v):
    return abs(u[0] - v[0]) + abs(u[1] - v[1])

def create_base_grid(dims_m, cell_size_m, seed):
    random.seed(seed)
    np.random.seed(seed)
    
    cols = int(dims_m[0] / cell_size_m)
    rows = int(dims_m[1] / cell_size_m)
    
    G = nx.grid_2d_graph(rows, cols)
    for u, v in G.edges():
        G[u][v]["weight"] = 1.0
        
    all_nodes = list(G.nodes())
    start_corner = (0, 0)
    goal_corner = (rows - 1, cols - 1)
    
    safe_nodes = {start_corner, goal_corner}
    for u in [start_corner, goal_corner]:
        if u in G:
            safe_nodes.update(G.neighbors(u))
            
    removable = [n for n in all_nodes if n not in safe_nodes]
    num_to_remove = int(len(all_nodes) * 0.05)
    
    to_remove = random.sample(removable, num_to_remove)
    G.remove_nodes_from(to_remove)
    
    components = list(nx.connected_components(G))
    if len(components) > 1:
        largest = max(components, key=len)
        nodes_to_remove = [n for n in G.nodes() if n not in largest]
        G.remove_nodes_from(nodes_to_remove)
        
    return G, rows, cols

def apply_dynamic_weights(G, rows, cols, num_pedestrians):
    Groute = G.copy()
    if num_pedestrians == 0:
        return Groute
        
    nodes = list(Groute.nodes())
    if not nodes:
        return Groute
        
    pedestrians = [random.choice(nodes) for _ in range(num_pedestrians)]
    
    occ = defaultdict(float)
    sigma = CONFIG["influence_radius_cells"]
    sigma2_2 = 2 * (sigma ** 2)
    pmax = CONFIG["pmax"]
    prox_factor = CONFIG["proximal_factor"]
    
    for r, c in Groute.nodes():
        node_occ = 0.0
        for pr, pc in pedestrians:
            dist_sq = (r - pr)**2 + (c - pc)**2
            node_occ += pmax * math.exp(-dist_sq / sigma2_2) * prox_factor
        occ[(r, c)] = node_occ
        
    edges_to_remove = []
    for u, v in Groute.edges():
        w = CONFIG["wbase"] + max(occ[u], occ[v])
        if w > CONFIG["block_threshold"]:
            edges_to_remove.append((u, v))
        else:
            Groute[u][v]["weight"] = w
            
    Groute.remove_edges_from(edges_to_remove)
    return Groute

# ─────────────────────────────────────────────────────────────────────────────
# MAIN BENCHMARK LOOP
# ─────────────────────────────────────────────────────────────────────────────
def run_scenario(scenario_name, config_data):
    print(f"Starting scenario {scenario_name}...")
    Gstatic, rows, cols = create_base_grid(config_data["dims_m"], CONFIG["cell_size_m"], CONFIG["seed"])
    navigable_cells = set(Gstatic.nodes())
    print(f"  Navigable cells: {len(navigable_cells)}")
    
    records = []
    
    for algo_id in ["Astar", "Dijk-Std", "Dijk-Bi", "Dijk-Dial"]:
        print(f"  Running algorithm: {algo_id}")
        
        random.seed(CONFIG["seed"])
        np.random.seed(CONFIG["seed"])
        
        unvisited = set(Gstatic.nodes())
        robot = min(unvisited, key=lambda n: manhattan(n, (0, 0)))
        unvisited.remove(robot)
        
        for frame in range(1, config_data["frames"] + 1):
            if not unvisited:
                break
                
            goal = min(unvisited, key=lambda n: manhattan(robot, n))
            num_ped = np.random.poisson(config_data["lambda_ped"])
            Groute = apply_dynamic_weights(Gstatic, rows, cols, num_ped)
            
            success = False
            path = None
            metrics = None
            
            if algo_id == "Astar":
                t0 = time.perf_counter()
                try:
                    path = nx.astar_path(Groute, robot, goal, heuristic=manhattan, weight='weight')
                    t_ms = (time.perf_counter() - t0) * 1000
                    cost = sum(Groute[path[i]][path[i+1]]["weight"] for i in range(len(path)-1))
                    
                    success = True
                    metrics = {
                        "nodes_expanded": np.nan, 
                        "max_queue_size": np.nan,
                        "time_ms": t_ms,
                        "path_cost": cost,
                        "path_length": len(path) - 1
                    }
                except nx.NetworkXNoPath:
                    t_ms = (time.perf_counter() - t0) * 1000
                    metrics = {
                        "nodes_expanded": 0,
                        "max_queue_size": 0,
                        "time_ms": t_ms,
                        "path_cost": float('inf'),
                        "path_length": 0
                    }
            else:
                variant_map = {
                    "Dijk-Std": "standard",
                    "Dijk-Bi": "bidirectional",
                    "Dijk-Dial": "dial"
                }
                path, metrics = dijkstra_path(Groute, robot, goal, variant=variant_map[algo_id])
                success = path is not None
                if not success:
                    metrics["path_cost"] = float('inf')
                    metrics["path_length"] = 0
            
            max_occ = 0.0
            if success and path and len(path) > 1:
                max_occ = max((Groute[path[i]][path[i+1]]["weight"] - CONFIG["wbase"] for i in range(len(path)-1)), default=0.0)
            
            records.append({
                "frame": frame,
                "algorithm": algo_id,
                "scenario": scenario_name,
                "robot_r": robot[0], "robot_c": robot[1],
                "goal_r": goal[0], "goal_c": goal[1],
                "nodes_expanded": metrics["nodes_expanded"],
                "max_queue_size": metrics["max_queue_size"],
                "time_ms": metrics["time_ms"],
                "path_cost": metrics["path_cost"],
                "path_length": metrics["path_length"],
                "success": success,
                "pedestrians_count": num_ped,
                "max_occupancy": max_occ
            })
            
            if success and path and len(path) > 1:
                robot = path[1]
                if robot in unvisited:
                    unvisited.remove(robot)
                    
    return records

# ─────────────────────────────────────────────────────────────────────────────
# AGGREGATION & VISUALIZATION
# ─────────────────────────────────────────────────────────────────────────────
def generate_summary(df_raw):
    summary = []
    for (scenario, algo), group in df_raw.groupby(["scenario", "algorithm"]):
        success_group = group[group["success"] == True]
        
        mean_nodes = success_group["nodes_expanded"].mean() if not success_group.empty else np.nan
        max_nodes = success_group["nodes_expanded"].max() if not success_group.empty else np.nan
        mean_queue = group["max_queue_size"].mean()
        
        total_time = group["time_ms"].sum()
        mean_time = group["time_ms"].mean()
        
        success_rate = group["success"].mean()
        steps = len(group)
        failures = len(group[group["success"] == False])
        
        summary.append({
            "scenario": scenario,
            "algorithm": algo,
            "mean_nodes_expanded": mean_nodes,
            "max_nodes_expanded": max_nodes,
            "mean_max_queue_size": mean_queue,
            "total_time_ms": total_time,
            "mean_time_ms": mean_time,
            "success_rate": success_rate,
            "total_steps": steps,
            "failures": failures
        })
    return pd.DataFrame(summary)

def generate_plots(df_raw, df_summary):
    sns.set_theme(style="whitegrid")
    
    # Fig 1: Nós Expandidos
    plt.figure(figsize=(10, 6))
    df_plot1 = df_raw[(df_raw["success"] == True) & (df_raw["algorithm"] != "Astar")]
    if not df_plot1.empty:
        sns.boxplot(data=df_plot1, x="algorithm", y="nodes_expanded", hue="scenario")
        plt.title("Figura 1: Nós Expandidos por Algoritmo (Buscas Bem-Sucedidas)")
        plt.tight_layout()
        plt.savefig(os.path.join(OUTPUT_DIR, "fig_nodes_expanded.png"), dpi=300)
    plt.close()
    
    # Fig 2: Tempo de Execução Acumulado
    plt.figure(figsize=(12, 6))
    for i, scenario in enumerate(df_raw["scenario"].unique(), 1):
        plt.subplot(1, 2, i)
        df_scen = df_raw[df_raw["scenario"] == scenario].copy()
        
        for algo in df_scen["algorithm"].unique():
            df_algo = df_scen[df_scen["algorithm"] == algo].sort_values("frame")
            df_algo["cum_time"] = df_algo["time_ms"].cumsum()
            plt.plot(df_algo["frame"], df_algo["cum_time"], label=algo)
            
        plt.title(f"{scenario} - Tempo Acumulado")
        plt.xlabel("Frame")
        plt.ylabel("Tempo (ms)")
        plt.legend()
    plt.tight_layout()
    plt.savefig(os.path.join(OUTPUT_DIR, "fig_time_accumulated.png"), dpi=300)
    plt.close()

    # Fig 3: Memória da Fila
    plt.figure(figsize=(12, 6))
    df_plot3 = df_raw[df_raw["algorithm"] != "Astar"]
    if not df_plot3.empty:
        for i, scenario in enumerate(df_plot3["scenario"].unique(), 1):
            plt.subplot(1, 2, i)
            df_scen = df_plot3[df_plot3["scenario"] == scenario]
            sns.scatterplot(data=df_scen, x="frame", y="max_queue_size", hue="algorithm", s=15, alpha=0.7)
            plt.title(f"{scenario} - Memória da Fila (Max Queue)")
    plt.tight_layout()
    plt.savefig(os.path.join(OUTPUT_DIR, "fig_max_queue.png"), dpi=300)
    plt.close()
    
    # Fig 4: Trade-off Tempo vs Nós Expandidos
    plt.figure(figsize=(8, 6))
    df_plot4 = df_summary[df_summary["algorithm"] != "Astar"].dropna(subset=["mean_nodes_expanded"])
    if not df_plot4.empty:
        sns.scatterplot(data=df_plot4, x="mean_nodes_expanded", y="mean_time_ms", hue="algorithm", style="scenario", s=150)
        plt.title("Figura 4: Trade-off Tempo vs Nós Expandidos")
        plt.xlabel("Média de Nós Expandidos")
        plt.ylabel("Tempo Médio de Busca (ms)")
        plt.grid(True)
    plt.tight_layout()
    plt.savefig(os.path.join(OUTPUT_DIR, "fig_tradeoff.png"), dpi=300)
    plt.close()

# ─────────────────────────────────────────────────────────────────────────────
# RUN
# ─────────────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    
    all_records = []
    for sc_name, sc_data in CONFIG["scenarios"].items():
        records = run_scenario(sc_name, sc_data)
        all_records.extend(records)
        
    df_raw = pd.DataFrame(all_records)
    df_raw.to_csv(os.path.join(OUTPUT_DIR, "benchmark_raw.csv"), index=False)
    
    df_summary = generate_summary(df_raw)
    df_summary.to_csv(os.path.join(OUTPUT_DIR, "benchmark_summary.csv"), index=False)
    
    print("Gerando gráficos...")
    generate_plots(df_raw, df_summary)
    
    print("Benchmark concluído com sucesso!")
    print(f"Arquivos salvos em: {OUTPUT_DIR}/")
