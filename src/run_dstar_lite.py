"""
run_dstar_lite.py
=================
Runner frame-a-frame do algoritmo D* Lite para os cenarios BR-06 e CN-01.

Saidas:
    output/dstar_lite/
        dstar_lite_raw.csv
        dstar_lite_summary.csv
        <cenario>_fig_coverage_over_time.png
        <cenario>_fig_nodes_expanded_dist.png
        <cenario>_fig_time_per_search.png
        <cenario>_fig_queue_size.png
        <cenario>_fig_occupancy_heatmap.png
        <cenario>_fig_replan_frequency.png     (NOVO)
        <cenario>_fig_cells_changed.png        (NOVO)

    output/comparativo/
        fig_triplo_comparativo.png             (se CSVs Dijkstra e AntiGravity existirem)
"""

import os
import sys
import time
import random
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

# ---------------------------------------------------------------------------
# Adicionar src ao path (para rodar de qualquer diretorio)
# ---------------------------------------------------------------------------
_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
if _THIS_DIR not in sys.path:
    sys.path.insert(0, _THIS_DIR)

from nav_utils import (
    create_base_grid,
    get_navigable_cells,
    generate_frame_pedestrians,
    compute_occupancy,
)
from dstar_lite_nav import DStarLiteNavigator

# ---------------------------------------------------------------------------
# Constantes
# ---------------------------------------------------------------------------
MAX_STEPS       = 15000
BLOCK_THRESHOLD = 400.0

SCENARIOS = {
    "BR-06": {
        "name"               : "BR-06",
        "arena_m"            : (25, 10),
        "grid"               : (25, 63),
        "navigable_estimate" : 1575,
        "frames"             : 400,
        "lambda_poisson"     : 8.79,
        "start"              : (0, 0),
        "end"                : (24, 62),
    },
    "CN-01": {
        "name"               : "CN-01",
        "arena_m"            : (15, 20),
        "grid"               : (50, 38),
        "navigable_estimate" : 1900,
        "frames"             : 99,
        "lambda_poisson"     : 34.32,
        "start"              : (0, 0),
        "end"                : (49, 37),
    },
}

# ---------------------------------------------------------------------------
# Simulacao principal
# ---------------------------------------------------------------------------
def manhattan_distance(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def run_dstar_lite_simulation(scenario_config, seed=42):
    """
    Executa a simulacao D* Lite frame-a-frame para um cenario.

    Parametros
    ----------
    scenario_config : dict
    seed            : int

    Retorna
    -------
    records       : list[dict]  — uma linha por frame
    last_occ_grid : np.ndarray  — grade de ocupacao do ultimo frame
    """
    random.seed(seed)
    np.random.seed(seed)

    rows, cols   = scenario_config['grid']
    n_frames     = scenario_config['frames']
    lambda_val   = scenario_config['lambda_poisson']
    start        = scenario_config['start']
    scenario_name = scenario_config['name']

    # Grade base (obstaculos estaticos 5%)
    base_grid  = create_base_grid(rows, cols, obstacle_ratio=0.05, seed=seed)
    navigable  = get_navigable_cells(base_grid)
    nav_set    = set(navigable)

    robot             = start
    cleaned           = {robot}
    passos            = 0
    consecutive_fails = 0
    records           = []
    last_occ_grid     = None

    # Navigator D* Lite
    navigator     = DStarLiteNavigator(block_threshold=BLOCK_THRESHOLD)
    initialized   = False
    stable_goal   = None   # goal mantido enquanto nao for alcancado/inacessivel

    print(f"  Iniciando D* Lite para {scenario_name}: {rows}x{cols}, {n_frames} frames")

    for frame in range(1, n_frames + 1):
        # 1. Gerar pedestres (deterministico por seed+frame)
        pedestrians = generate_frame_pedestrians(
            frame_idx=frame,
            lambda_val=lambda_val,
            grid_rows=rows,
            grid_cols=cols,
            seed=seed,
        )

        # 2. Calcular grade de ocupacao
        occ_grid      = compute_occupancy(rows, cols, pedestrians)
        last_occ_grid = occ_grid

        # 3. Checar condicoes de parada
        if passos >= MAX_STEPS or len(cleaned) >= len(navigable):
            break

        not_cleaned = [cell for cell in navigable if cell not in cleaned]
        if not not_cleaned:
            break

        # 4. Escolher goal com estabilidade:
        #    Manter stable_goal enquanto ainda nao foi limpo e esta acessivel.
        #    D* Lite ganha beneficio incremental quando o goal nao muda.
        needs_new_goal = (
            stable_goal is None
            or stable_goal in cleaned          # chegou no goal anterior
            or stable_goal not in nav_set      # goal eh obstaculo estatico
        )
        if needs_new_goal:
            stable_goal = min(not_cleaned,
                              key=lambda c: manhattan_distance(c, robot))

        next_goal = stable_goal

        # 5. D* Lite: inicializar (primeiro frame) ou replanejar incrementalmente
        t0 = time.perf_counter()

        if not initialized:
            metrics   = navigator.initialize(occ_grid, robot, next_goal)
            initialized = True
        else:
            metrics = navigator.step(occ_grid, robot, next_goal)

        time_ms = (time.perf_counter() - t0) * 1000

        # 6. Executar acao
        path = navigator.path

        if metrics['success'] and len(path) >= 2:
            robot = path[1]  # avanca 1 celula
            cleaned.add(robot)
            passos += 1
            action = "ADVANCE"
            consecutive_fails = 0
        else:
            action = "WAIT"
            consecutive_fails += 1

            if consecutive_fails > 5:
                # Goal atual parece inacessivel: escolher novo goal proximo
                nearby = [c for c in not_cleaned
                          if manhattan_distance(c, robot) < 20]
                if nearby:
                    stable_goal = random.choice(nearby)
                else:
                    stable_goal = random.choice(not_cleaned)
                next_goal = stable_goal
                consecutive_fails = 0

        # 7. Registrar metricas
        records.append({
            'scenario'         : scenario_name,
            'frame'            : frame,
            'robot_r'          : robot[0],
            'robot_c'          : robot[1],
            'goal_r'           : next_goal[0],
            'goal_c'           : next_goal[1],
            'nodes_expanded'   : metrics['nodes_expanded'],
            'max_queue_size'   : metrics['max_queue_size'],
            'time_ms'          : time_ms,
            'path_cost'        : metrics['path_cost'],
            'path_length'      : metrics['path_length'],
            'success'          : metrics['success'],
            'action'           : action,
            'pedestrians_count': len(pedestrians),
            'max_occupancy'    : occ_grid.max(),
            'coverage'         : len(cleaned) / len(navigable) * 100,
            # Metricas especificas do D* Lite
            'replan_triggered' : metrics['replan_triggered'],
            'cells_changed'    : metrics['cells_changed'],
            'km_value'         : metrics['km_value'],
        })

    print(f"  {scenario_name}: {len(records)} frames, "
          f"cobertura={records[-1]['coverage']:.1f}% "
          f"(passos={passos})")

    return records, last_occ_grid


# ---------------------------------------------------------------------------
# Geracao de graficos
# ---------------------------------------------------------------------------
def generate_plots(records, last_occ_grid, scenario_name, out_dir):
    """Gera os 7 graficos PNG especificados na SPEC."""
    df = pd.DataFrame(records)
    if df.empty:
        print(f"  Sem dados para {scenario_name}, graficos pulados.")
        return

    # Paleta consistente
    COLOR_MAIN   = '#4A90D9'
    COLOR_SECOND = '#E74C3C'
    COLOR_THIRD  = '#2ECC71'
    COLOR_FOURTH = '#9B59B6'

    def _save(name):
        path = os.path.join(out_dir, f'{scenario_name}_{name}')
        plt.savefig(path, dpi=300, bbox_inches='tight')
        plt.close()
        print(f"    Salvo: {os.path.basename(path)}")

    # 1. Coverage over time
    plt.figure(figsize=(10, 5))
    plt.plot(df['frame'], df['coverage'], color=COLOR_MAIN, linewidth=1.5)
    plt.fill_between(df['frame'], df['coverage'], alpha=0.15, color=COLOR_MAIN)
    plt.title(f'Cobertura ao Longo do Tempo — {scenario_name}', fontsize=13)
    plt.xlabel('Frame'); plt.ylabel('Cobertura (%)')
    plt.grid(True, alpha=0.4); plt.tight_layout()
    _save('fig_coverage_over_time.png')

    # 2. Nodes expanded distribution
    plt.figure(figsize=(10, 5))
    df_s = df[df['success'] == True]
    if not df_s.empty:
        plt.hist(df_s['nodes_expanded'], bins=30, color=COLOR_FOURTH,
                 edgecolor='white', linewidth=0.5)
    plt.title(f'Distribuicao de Nos Expandidos — {scenario_name}', fontsize=13)
    plt.xlabel('Nos Expandidos'); plt.ylabel('Frequencia')
    plt.grid(True, alpha=0.4, axis='y'); plt.tight_layout()
    _save('fig_nodes_expanded_dist.png')

    # 3. Time per search
    plt.figure(figsize=(10, 5))
    plt.plot(df['frame'], df['time_ms'], color=COLOR_THIRD, linewidth=1.0, alpha=0.8)
    plt.axhline(df['time_ms'].mean(), color=COLOR_SECOND, linestyle='--',
                linewidth=1.5, label=f'Media: {df["time_ms"].mean():.3f} ms')
    plt.title(f'Tempo por Frame (ms) — {scenario_name}', fontsize=13)
    plt.xlabel('Frame'); plt.ylabel('Tempo (ms)')
    plt.legend(); plt.grid(True, alpha=0.4); plt.tight_layout()
    _save('fig_time_per_search.png')

    # 4. Queue size
    plt.figure(figsize=(10, 5))
    plt.plot(df['frame'], df['max_queue_size'], color=COLOR_SECOND, linewidth=1.0)
    plt.title(f'Tamanho Maximo da Fila OPEN — {scenario_name}', fontsize=13)
    plt.xlabel('Frame'); plt.ylabel('Max Queue Size')
    plt.grid(True, alpha=0.4); plt.tight_layout()
    _save('fig_queue_size.png')

    # 5. Occupancy heatmap
    plt.figure(figsize=(10, 8))
    im = plt.imshow(last_occ_grid, cmap='hot', interpolation='nearest', aspect='auto')
    plt.colorbar(im, label='Valor de Ocupacao')
    plt.title(f'Heatmap de Ocupacao (Ultimo Frame) — {scenario_name}', fontsize=13)
    plt.xlabel('Colunas'); plt.ylabel('Linhas')
    plt.tight_layout()
    _save('fig_occupancy_heatmap.png')

    # 6. Replan frequency (NOVO)
    plt.figure(figsize=(10, 4))
    replan_col = df['replan_triggered'].astype(int)
    replan_pct = replan_col.rolling(20, min_periods=1).mean() * 100
    plt.bar(df['frame'], replan_col, color=COLOR_MAIN, alpha=0.3, label='Replan (0/1)')
    plt.plot(df['frame'], replan_pct, color=COLOR_SECOND, linewidth=2,
             label=f'Media movel 20f: {replan_col.mean()*100:.1f}%')
    plt.title(f'Frequencia de Replanejamento — {scenario_name}', fontsize=13)
    plt.xlabel('Frame'); plt.ylabel('Replan (%)')
    plt.legend(); plt.grid(True, alpha=0.4, axis='y'); plt.tight_layout()
    _save('fig_replan_frequency.png')

    # 7. Cells changed vs time_ms (NOVO)
    fig, ax1 = plt.subplots(figsize=(10, 5))
    ax2 = ax1.twinx()
    ax1.bar(df['frame'], df['cells_changed'], color=COLOR_MAIN, alpha=0.4, label='Celulas Alteradas')
    ax2.plot(df['frame'], df['time_ms'], color=COLOR_SECOND, linewidth=1.2, label='Tempo (ms)')
    ax1.set_xlabel('Frame')
    ax1.set_ylabel('Celulas Alteradas', color=COLOR_MAIN)
    ax2.set_ylabel('Tempo (ms)', color=COLOR_SECOND)
    ax1.tick_params(axis='y', labelcolor=COLOR_MAIN)
    ax2.tick_params(axis='y', labelcolor=COLOR_SECOND)
    lines1, labels1 = ax1.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    ax1.legend(lines1 + lines2, labels1 + labels2, loc='upper right')
    plt.title(f'Celulas Alteradas vs Tempo — {scenario_name}', fontsize=13)
    plt.tight_layout()
    _save('fig_cells_changed.png')


# ---------------------------------------------------------------------------
# Grafico comparativo triplo
# ---------------------------------------------------------------------------
def generate_triplo_comparativo(out_dir_cmp, dstar_raw_path,
                                 dijk_raw_path, ag_raw_path):
    """
    Gera fig_triplo_comparativo.png comparando Dijk-Std, AntiGravity e D* Lite.
    So e gerado se todos os 3 CSVs existirem.
    """
    if not all(os.path.exists(p) for p in [dstar_raw_path, dijk_raw_path, ag_raw_path]):
        missing = [p for p in [dijk_raw_path, ag_raw_path, dstar_raw_path]
                   if not os.path.exists(p)]
        print(f"  Comparativo triplo pulado — CSVs ausentes: {[os.path.basename(p) for p in missing]}")
        return

    df_d  = pd.read_csv(dijk_raw_path)
    df_ag = pd.read_csv(ag_raw_path)
    df_ds = pd.read_csv(dstar_raw_path)

    scenarios = list(SCENARIOS.keys())

    fig = plt.figure(figsize=(16, 10))
    fig.suptitle('Comparativo Triplo: Dijkstra-Std | AntiGravity | D* Lite',
                 fontsize=15, fontweight='bold', y=1.01)

    # 2 cenarios x 3 metricas = 6 subplots
    gs = gridspec.GridSpec(3, len(scenarios), figure=fig, hspace=0.55, wspace=0.35)

    COLORS = {
        'Dijkstra-Std': '#E74C3C',
        'AntiGravity' : '#3498DB',
        'D* Lite'     : '#2ECC71',
    }

    metric_configs = [
        ('coverage',        'Cobertura (%)',        'Cobertura ao Longo do Tempo'),
        ('time_ms',         'Tempo (ms)',            'Tempo por Frame (ms)'),
        ('nodes_expanded',  'Nos Expandidos',        'Nos Expandidos por Frame'),
    ]

    for col_idx, scen in enumerate(scenarios):
        for row_idx, (metric, ylabel, title) in enumerate(metric_configs):
            ax = fig.add_subplot(gs[row_idx, col_idx])

            for label, df in [('Dijkstra-Std', df_d),
                               ('AntiGravity',  df_ag),
                               ('D* Lite',      df_ds)]:
                sub = df[df['scenario'] == scen]
                if sub.empty or metric not in sub.columns:
                    continue
                ax.plot(sub['frame'], sub[metric],
                        label=label, color=COLORS[label],
                        linewidth=1.4, alpha=0.85)

            ax.set_title(f'{title}\n{scen}', fontsize=9)
            ax.set_xlabel('Frame', fontsize=8)
            ax.set_ylabel(ylabel, fontsize=8)
            ax.tick_params(labelsize=7)
            ax.grid(True, alpha=0.3)
            if row_idx == 0 and col_idx == 0:
                ax.legend(fontsize=7, loc='upper left')

    out_path = os.path.join(out_dir_cmp, 'fig_triplo_comparativo.png')
    plt.savefig(out_path, dpi=300, bbox_inches='tight')
    plt.close()
    print(f"  Comparativo triplo salvo: {out_path}")


# ---------------------------------------------------------------------------
# Calculo de summary
# ---------------------------------------------------------------------------
def build_summary(records, config):
    df = pd.DataFrame(records)
    if df.empty:
        return {}

    total_steps  = df[df['action'] == 'ADVANCE'].shape[0]
    coverage_pct = df['coverage'].iloc[-1]
    n_frames     = config['frames']
    df_succ      = df[df['success'] == True]

    # Porcentagem de frames com replanejamento
    replan_pct = df['replan_triggered'].mean() * 100

    return {
        'scenario'              : config['name'],
        'n_frames'              : n_frames,
        'total_steps'           : total_steps,
        'coverage_percent'      : coverage_pct,
        'coverage_per_step'     : (coverage_pct / total_steps) if total_steps > 0 else 0,
        'advance_rate'          : total_steps / n_frames * 100,
        'avg_nodes_expanded'    : df_succ['nodes_expanded'].mean() if not df_succ.empty else 0,
        'max_nodes_expanded'    : df['nodes_expanded'].max(),
        'avg_queue_size'        : df_succ['max_queue_size'].mean() if not df_succ.empty else 0,
        'total_time_ms'         : df['time_ms'].sum(),
        'avg_time_ms'           : df['time_ms'].mean(),
        'max_time_ms'           : df['time_ms'].max(),
        'success_rate'          : df_succ.shape[0] / len(df) * 100,
        'wait_count'            : df[df['action'] == 'WAIT'].shape[0],
        'replan_pct_frames'     : replan_pct,
        'avg_cells_changed'     : df['cells_changed'].mean(),
        'max_km_value'          : df['km_value'].max(),
        'max_consecutive_waits' : (df['action'] == 'WAIT').astype(int)
                                    .groupby((df['action'] != 'WAIT').cumsum())
                                    .sum().max(),
    }


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    # Detectar diretorio de trabalho (rodar de src/ ou da raiz do projeto)
    if os.path.basename(os.getcwd()) == 'src':
        base_dir = os.path.join('..', 'output')
    else:
        base_dir = 'output'

    out_dir     = os.path.join(base_dir, 'dstar_lite')
    out_dir_cmp = os.path.join(base_dir, 'comparativo')
    os.makedirs(out_dir,     exist_ok=True)
    os.makedirs(out_dir_cmp, exist_ok=True)

    all_records = []
    summaries   = []

    for name, config in SCENARIOS.items():
        print(f"\nExecutando D* Lite — {name}...")
        records, last_occ_grid = run_dstar_lite_simulation(config)
        all_records.extend(records)

        print(f"  Gerando graficos para {name}...")
        generate_plots(records, last_occ_grid, name, out_dir)

        summary = build_summary(records, config)
        if summary:
            summaries.append(summary)

    # Salvar CSVs
    raw_path = os.path.join(out_dir, 'dstar_lite_raw.csv')
    sum_path = os.path.join(out_dir, 'dstar_lite_summary.csv')
    pd.DataFrame(all_records).to_csv(raw_path, index=False)
    pd.DataFrame(summaries).to_csv(sum_path,  index=False)
    print(f"\nCSVs salvos:\n  {raw_path}\n  {sum_path}")

    # Grafico comparativo triplo (se CSVs dos outros algoritmos existirem)
    print("\nTentando gerar comparativo triplo...")
    generate_triplo_comparativo(
        out_dir_cmp,
        dstar_raw_path=raw_path,
        dijk_raw_path =os.path.join(base_dir, 'dijkstra',   'dijkstra_raw.csv'),
        ag_raw_path   =os.path.join(base_dir, 'antigravity', 'antigravity_raw.csv'),
    )

    # Relatorio de aceitacao
    print("\n" + "=" * 60)
    print("CRITERIOS DE ACEITACAO")
    print("=" * 60)
    df_all = pd.DataFrame(all_records)
    for name, config in SCENARIOS.items():
        sub = df_all[df_all['scenario'] == name]
        if sub.empty:
            continue
        cov   = sub['coverage'].iloc[-1]
        t_avg = sub['time_ms'].mean()
        r_pct = sub['replan_triggered'].mean() * 100
        n_lin = len(sub)
        print(f"\n  {name}:")
        print(f"    Linhas CSV     : {n_lin} (esperado {config['frames']}) "
              f"{'OK' if n_lin == config['frames'] else 'FALHOU'}")
        if name == 'BR-06':
            print(f"    Cobertura      : {cov:.1f}%  alvo 20-25%  "
                  f"{'OK' if 20 <= cov <= 25 else 'FORA DO ALVO'}")
            print(f"    Tempo medio    : {t_avg:.3f} ms  alvo <1ms  "
                  f"{'OK' if t_avg < 1 else 'FORA DO ALVO'}")
            print(f"    Replanejamentos: {r_pct:.1f}%  alvo <50%   "
                  f"{'OK' if r_pct < 50 else 'FORA DO ALVO'}")
        else:
            print(f"    Cobertura      : {cov:.1f}%  alvo 3-5%   "
                  f"{'OK' if 3 <= cov <= 5 else 'FORA DO ALVO'}")
            print(f"    Tempo medio    : {t_avg:.3f} ms  alvo <2ms  "
                  f"{'OK' if t_avg < 2 else 'FORA DO ALVO'}")
            print(f"    Replanejamentos: {r_pct:.1f}%  alvo <70%   "
                  f"{'OK' if r_pct < 70 else 'FORA DO ALVO'}")

    print("\nSimulacao D* Lite concluida com sucesso.")


if __name__ == '__main__':
    main()
