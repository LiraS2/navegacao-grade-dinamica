"""
test_integration.py — Critérios 3, 4 e 5 do Definition of Done.

Crit. 3: Pesos altos — grade 5×5, aresta central peso 500.0. Dial sem overflow.
Crit. 4: Shadow test — 10 frames com Dijkstra Standard vs A* Manhattan: caminhos idênticos.
Crit. 5: Métricas exportáveis para DataFrame com coluna "algorithm".
"""

from __future__ import annotations

import pytest
import networkx as nx

try:
    import pandas as pd
    HAS_PANDAS = True
except ImportError:
    HAS_PANDAS = False

from antigravity import dijkstra_path

VARIANTS = ["standard", "bidirectional", "dial"]


# ─────────────────────────────────────────────────────────────────────────────
# Critério 3 — Pesos altos (aresta central = 500.0)
# ─────────────────────────────────────────────────────────────────────────────

def make_grid_high_weight() -> nx.Graph:
    """Grade 5×5, peso padrão 1.0, aresta central (2,2)-(2,3) com peso 500.0."""
    G = nx.grid_2d_graph(5, 5)
    for u, v in G.edges():
        G[u][v]["weight"] = 1.0
    # Aresta central com peso alto
    if G.has_edge((2, 2), (2, 3)):
        G[(2, 2)][(2, 3)]["weight"] = 500.0
    if G.has_edge((2, 3), (2, 2)):
        G[(2, 3)][(2, 2)]["weight"] = 500.0
    return G


@pytest.mark.parametrize("variant", VARIANTS)
def test_high_weight_finds_path(variant):
    G = make_grid_high_weight()
    path, metrics = dijkstra_path(G, (0, 0), (4, 4), variant=variant)

    assert path is not None, f"[{variant}] Nenhum caminho encontrado com pesos altos"
    assert metrics["path_cost"] is not None
    # O caminho mínimo deve contornar a aresta cara (custo < 500)
    assert metrics["path_cost"] < 500.0, (
        f"[{variant}] Custo {metrics['path_cost']} ≥ 500 (não contornou aresta cara)"
    )


@pytest.mark.parametrize("variant", VARIANTS)
def test_high_weight_no_overflow(variant):
    """Dial: verificar que não há OverflowError com pesos grandes."""
    G = make_grid_high_weight()
    try:
        path, metrics = dijkstra_path(G, (0, 0), (4, 4), variant=variant)
    except OverflowError as e:
        pytest.fail(f"[{variant}] OverflowError com peso alto: {e}")
    assert path is not None


# ─────────────────────────────────────────────────────────────────────────────
# Critério 4 — Shadow: Dijkstra Standard vs A* (caminho com custo igual)
# ─────────────────────────────────────────────────────────────────────────────

def manhattan(u, v):
    return abs(u[0] - v[0]) + abs(u[1] - v[1])


def make_dynamic_grid(frame: int) -> tuple[nx.Graph, tuple, tuple]:
    """
    Simula um frame do pipeline: grade 10×10 com peso base 1.0
    e um bloqueio variável por frame (aresta com peso alto).
    """
    G = nx.grid_2d_graph(10, 10)
    for u, v in G.edges():
        G[u][v]["weight"] = 1.0

    # Bloqueio dinâmico: aresta diferente a cada frame
    r, c = frame % 8 + 1, frame % 8 + 1
    if G.has_edge((r, c), (r, c + 1)):
        G[(r, c)][(r, c + 1)]["weight"] = 999.0

    source = (0, 0)
    target = (9, 9)
    return G, source, target


def test_shadow_standard_vs_astar_10_frames():
    """
    Para 10 frames, Dijkstra Standard e A* devem encontrar caminhos
    com o mesmo custo total (ambos são ótimos).
    """
    for frame in range(10):
        G, source, target = make_dynamic_grid(frame)

        path_dijkstra, metrics_d = dijkstra_path(
            G, source, target, variant="standard", weight="weight"
        )
        path_astar = nx.astar_path(
            G, source, target, heuristic=manhattan, weight="weight"
        )

        assert path_dijkstra is not None, f"Frame {frame}: Dijkstra não encontrou caminho"

        # Custo do A*
        cost_astar = sum(
            G[path_astar[i]][path_astar[i + 1]]["weight"]
            for i in range(len(path_astar) - 1)
        )

        assert metrics_d["path_cost"] == pytest.approx(cost_astar, rel=1e-6), (
            f"Frame {frame}: custo Dijkstra {metrics_d['path_cost']} "
            f"≠ custo A* {cost_astar}"
        )


# ─────────────────────────────────────────────────────────────────────────────
# Critério 5 — Exportação de métricas para DataFrame
# ─────────────────────────────────────────────────────────────────────────────

@pytest.mark.skipif(not HAS_PANDAS, reason="pandas não instalado")
def test_metrics_dataframe_export():
    """
    Simula acumulação de métricas de múltiplos algoritmos e frames
    e verifica que o DataFrame tem as colunas esperadas.
    """
    G, source, target = make_dynamic_grid(0)

    records = []

    # A*
    path_a = nx.astar_path(G, source, target, heuristic=manhattan, weight="weight")
    cost_a = sum(G[path_a[i]][path_a[i + 1]]["weight"] for i in range(len(path_a) - 1))
    records.append({
        "frame": 0,
        "algorithm": "A*",
        "nodes_expanded": None,   # A* do NetworkX não expõe esse dado
        "max_queue_size": None,
        "time_ms": None,
        "path_cost": cost_a,
        "path_length": len(path_a) - 1,
    })

    # Dijkstra variantes
    algo_labels = {
        "standard": "Dijkstra-Std",
        "bidirectional": "Dijkstra-Bi",
        "dial": "Dijkstra-Dial",
    }
    for variant, label in algo_labels.items():
        _, m = dijkstra_path(G, source, target, variant=variant)
        records.append({
            "frame": 0,
            "algorithm": label,
            "nodes_expanded": m["nodes_expanded"],
            "max_queue_size": m["max_queue_size"],
            "time_ms": m["time_ms"],
            "path_cost": m["path_cost"],
            "path_length": m["path_length"],
        })

    df = pd.DataFrame(records)

    expected_cols = {
        "frame", "algorithm", "nodes_expanded",
        "max_queue_size", "time_ms", "path_cost", "path_length",
    }
    assert expected_cols.issubset(df.columns), (
        f"Colunas faltando: {expected_cols - set(df.columns)}"
    )
    assert set(df["algorithm"]) == {"A*", "Dijkstra-Std", "Dijkstra-Bi", "Dijkstra-Dial"}
    assert len(df) == 4
