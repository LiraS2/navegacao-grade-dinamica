"""
test_minimal.py — Critério 1 do Definition of Done.

Testa as 3 variantes em grafo 3×3 com pesos uniformes 1.0.
Caminho esperado de (0,0) a (2,2): 4 arestas / 5 nós.
"""

import pytest
import networkx as nx
from antigravity import dijkstra_path


def make_grid(rows: int, cols: int, weight: float = 1.0) -> nx.Graph:
    """Grade 4-conectada com pesos uniformes."""
    G = nx.grid_2d_graph(rows, cols)
    for u, v in G.edges():
        G[u][v]["weight"] = weight
    return G


VARIANTS = ["standard", "bidirectional", "dial"]


@pytest.mark.parametrize("variant", VARIANTS)
def test_path_exists_3x3(variant):
    G = make_grid(3, 3)
    source, target = (0, 0), (2, 2)
    path, metrics = dijkstra_path(G, source, target, variant=variant)

    assert path is not None, f"[{variant}] Nenhum caminho encontrado"
    assert path[0] == source, f"[{variant}] Caminho não inicia em source"
    assert path[-1] == target, f"[{variant}] Caminho não termina em target"
    assert metrics["path_length"] == 4, (
        f"[{variant}] Comprimento esperado 4, obtido {metrics['path_length']}"
    )
    assert metrics["path_cost"] == pytest.approx(4.0), (
        f"[{variant}] Custo esperado 4.0, obtido {metrics['path_cost']}"
    )


@pytest.mark.parametrize("variant", VARIANTS)
def test_metrics_keys_present(variant):
    G = make_grid(3, 3)
    _, metrics = dijkstra_path(G, (0, 0), (2, 2), variant=variant)

    required_keys = {
        "nodes_expanded", "max_queue_size", "time_ms",
        "path_cost", "path_length", "variant",
    }
    assert required_keys.issubset(metrics.keys()), (
        f"[{variant}] Chaves faltando: {required_keys - metrics.keys()}"
    )
    assert metrics["nodes_expanded"] > 0
    assert metrics["time_ms"] >= 0.0
    assert metrics["variant"] == variant


@pytest.mark.parametrize("variant", VARIANTS)
def test_source_equals_target(variant):
    G = make_grid(3, 3)
    path, metrics = dijkstra_path(G, (1, 1), (1, 1), variant=variant)

    assert path is not None
    assert path == [(1, 1)]
    assert metrics["path_length"] == 0
    assert metrics["path_cost"] == pytest.approx(0.0)


def test_invalid_variant_raises():
    G = make_grid(2, 2)
    with pytest.raises(ValueError, match="Variante desconhecida"):
        dijkstra_path(G, (0, 0), (1, 1), variant="inexistente")


@pytest.mark.parametrize("variant", VARIANTS)
def test_path_is_valid_sequence(variant):
    """Cada nó consecutivo deve ser vizinho no grafo."""
    G = make_grid(5, 5)
    path, _ = dijkstra_path(G, (0, 0), (4, 4), variant=variant)
    assert path is not None
    for i in range(len(path) - 1):
        assert G.has_edge(path[i], path[i + 1]), (
            f"[{variant}] Aresta {path[i]}→{path[i+1]} não existe no grafo"
        )
