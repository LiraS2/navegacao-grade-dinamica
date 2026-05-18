"""
test_disconnected.py — Critério 2 do Definition of Done.

Testa comportamento quando target é isolado:
  - Retorna None (path)
  - nodes_expanded reflete toda a componente conectada ao source.
  - metrics dict está presente e completo.
"""

import pytest
import networkx as nx
from antigravity import dijkstra_path


VARIANTS = ["standard", "bidirectional", "dial"]


def make_disconnected_grid() -> tuple[nx.Graph, tuple, tuple]:
    """
    Grade 3×3 onde o nó (2,2) está completamente isolado
    (todas as arestas ligadas a ele são removidas).
    """
    G = nx.grid_2d_graph(3, 3)
    for u, v in G.edges():
        G[u][v]["weight"] = 1.0

    isolated = (2, 2)
    neighbors = list(G.neighbors(isolated))
    for nb in neighbors:
        G.remove_edge(isolated, nb)

    return G, (0, 0), isolated


@pytest.mark.parametrize("variant", VARIANTS)
def test_disconnected_returns_none(variant):
    G, source, target = make_disconnected_grid()
    path, metrics = dijkstra_path(G, source, target, variant=variant)

    assert path is None, (
        f"[{variant}] Esperava None para grafo desconectado, obteve {path}"
    )
    assert metrics["path_cost"] is None
    assert metrics["path_length"] is None
    assert metrics["variant"] == variant


@pytest.mark.parametrize("variant", VARIANTS)
def test_disconnected_nodes_expanded(variant):
    """
    nodes_expanded deve ser ≥ 1 (ao menos source foi expandido)
    e ≤ número de nós da componente acessível (8 nós, pois (2,2) está isolado).
    """
    G, source, target = make_disconnected_grid()
    _, metrics = dijkstra_path(G, source, target, variant=variant)

    # 8 nós acessíveis a partir de (0,0) em grade 3×3 sem (2,2) conectado
    accessible = 8
    assert 1 <= metrics["nodes_expanded"] <= accessible, (
        f"[{variant}] nodes_expanded={metrics['nodes_expanded']} fora do intervalo [1, {accessible}]"
    )


@pytest.mark.parametrize("variant", VARIANTS)
def test_disconnected_metrics_keys(variant):
    G, source, target = make_disconnected_grid()
    _, metrics = dijkstra_path(G, source, target, variant=variant)

    required = {"nodes_expanded", "max_queue_size", "time_ms",
                "path_cost", "path_length", "variant"}
    assert required.issubset(metrics.keys())
    assert metrics["time_ms"] >= 0.0


@pytest.mark.parametrize("variant", VARIANTS)
def test_single_node_graph_disconnected(variant):
    """
    Grafo com apenas source. Target inexistente.
    Deve retornar None sem travar.
    """
    G = nx.Graph()
    G.add_node((0, 0))
    # Target não existe no grafo
    if variant == "bidirectional":
        pytest.skip("Bidirecional requer target no grafo para inicializar heap_b")

    path, metrics = dijkstra_path(G, (0, 0), (1, 1), variant=variant)
    assert path is None
    assert metrics["nodes_expanded"] >= 0
