"""
Dial's Algorithm (Bucket Dijkstra) — array de buckets indexados por distância discreta.

Substitui a heap por deques indexadas pelo custo acumulado discretizado.
Complexidade: O(E + C·V) onde C = peso máximo discretizado.
Trade-off: vence o heap para V < 1000 e C ≈ 500 (cenário do artigo).

Pesos do pipeline: wbase=1.0, Pmax=500.0  →  w_max ≈ 501.0
Discretização:  SCALE=100  →  C_max = 50100 buckets.
"""

from __future__ import annotations

import math
import time
from collections import defaultdict, deque
from typing import Optional

import networkx as nx

# ── Parâmetros de discretização ───────────────────────────────────────────────
SCALE: int = 100          # 2 casas decimais
W_MAX_FLOAT: float = 501.0
C_MAX: int = int(round(W_MAX_FLOAT * SCALE))   # 50100


def _discretize(w: float) -> int:
    """Converte peso float para índice inteiro de bucket."""
    return int(round(w * SCALE))


def _reconstruct_path(
    pred: dict, source: tuple[int, int], target: tuple[int, int]
) -> list[tuple[int, int]]:
    path = []
    node = target
    while node is not None:
        path.append(node)
        node = pred.get(node)
    path.reverse()
    return path if (path and path[0] == source) else []


def dijkstra_dial(
    G: nx.Graph,
    source: tuple[int, int],
    target: tuple[int, int],
    weight: str = "weight",
) -> tuple[Optional[list[tuple[int, int]]], dict]:
    """
    Dial's Algorithm (Bucket Dijkstra).

    Usa defaultdict(deque) em vez de array fixo para economia de memória.
    Entradas obsoletas (stale) são ignoradas no pop via verificação dist[u] != curr.

    Retorna:
        path  — lista de nós do source ao target (inclusivo), ou None se inalcançável.
        metrics — dict padronizado.
    """
    dist: dict = {source: 0}
    pred: dict = {source: None}
    visited: set = set()

    # buckets indexados por dist discreta acumulada
    buckets: dict = defaultdict(deque)
    buckets[0].append(source)

    curr = 0
    # limite superior: V vezes o peso máximo de uma aresta
    dist_limit = G.number_of_nodes() * C_MAX

    nodes_expanded = 0
    max_queue_size = 0
    _total_in_queue = 1  # contador incremental (evita sum(len) a cada passo)

    t0 = time.perf_counter()

    while curr <= dist_limit:
        # Avançar curr até o próximo bucket não-vazio
        while curr <= dist_limit and not buckets.get(curr):
            curr += 1
        if curr > dist_limit:
            break

        max_queue_size = max(max_queue_size, _total_in_queue)

        u = buckets[curr].popleft()
        _total_in_queue -= 1

        # Entrada obsoleta: dist do nó já foi atualizada para menor valor
        if u in visited:
            continue
        if dist.get(u, math.inf) != curr:
            continue

        visited.add(u)
        nodes_expanded += 1

        if u == target:
            break

        for v, edge_data in G[u].items():
            if v in visited:
                continue
            w_disc = _discretize(edge_data.get(weight, 1.0))
            new_dist = curr + w_disc
            if new_dist < dist.get(v, math.inf):
                dist[v] = new_dist
                pred[v] = u
                buckets[new_dist].append(v)
                _total_in_queue += 1

    t_ms = (time.perf_counter() - t0) * 1000.0

    if target not in visited:
        return None, {
            "nodes_expanded": nodes_expanded,
            "max_queue_size": max_queue_size,
            "time_ms": t_ms,
            "path_cost": None,
            "path_length": None,
            "variant": "dial",
        }

    path = _reconstruct_path(pred, source, target)
    path_cost = sum(
        G[path[i]][path[i + 1]].get(weight, 1.0) for i in range(len(path) - 1)
    )

    return path, {
        "nodes_expanded": nodes_expanded,
        "max_queue_size": max_queue_size,
        "time_ms": t_ms,
        "path_cost": path_cost,
        "path_length": len(path) - 1,
        "variant": "dial",
    }
