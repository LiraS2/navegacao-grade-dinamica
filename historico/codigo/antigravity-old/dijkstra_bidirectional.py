"""
Dijkstra Bidirecional — duas fronteiras simultâneas (forward + backward).

Reduz a área de expansão de ~n² para ~2·(n/2)² em grades 4-conectadas.
Complexidade: O(E + V log V) teórica, ~½ nós expandidos na prática.
Memória: ~2× as estruturas do Dijkstra standard.
"""

from __future__ import annotations

import heapq
import math
import time
from typing import Optional

import networkx as nx


def _reconstruct_bidir(
    pred_f: dict,
    pred_b: dict,
    source: tuple[int, int],
    target: tuple[int, int],
    meeting: tuple[int, int],
) -> list[tuple[int, int]]:
    """Monta o caminho juntando as duas metades no nó de encontro."""
    # Metade forward: source → meeting
    fwd: list = []
    node = meeting
    while node is not None:
        fwd.append(node)
        node = pred_f.get(node)
    fwd.reverse()

    # Metade backward: meeting → target (seguindo pred_b)
    bwd: list = []
    node = pred_b.get(meeting)  # meeting já está em fwd
    while node is not None:
        bwd.append(node)
        node = pred_b.get(node)

    return fwd + bwd


def dijkstra_bidirectional(
    G: nx.Graph,
    source: tuple[int, int],
    target: tuple[int, int],
    weight: str = "weight",
) -> tuple[Optional[list[tuple[int, int]]], dict]:
    """
    Dijkstra bidirecional.

    Retorna:
        path  — lista de nós do source ao target (inclusivo), ou None se inalcançável.
        metrics — dict padronizado.
    """
    # Edge case trivial
    if source == target:
        return [source], {
            "nodes_expanded": 0,
            "max_queue_size": 0,
            "time_ms": 0.0,
            "path_cost": 0.0,
            "path_length": 0,
            "variant": "bidirectional",
        }

    dist_f: dict = {source: 0.0}
    dist_b: dict = {target: 0.0}
    pred_f: dict = {source: None}
    pred_b: dict = {target: None}
    visited_f: set = set()
    visited_b: set = set()
    heap_f: list = [(0.0, source)]
    heap_b: list = [(0.0, target)]

    best_len = math.inf
    meeting: Optional[tuple[int, int]] = None

    nodes_expanded = 0
    max_queue_size = 0

    t0 = time.perf_counter()

    while heap_f and heap_b:
        max_queue_size = max(max_queue_size, len(heap_f) + len(heap_b))

        # Escolher a frente com menor topo
        use_forward = heap_f[0][0] <= heap_b[0][0]

        if use_forward:
            d, u = heapq.heappop(heap_f)
            if u in visited_f:
                continue
            visited_f.add(u)
            nodes_expanded += 1

            if u in visited_b:
                candidate = dist_f.get(u, math.inf) + dist_b.get(u, math.inf)
                if candidate < best_len:
                    best_len = candidate
                    meeting = u

            for v, edge_data in G[u].items():
                if v in visited_f:
                    continue
                w = edge_data.get(weight, 1.0)
                nd = d + w
                if nd < dist_f.get(v, math.inf):
                    dist_f[v] = nd
                    pred_f[v] = u
                    heapq.heappush(heap_f, (nd, v))
        else:
            d, u = heapq.heappop(heap_b)
            if u in visited_b:
                continue
            visited_b.add(u)
            nodes_expanded += 1

            if u in visited_f:
                candidate = dist_f.get(u, math.inf) + dist_b.get(u, math.inf)
                if candidate < best_len:
                    best_len = candidate
                    meeting = u

            for v, edge_data in G[u].items():
                if v in visited_b:
                    continue
                w = edge_data.get(weight, 1.0)
                nd = d + w
                if nd < dist_b.get(v, math.inf):
                    dist_b[v] = nd
                    pred_b[v] = u
                    heapq.heappush(heap_b, (nd, v))

        # Parada antecipada: se a soma dos menores topos já supera best_len
        if heap_f and heap_b:
            if heap_f[0][0] + heap_b[0][0] >= best_len:
                break

    t_ms = (time.perf_counter() - t0) * 1000.0

    if meeting is None:
        return None, {
            "nodes_expanded": nodes_expanded,
            "max_queue_size": max_queue_size,
            "time_ms": t_ms,
            "path_cost": None,
            "path_length": None,
            "variant": "bidirectional",
        }

    path = _reconstruct_bidir(pred_f, pred_b, source, target, meeting)
    path_cost = sum(
        G[path[i]][path[i + 1]].get(weight, 1.0) for i in range(len(path) - 1)
    )

    return path, {
        "nodes_expanded": nodes_expanded,
        "max_queue_size": max_queue_size,
        "time_ms": t_ms,
        "path_cost": path_cost,
        "path_length": len(path) - 1,
        "variant": "bidirectional",
    }
