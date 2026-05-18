"""
Dijkstra Standard — heap binária (heapq).

Complexidade: O(E + V log V)
Memória: O(V) dist + O(V) heap no pior caso.
"""

from __future__ import annotations

import heapq
import math
import time
from typing import Optional

import networkx as nx


def _reconstruct_path(
    pred: dict, source: tuple[int, int], target: tuple[int, int]
) -> list[tuple[int, int]]:
    path = []
    node = target
    while node is not None:
        path.append(node)
        node = pred.get(node)
    path.reverse()
    if path[0] != source:
        return []  # desconectado
    return path


def dijkstra_standard(
    G: nx.Graph,
    source: tuple[int, int],
    target: tuple[int, int],
    weight: str = "weight",
) -> tuple[Optional[list[tuple[int, int]]], dict]:
    """
    Dijkstra com heap binária.

    Retorna:
        path  — lista de nós do source ao target (inclusivo), ou None se inalcançável.
        metrics — dict com nodes_expanded, max_queue_size, time_ms, path_cost, path_length.
    """
    dist: dict = {source: 0.0}
    pred: dict = {source: None}
    visited: set = set()
    heap: list = [(0.0, source)]

    nodes_expanded = 0
    max_queue_size = 0

    # ── início da janela de tempo (apenas busca) ──────────────────────────────
    t0 = time.perf_counter()

    while heap:
        max_queue_size = max(max_queue_size, len(heap))
        d, u = heapq.heappop(heap)

        if u in visited:
            continue
        visited.add(u)
        nodes_expanded += 1

        if u == target:
            break

        for v, edge_data in G[u].items():
            if v in visited:
                continue
            w = edge_data.get(weight, 1.0)
            new_dist = d + w
            if new_dist < dist.get(v, math.inf):
                dist[v] = new_dist
                pred[v] = u
                heapq.heappush(heap, (new_dist, v))

    t_ms = (time.perf_counter() - t0) * 1000.0
    # ── fim da janela de tempo ────────────────────────────────────────────────

    if target not in visited:
        return None, {
            "nodes_expanded": nodes_expanded,
            "max_queue_size": max_queue_size,
            "time_ms": t_ms,
            "path_cost": None,
            "path_length": None,
            "variant": "standard",
        }

    # Reconstrução (fora da janela de tempo)
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
        "variant": "standard",
    }
