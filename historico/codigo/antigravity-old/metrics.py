"""
metrics.py — Wrapper unificado de instrumentação.

Ponto de entrada único para o pipeline: substitui nx.astar_path com drop-in.

Uso no loop de simulação:
    # Antes
    path = nx.astar_path(Groute, robot, next_goal, heuristic=manhattan, weight="weight")

    # Depois
    path, metrics = dijkstra_path(Groute, robot, next_goal, variant="standard", weight="weight")
"""

from __future__ import annotations

from typing import Literal, Optional

import networkx as nx

from .dijkstra_bidirectional import dijkstra_bidirectional
from .dijkstra_dial import dijkstra_dial
from .dijkstra_standard import dijkstra_standard

Variant = Literal["standard", "bidirectional", "dial"]


def dijkstra_path(
    G: nx.Graph,
    source: tuple[int, int],
    target: tuple[int, int],
    variant: Variant = "standard",
    weight: str = "weight",
) -> tuple[Optional[list[tuple[int, int]]], dict]:
    """
    Ponto de entrada unificado para as 3 variantes de Dijkstra.

    Parâmetros
    ----------
    G        : nx.Graph — GrRoute ou Gstatic do pipeline.
    source   : tuple (r, c) — nó de origem.
    target   : tuple (r, c) — nó de destino.
    variant  : "standard" | "bidirectional" | "dial"
    weight   : nome do atributo de peso nas arestas (default "weight").

    Retorna
    -------
    path     : lista de nós source → target (inclusiva), ou None se inalcançável.
    metrics  : {
        "nodes_expanded": int,
        "max_queue_size": int,
        "time_ms": float,
        "path_cost": float | None,
        "path_length": int | None,
        "variant": str,
    }

    Levanta
    -------
    ValueError se variant não for reconhecida.
    """
    _DISPATCH = {
        "standard": dijkstra_standard,
        "bidirectional": dijkstra_bidirectional,
        "dial": dijkstra_dial,
    }

    fn = _DISPATCH.get(variant)
    if fn is None:
        raise ValueError(
            f"Variante desconhecida: {variant!r}. "
            f"Use um de: {list(_DISPATCH.keys())}"
        )

    return fn(G, source, target, weight=weight)
