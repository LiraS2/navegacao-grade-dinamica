"""
dijkstra_nav.py
===============
Algoritmo Dijkstra padrao para navegacao em grade dinamica.
Funcoes necessarias para o run_simulation.py:
    - build_frame_graph: constroi grafo dinamico com custos baseados em ocupacao
    - dijkstra_std: busca Dijkstra de fonte unica
"""

import heapq
import time

BLOCK_THRESHOLD = 400.0


def build_frame_graph(base_graph, occ_grid, block_threshold=BLOCK_THRESHOLD):
    """
    Constroi um grafo dinamico a partir do grafo base,
    removendo arestas para celulas bloqueadas (ocupacao > threshold).

    Parametros
    ----------
    base_graph : dict
        {(r,c): [((nr,nc), peso), ...]}
    occ_grid : np.ndarray
        Grade de ocupacao (rows x cols).
    block_threshold : float
        Ocupacao acima deste valor bloqueia a celula.

    Retorna
    -------
    frame_graph : dict
        Mesmo formato que base_graph, sem arestas para celulas bloqueadas.
    """
    frame_graph = {}
    for node, neighbors in base_graph.items():
        r, c = node
        # Se o proprio no esta bloqueado, ainda aparece no grafo
        # mas pode nao ser alcancavel
        if occ_grid[r, c] > block_threshold:
            frame_graph[node] = []
            continue
        valid_neighbors = []
        for (nr, nc), weight in neighbors:
            if occ_grid[nr, nc] <= block_threshold:
                valid_neighbors.append(((nr, nc), weight))
        frame_graph[node] = valid_neighbors
    return frame_graph


def dijkstra_std(graph, start, goal):
    """
    Dijkstra padrao de fonte unica.

    Parametros
    ----------
    graph : dict
        {node: [(neighbor, weight), ...]}
    start : tuple (r, c)
    goal  : tuple (r, c)

    Retorna
    -------
    dict com:
        success        : bool
        path           : list[(r,c)]
        path_cost      : float
        path_length    : int
        nodes_expanded : int
        max_queue_size : int
        time_ms        : float
    """
    t0 = time.perf_counter()

    dist = {start: 0.0}
    prev = {start: None}
    heap = [(0.0, start)]
    visited = set()
    nodes_expanded = 0
    max_queue_size = 1

    while heap:
        max_queue_size = max(max_queue_size, len(heap))
        cost, u = heapq.heappop(heap)

        if u in visited:
            continue
        visited.add(u)
        nodes_expanded += 1

        if u == goal:
            break

        for (v, w) in graph.get(u, []):
            new_cost = cost + w
            if v not in dist or new_cost < dist[v]:
                dist[v] = new_cost
                prev[v] = u
                heapq.heappush(heap, (new_cost, v))

    time_ms = (time.perf_counter() - t0) * 1000

    # Reconstruir caminho
    if goal in visited:
        path = []
        node = goal
        while node is not None:
            path.append(node)
            node = prev[node]
        path.reverse()
        return {
            'success': True,
            'path': path,
            'path_cost': dist[goal],
            'path_length': len(path) - 1,
            'nodes_expanded': nodes_expanded,
            'max_queue_size': max_queue_size,
            'time_ms': time_ms,
        }
    else:
        return {
            'success': False,
            'path': [],
            'path_cost': -1,
            'path_length': 0,
            'nodes_expanded': nodes_expanded,
            'max_queue_size': max_queue_size,
            'time_ms': time_ms,
        }
