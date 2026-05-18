import heapq
import time
import numpy as np

def compute_edge_weight(occ_grid, u, v, base_weight=1.0, block_threshold=400.0):
    occ_u = occ_grid[u[0], u[1]]
    occ_v = occ_grid[v[0], v[1]]
    weight = base_weight + max(occ_u, occ_v)
    if weight > block_threshold:
        return None  # Aresta removida
    return weight

def build_frame_graph(base_graph, occ_grid, block_threshold=400.0):
    frame_graph = {}
    for u, neighbors in base_graph.items():
        frame_graph[u] = []
        for v, _ in neighbors:
            w = compute_edge_weight(occ_grid, u, v, block_threshold=block_threshold)
            if w is not None:
                frame_graph[u].append((v, w))
    return frame_graph

def dijkstra_std(graph, source, target):
    dist = {v: float('inf') for v in graph}
    pred = {v: None for v in graph}
    if source in graph:
        dist[source] = 0
    else:
        return {
            'path': None, 'path_cost': 0.0, 'path_length': -1,
            'nodes_expanded': 0, 'max_queue_size': 0, 'time_ms': 0.0, 'success': False
        }
        
    heap = [(0, source)]
    visited = set()
    nodes_expanded = 0
    max_queue = 0

    t0 = time.perf_counter()

    while heap:
        max_queue = max(max_queue, len(heap))
        d, u = heapq.heappop(heap)

        if u in visited:
            continue
        visited.add(u)
        nodes_expanded += 1

        if u == target:
            break

        for v, w in graph.get(u, []):
            if v in visited:
                continue
            new_dist = dist[u] + w
            if new_dist < dist[v]:
                dist[v] = new_dist
                pred[v] = u
                heapq.heappush(heap, (new_dist, v))

    time_ms = (time.perf_counter() - t0) * 1000

    # Reconstruir path
    if target not in dist or dist[target] == float('inf'):
        path = None
        path_cost = 0.0
        path_length = -1
    else:
        path = []
        node = target
        while node is not None:
            path.append(node)
            node = pred[node]
        path.reverse()
        path_cost = dist[target]
        path_length = len(path) - 1

    return {
        'path': path,
        'path_cost': path_cost,
        'path_length': path_length,
        'nodes_expanded': nodes_expanded,
        'max_queue_size': max_queue,
        'time_ms': time_ms,
        'success': path is not None
    }

if __name__ == '__main__':
    # Unit tests
    print("Running Dijkstra Unit Tests...")
    
    # 1. Dijkstra-Std encontra caminho minimo em grade 3x3 sem obstaculos
    from nav_utils import create_base_graph_and_navigable
    base_graph, _ = create_base_graph_and_navigable(3, 3, obstacle_ratio=0.0)
    occ_grid_empty = np.zeros((3, 3))
    
    frame_graph = build_frame_graph(base_graph, occ_grid_empty)
    result = dijkstra_std(frame_graph, (0,0), (2,2))
    
    assert result['success'], "Test 1 Failed: Dijkstra didn't find path"
    assert result['path_length'] == 4, f"Test 1 Failed: Path length is {result['path_length']} expected 4"
    print("Test 1 Passed: Dijkstra found shortest path in empty grid.")
    
    # 2. Dijkstra-Std retorna path=None quando target isolado
    occ_grid_isolated = np.zeros((3, 3))
    # Block nodes around target (2,2)
    occ_grid_isolated[1, 2] = 500.0
    occ_grid_isolated[2, 1] = 500.0
    
    frame_graph = build_frame_graph(base_graph, occ_grid_isolated)
    result = dijkstra_std(frame_graph, (0,0), (2,2))
    
    assert not result['success'], "Test 2 Failed: Path should be None"
    assert result['path'] is None, "Test 2 Failed: Path should be None"
    print("Test 2 Passed: Target isolated returned path=None.")
    print("All tests passed.")
