import random
import math
import numpy as np

def generate_frame_pedestrians(frame_idx, lambda_val, grid_rows, grid_cols, seed=42):
    rng = random.Random(seed + frame_idx * 1000003)

    # Poisson: numero de pedestres (algoritmo de Knuth)
    n_pedestrians = 0
    L = math.exp(-lambda_val)
    p = 1.0
    while p > L:
        n_pedestrians += 1
        p *= rng.random()
    n_pedestrians -= 1

    # Posicoes aleatorias dentro da grade
    positions = []
    for _ in range(n_pedestrians):
        r = rng.randint(0, grid_rows - 1)
        c = rng.randint(0, grid_cols - 1)
        positions.append((r, c))

    return positions

def compute_occupancy(rows, cols, pedestrians, pmax=500.0, 
                      influence_radius=1.5, proximal_factor=2.0):
    occ = np.zeros((rows, cols), dtype=np.float64)
    sigma = influence_radius
    radius = math.ceil(3 * sigma)

    for pr, pc in pedestrians:
        for dr in range(-radius, radius + 1):
            for dc in range(-radius, radius + 1):
                r, c = pr + dr, pc + dc
                if 0 <= r < rows and 0 <= c < cols:
                    d = math.sqrt(dr*dr + dc*dc)
                    if d <= 3 * sigma:
                        occ[r, c] += pmax * math.exp(-d*d / (2*sigma*sigma)) * proximal_factor

    return occ

def create_base_grid(rows, cols, obstacle_ratio=0.05, seed=42):
    rng = random.Random(seed)
    grid = np.zeros((rows, cols), dtype=int)
    
    # We should not place obstacles at (0,0) and the opposite corner (rows-1, cols-1) 
    # as these are the typical start and end points in our scenarios.
    protected_cells = {(0, 0), (rows-1, cols-1)}
    
    total_cells = rows * cols
    num_obstacles = int(total_cells * obstacle_ratio)
    
    obstacles_placed = 0
    while obstacles_placed < num_obstacles:
        r = rng.randint(0, rows - 1)
        c = rng.randint(0, cols - 1)
        if (r, c) not in protected_cells and grid[r, c] == 0:
            grid[r, c] = 1
            obstacles_placed += 1
            
    return grid

def get_navigable_cells(base_grid):
    navigable = []
    rows, cols = base_grid.shape
    for r in range(rows):
        for c in range(cols):
            if base_grid[r, c] == 0:
                navigable.append((r, c))
    return navigable

# For Dijkstra, it expects (base_graph, navigable) from create_base_grid, 
# so we provide a specific function for it.
def create_base_graph_and_navigable(rows, cols, obstacle_ratio=0.05, seed=42):
    grid = create_base_grid(rows, cols, obstacle_ratio, seed)
    navigable = get_navigable_cells(grid)
    
    base_graph = {}
    nav_set = set(navigable)
    for r, c in navigable:
        base_graph[(r, c)] = []
        for dr, dc in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
            nr, nc = r + dr, c + dc
            if (nr, nc) in nav_set:
                base_graph[(r, c)].append(((nr, nc), 1.0))
                
    return base_graph, navigable
