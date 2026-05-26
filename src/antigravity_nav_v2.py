import numpy as np
import math
import random

def attractive_force(robot, goal, k_att=1.0):
    dr = goal[0] - robot[0]
    dc = goal[1] - robot[1]
    return np.array([k_att * dr, k_att * dc], dtype=float)

def repulsive_force(robot, occ_grid, k_rep=500.0, d0=1.5, block_threshold=400.0):
    rows, cols = occ_grid.shape
    F_rep = np.array([0.0, 0.0], dtype=float)
    r0, c0 = robot
    search_radius = int(math.ceil(d0 * 2))

    for dr in range(-search_radius, search_radius + 1):
        for dc in range(-search_radius, search_radius + 1):
            r, c = r0 + dr, c0 + dc
            if 0 <= r < rows and 0 <= c < cols:
                if occ_grid[r, c] > block_threshold:
                    d = math.sqrt(dr*dr + dc*dc)
                    if 0 < d < d0:
                        magnitude = k_rep * (1.0/d - 1.0/d0) * (1.0 / (d*d))
                        direction = np.array([dr, dc], dtype=float) / d
                        F_rep += magnitude * direction

    return F_rep

def detect_corridor(robot, occ_grid, block_threshold):
    r, c = robot
    rows, cols = occ_grid.shape
    left_blocked = (c > 0 and occ_grid[r, c-1] > block_threshold) or c == 0
    right_blocked = (c < cols-1 and occ_grid[r, c+1] > block_threshold) or c == cols - 1
    up_blocked = (r > 0 and occ_grid[r-1, c] > block_threshold) or r == 0
    down_blocked = (r < rows-1 and occ_grid[r+1, c] > block_threshold) or r == rows - 1

    # Se bloqueado em ambos os lados de um eixo
    if (left_blocked and right_blocked) or (up_blocked and down_blocked):
        return True  # esta em corredor
    return False

def compute_resultant_force_v2(robot, goal, occ_grid, config, block_threshold=400.0, max_force=10.0):
    k_att = config.get("K_ATTRACTIVE", 1.0)
    k_rep = config.get("K_REPULSIVE", 500.0)
    d0 = config.get("D0", 1.5)
    goal_threshold = config.get("GOAL_THRESHOLD", 1.0)
    corridor_mode = config.get("CORRIDOR_MODE", True)
    goal_zone_radius = config.get("GOAL_ZONE_RADIUS", 3)

    dist_to_goal = math.sqrt((robot[0]-goal[0])**2 + (robot[1]-goal[1])**2)
    
    # Modo Zona de Chegada
    if dist_to_goal <= goal_zone_radius:
        k_att_efetivo = k_att * 3.0
        k_rep_efetivo = k_rep * 0.1
    else:
        k_att_efetivo = k_att
        k_rep_efetivo = k_rep

    # Modo Corredor
    if corridor_mode and detect_corridor(robot, occ_grid, block_threshold):
        k_rep_efetivo = k_rep_efetivo * 0.3

    F_att = attractive_force(robot, goal, k_att_efetivo)
    
    if dist_to_goal < goal_threshold:
        F_rep = np.array([0.0, 0.0])
    else:
        F_rep = repulsive_force(robot, occ_grid, k_rep_efetivo, d0, block_threshold)

    F_total = F_att + F_rep

    magnitude = np.linalg.norm(F_total)
    if magnitude > max_force:
        F_total = (F_total / magnitude) * max_force

    is_in_corridor = detect_corridor(robot, occ_grid, block_threshold)

    return F_total, is_in_corridor

def force_to_direction(F_total):
    fr, fc = F_total

    if abs(fr) >= abs(fc):
        dr = int(np.sign(fr))
        dc = 0
    else:
        dr = 0
        dc = int(np.sign(fc))

    if np.linalg.norm(F_total) < 0.1:
        dr, dc = 0, 0

    return (dr, dc)

def is_valid_move(robot, dr, dc, rows, cols, occ_grid, block_threshold=400.0):
    new_r = robot[0] + dr
    new_c = robot[1] + dc

    if not (0 <= new_r < rows and 0 <= new_c < cols):
        return False

    if occ_grid[new_r, new_c] > block_threshold:
        return False

    return True

def smart_fallback(robot, goal, occ_grid, rows, cols, visited_history, block_threshold=400.0,
                   hp=50.0, of=1.0, gf=1.0):
    directions = [(0,1), (0,-1), (1,0), (-1,0)]
    scores = []

    for dr, dc in directions:
        new_r, new_c = robot[0] + dr, robot[1] + dc
        if not is_valid_move(robot, dr, dc, rows, cols, occ_grid, block_threshold):
            continue

        score = 0
        
        # Penaliza se ja visitou recentemente
        if (new_r, new_c) in visited_history:
            score -= hp

        # Bonifica se aumenta distancia dos obstaculos (occ_value menor é melhor)
        occ_value = occ_grid[new_r, new_c]
        score -= of * occ_value

        # Bonifica se aproxima do goal (Manhattan distance)
        goal_dist = abs(new_r - goal[0]) + abs(new_c - goal[1])
        score -= gf * goal_dist

        scores.append((score, dr, dc))

    if scores:
        # Choose the move with the highest score
        return max(scores, key=lambda x: x[0])[1:]
        
    return (0, 0)

if __name__ == '__main__':
    # Unit tests
    print("Running AntiGravity v2.0 Unit Tests...")
    
    occ_grid_empty = np.zeros((5, 5))
    robot = (0, 0)
    goal = (4, 4)
    config = {
        "K_ATTRACTIVE": 1.0,
        "K_REPULSIVE": 500.0,
        "D0": 1.5,
        "GOAL_THRESHOLD": 1.0,
        "CORRIDOR_MODE": True,
        "GOAL_ZONE_RADIUS": 3,
    }
    path = [robot]
    for _ in range(20):
        if robot == goal:
            break
        F, _ = compute_resultant_force_v2(robot, goal, occ_grid_empty, config)
        dr, dc = force_to_direction(F)
        if is_valid_move(robot, dr, dc, 5, 5, occ_grid_empty):
            robot = (robot[0]+dr, robot[1]+dc)
            path.append(robot)
        else:
            break
            
    assert path[-1] == goal, "Test 1 Failed: Robot did not reach goal in empty grid"
    print("Test 1 Passed: Reached goal in empty grid.")

    # Test corridor detection
    occ_grid_corridor = np.zeros((3, 3))
    occ_grid_corridor[0, 1] = 500 # up blocked
    occ_grid_corridor[2, 1] = 500 # down blocked
    robot = (1, 1)
    assert detect_corridor(robot, occ_grid_corridor, 400.0) == True, "Test 2 Failed: Corridor not detected"
    print("Test 2 Passed: Corridor detected.")
    
    # Test smart fallback
    from collections import deque
    history = deque(maxlen=10)
    history.append((1, 0)) # Came from left
    occ_grid = np.zeros((3, 3))
    occ_grid[1, 2] = 500 # Blocked right
    robot = (1, 1)
    goal = (1, 2)
    dr, dc = smart_fallback(robot, goal, occ_grid, 3, 3, history)
    # The best move should be up or down, not right (blocked) and not left (in history)
    assert (dr, dc) in [(1, 0), (-1, 0)], f"Test 3 Failed: Smart fallback chose {(dr, dc)}"
    print("Test 3 Passed: Smart fallback avoided history.")

    print("All tests passed.")
