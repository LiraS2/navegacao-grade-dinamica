# SPEC DE IMPLEMENTACAO — AntiGravity (Campos Potenciais) para Navegacao em Grade Dinamica

**Versao:** 1.0 (Implementacao)
**Data:** 2026-05-18
**Escopo:** Algoritmo AntiGravity baseado em campos potenciais artificiais. Implementacao do zero.
**Objetivo:** Gerar baseline de navegacao reativa para comparacao com Dijk-Std no artigo.

---

## 1. Conceito

AntiGravity modela o ambiente como um campo de forcas:
- **Forca ATRATIVA:** puxa o robo em direcao ao goal (potencial quadratico).
- **Forca REPULSIVA:** empurra o robo longe de pedestres e obstaculos (potencial inverso ao quadrado).
- **Resultante:** vetor soma das forcas define a direcao de movimento do robo.

Diferente do Dijkstra (planejamento global), AntiGravity e **reativo** — decide o proximo passo baseado apenas no estado atual do frame, sem busca em grafo.

---

## 2. PARAMETROS FIXOS

```python
# Constantes fisicas e de simulacao
CELL_SIZE = 0.40              # metros
PMAX = 500.0                  # penalidade maxima gaussiana (reutilizado do Dijk-Std)
INFLUENCE_RADIUS = 1.5        # celulas (sigma) — raio de repulsao
PROXIMAL_FACTOR = 2.0         # amplificador
BLOCK_THRESHOLD = 400.0       # se ocupacao > threshold, celula e bloqueada
MAX_STEPS = 15000             # limite de passos do robo
SEED = 42                     # reprodutibilidade total

# Parametros especificos do AntiGravity
K_ATTRACTIVE = 1.0            # ganho da forca atrativa
K_REPULSIVE = 500.0           # ganho da forca repulsiva
D0 = 1.5                      # distancia minima de influencia repulsiva (celulas)
GOAL_THRESHOLD = 1.0          # distancia (celulas) para considerar goal alcancado
MAX_FORCE_MAGNITUDE = 10.0    # limite superior da magnitude da forca resultante

# Cenarios (mesmos do Dijk-Std)
SCENARIOS = {
    "BR-06": {
        "arena_m": (25, 10),
        "grid": (25, 63),
        "navigable_estimate": 639,
        "frames": 400,
        "lambda_poisson": 8.79,
        "start": (0, 0),
        "end": (24, 62),
    },
    "CN-01": {
        "arena_m": (15, 20),
        "grid": (50, 38),
        "navigable_estimate": 599,
        "frames": 99,
        "lambda_poisson": 34.32,
        "start": (0, 0),
        "end": (49, 37),
    }
}
```

---

## 3. Geracao de Pedestres (Frame a Frame) — DETERMINISTICA

Mesma implementacao do Dijk-Std:
```python
import random
import math

def generate_frame_pedestrians(frame_idx, lambda_val, grid_rows, grid_cols, seed=42):
    rng = random.Random(seed + frame_idx * 1000003)

    n_pedestrians = 0
    L = math.exp(-lambda_val)
    p = 1.0
    while p > L:
        n_pedestrians += 1
        p *= rng.random()
    n_pedestrians -= 1

    positions = []
    for _ in range(n_pedestrians):
        r = rng.randint(0, grid_rows - 1)
        c = rng.randint(0, grid_cols - 1)
        positions.append((r, c))

    return positions
```

---

## 4. Calculo de Ocupacao Gaussiana

Mesma implementacao do Dijk-Std:
```python
import numpy as np
import math

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
```

---

## 5. ALGORITMO ANTIGRAVITY

### 5.1 Forca Atrativa (potencial quadratico)
```python
def attractive_force(robot, goal, k_att=1.0):
    dr = goal[0] - robot[0]
    dc = goal[1] - robot[1]
    return np.array([k_att * dr, k_att * dc])
```

### 5.2 Forca Repulsiva (potencial inverso)
```python
def repulsive_force(robot, occ_grid, k_rep=500.0, d0=1.5, block_threshold=400.0):
    rows, cols = occ_grid.shape
    F_rep = np.array([0.0, 0.0])
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
                        direction = np.array([dr, dc]) / d
                        F_rep += magnitude * direction

    return F_rep
```

### 5.3 Forca Resultante e Escolha de Direcao
```python
def compute_resultant_force(robot, goal, occ_grid, 
                            k_att=1.0, k_rep=500.0, d0=1.5, 
                            block_threshold=400.0, max_force=10.0):
    F_att = attractive_force(robot, goal, k_att)
    F_rep = repulsive_force(robot, occ_grid, k_rep, d0, block_threshold)

    F_total = F_att + F_rep

    magnitude = np.linalg.norm(F_total)
    if magnitude > max_force:
        F_total = F_total / magnitude * max_force

    return F_total
```

### 5.4 Conversao de Forca para Movimento em Grade
```python
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
```

### 5.5 Verificacao de Validade do Movimento
```python
def is_valid_move(robot, dr, dc, rows, cols, occ_grid, block_threshold=400.0):
    new_r = robot[0] + dr
    new_c = robot[1] + dc

    if not (0 <= new_r < rows and 0 <= new_c < cols):
        return False

    if occ_grid[new_r, new_c] > block_threshold:
        return False

    return True
```

### 5.6 Fallback: Movimento Aleatorio em Minimo Local
```python
def random_valid_move(robot, rows, cols, occ_grid, block_threshold=400.0, seed=42):
    rng = random.Random(seed + robot[0] * 1000 + robot[1])
    directions = [(0,1), (0,-1), (1,0), (-1,0), (0,0)]
    rng.shuffle(directions)

    for dr, dc in directions:
        if is_valid_move(robot, dr, dc, rows, cols, occ_grid, block_threshold):
            return (dr, dc)

    return (0, 0)
```

---

## 6. SIMULACAO FRAME A FRAME

```python
def run_antigravity_simulation(scenario_config, seed=42):
    import random
    import numpy as np
    import time

    random.seed(seed)
    np.random.seed(seed)

    rows, cols = scenario_config['grid']
    n_frames = scenario_config['frames']
    lambda_val = scenario_config['lambda_poisson']
    start = scenario_config['start']

    base_grid = create_base_grid(rows, cols, obstacle_ratio=0.05, seed=seed)
    navigable = get_navigable_cells(base_grid)

    robot = start
    cleaned = {robot}
    passos = 0
    consecutive_fails = 0
    records = []

    for frame in range(1, n_frames + 1):
        pedestrians = generate_frame_pedestrians(
            frame_idx=frame,
            lambda_val=lambda_val,
            grid_rows=rows,
            grid_cols=cols,
            seed=seed
        )

        occ_grid = compute_occupancy(rows, cols, pedestrians)

        if passos >= MAX_STEPS or len(cleaned) >= len(navigable):
            break

        not_cleaned = [cell for cell in navigable if cell not in cleaned]
        if not not_cleaned:
            break

        next_goal = min(not_cleaned,
                       key=lambda c: abs(c[0]-robot[0]) + abs(c[1]-robot[1]))

        t0 = time.perf_counter()

        F_total = compute_resultant_force(
            robot, next_goal, occ_grid,
            k_att=K_ATTRACTIVE,
            k_rep=K_REPULSIVE,
            d0=D0,
            block_threshold=BLOCK_THRESHOLD,
            max_force=MAX_FORCE_MAGNITUDE
        )

        dr, dc = force_to_direction(F_total)

        if not is_valid_move(robot, dr, dc, rows, cols, occ_grid, BLOCK_THRESHOLD):
            dr, dc = random_valid_move(robot, rows, cols, occ_grid, BLOCK_THRESHOLD, seed)

        new_r = robot[0] + dr
        new_c = robot[1] + dc

        time_ms = (time.perf_counter() - t0) * 1000

        if (dr, dc) != (0, 0) and (new_r, new_c) != robot:
            robot = (new_r, new_c)
            cleaned.add(robot)
            passos += 1
            action = "ADVANCE"
            consecutive_fails = 0
            success = True
        else:
            action = "WAIT"
            consecutive_fails += 1
            success = False

            if consecutive_fails > 5:
                nearby = [c for c in not_cleaned
                         if abs(c[0]-robot[0]) + abs(c[1]-robot[1]) < 15]
                if nearby:
                    next_goal = random.choice(nearby)
                consecutive_fails = 0

        records.append({
            'scenario': scenario_config['name'],
            'frame': frame,
            'robot_r': robot[0],
            'robot_c': robot[1],
            'goal_r': next_goal[0],
            'goal_c': next_goal[1],
            'force_r': F_total[0],
            'force_c': F_total[1],
            'dr': dr,
            'dc': dc,
            'time_ms': time_ms,
            'success': success,
            'action': action,
            'pedestrians_count': len(pedestrians),
            'max_occupancy': occ_grid.max(),
            'coverage': len(cleaned) / len(navigable) * 100
        })

    return records
```

---

## 7. METRICAS FINAIS POR CENARIO

| Metrica | Descricao | Alvo BR-06 | Alvo CN-01 |
|---------|-----------|------------|------------|
| total_steps | Passos de ADVANCE | ~400 | ~99 |
| coverage_percent | len(cleaned) / navigable * 100 | > 85% | > 75% |
| avg_force_magnitude | Media de norma(F_total) | < 5.0 | < 5.0 |
| max_force_magnitude | Pico de norma(F_total) | < 10.0 | < 10.0 |
| total_time_ms | Soma de todos os tempos | < 100 | < 50 |
| avg_time_ms | Media por frame | < 0.5 | < 0.5 |
| success_rate | Frames com ADVANCE / total | > 90% | > 85% |
| wait_count | Frames com WAIT | < 40 | < 20 |
| max_consecutive_waits | Maximo de waits seguidos | < 5 | < 5 |
| minima_locais | Quantas vezes usou fallback aleatorio | < 10 | < 10 |

---

## 8. PROBLEMAS CONHECIDOS DO ANTIGRAVITY

### 8.1 Minimos Locais
- **Sintoma:** Robo fica oscilando entre duas celulas ou parado.
- **Causa:** Forca atrativa e repulsiva se cancelam.
- **Mitigacao:** Fallback aleatorio apos 2 frames parado.

### 8.2 Passagem em Corredores Estreitos
- **Sintoma:** Robo nao consegue passar entre dois pedestres proximos.
- **Causa:** Forca repulsiva de ambos os lados empurra o robo para tras.
- **Mitigacao:** Reduzir K_REPULSIVE ou aumentar D0 em corredores.

### 8.3 Oscilacao em Torno do Goal
- **Sintoma:** Robo circunda o goal sem nunca alcanca-lo.
- **Causa:** Forca repulsiva de pedestres proximos ao goal.
- **Mitigacao:** Quando distancia ao goal < GOAL_THRESHOLD, desativar repulsao.

---

## 9. ENTREGAVEIS

### 9.1 Codigo
- `antigravity_nav.py`: modulo principal (forcas, simulacao)
- `run_antigravity.py`: script que roda BR-06 e CN-01

### 9.2 Dados (output/antigravity/)
- `antigravity_raw.csv`: uma linha por frame
- `antigravity_summary.csv`: uma linha por cenario (metricas agregadas)

### 9.3 Graficos (output/antigravity/, 300 DPI, PNG)
1. `fig_coverage_over_time.png`: cobertura (%) vs frame
2. `fig_force_magnitude.png`: magnitude da forca resultante por frame
3. `fig_time_per_frame.png`: tempo (ms) por frame
4. `fig_trajectory.png`: trajetoria do robo sobre grade (ultimo frame)
5. `fig_occupancy_heatmap.png`: mapa de calor de ocupacao (ultimo frame)

---

## 10. RESTRICOES TECNICAS

- Python 3.10+
- Dependencias: numpy, pandas, matplotlib, random, time, math
- Proibido: networkx, heapq (nao usa busca em grafo)
- Performance: simulacao completa < 30 segundos em CPU comum
- Seed: random.seed(42) e np.random.seed(42) para reprodutibilidade

---

## 11. CRITERIOS DE ACEITACAO

- [ ] AntiGravity move o robo em grade 3x3 sem obstaculos ate o goal (teste unitario)
- [ ] AntiGravity desvia de 1 pedestre no caminho (teste unitario)
- [ ] BR-06 e CN-01 rodam sem erros e geram CSVs
- [ ] Cobertura final > 85% em BR-06, > 75% em CN-01
- [ ] Tempo medio por frame < 0.5ms (ambos cenarios)
- [ ] Nenhum tempo de frame > 10ms
- [ ] Graficos gerados em output/antigravity/
- [ ] antigravity_summary.csv contem 2 linhas (BR-06 e CN-01)
- [ ] Comparacao visual com Dijk-Std: trajetorias sobrepostas no mesmo grafico

---

## 12. COMPARACAO ESPERADA COM DIJK-STD

| Aspecto | Dijk-Std | AntiGravity |
|---------|----------|-------------|
| Tipo | Planejamento global | Reativo local |
| Busca em grafo | Sim (heapq) | Nao |
| Garantia de otimalidade | Sim (custo minimo) | Nao |
| Tempo por decisao | ~0.5-2ms | ~0.01-0.1ms |
| Memoria | O(V) para dist[] e pred[] | O(1) por frame |
| Robustez a mudancas | Requer replanejamento | Adapta-se instantaneamente |
| Minimos locais | Nao tem | Possivel (mitigado por fallback) |
| Cobertura esperada | > 90% | > 80% |

---

## 13. Rascunho de Paragrafo para o Artigo

"Como abordagem reativa de contraste, implementou-se o algoritmo AntiGravity baseado em campos potenciais artificiais. O metodo combina uma forca atrativa quadraticamente proporcional a distancia ate o goal nao-limpado mais proximo com forcas repulsivas inversamente quadraticas emanadas de celulas ocupadas por pedestres (K_att = 1,0; K_rep = 500; raio de influencia = 1,5 celulas). A cada quadro, o vetor resultante e convertido em movimento na grade 4-conectada, com fallback estocastico para escapar de minimos locais. Nos cenarios BR-06 e CN-01, o AntiGravity apresentou tempo de decisao medio de [X] ms — ordens de magnitude inferior ao Dijkstra — porem com cobertura de [Y]%, refletindo o compromisso inerente entre reatividade e completude em navegacao dinamica."

---

## 14. CHECKLIST DE IMPLEMENTACAO

| Etapa | Status |
|-------|--------|
| Modulo de forcas (atrativa + repulsiva) | PENDENTE |
| Conversao forca -> direcao em grade | PENDENTE |
| Fallback para minimos locais | PENDENTE |
| Loop de simulacao frame a frame | PENDENTE |
| Geracao de CSVs | PENDENTE |
| Geracao de graficos | PENDENTE |
| Testes unitarios | PENDENTE |
| Validacao contra criterios de aceitacao | PENDENTE |

---

**Status:** Pronta para implementacao.
**Proximo passo:** Codificar antigravity_nav.py e run_antigravity.py, rodar e validar.
