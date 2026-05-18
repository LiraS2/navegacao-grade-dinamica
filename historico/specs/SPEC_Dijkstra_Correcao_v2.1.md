# SPEC DE CORRECAO — Dijkstra Padrao (Dijk-Std) para Navegacao em Grade Dinamica

**Versao:** 2.1 (Correcao)
**Data:** 2026-05-18
**Escopo:** Foco EXCLUSIVO em Dijkstra Padrao (heapq). Todas as outras variantes foram movidas para `inativado.md`.
**Objetivo:** Corrigir bugs criticos identificados no benchmark anterior e gerar dados limpos para o artigo.

---

## 1. Diagnostico dos Problemas Anteriores

### 1.1 Dijk-Dial Travando (REMOVIDO)
- **Sintoma:** Tempos de ~5000ms em frames com falha.
- **Causa raiz:** Loop infinito na deteccao de esvaziamento de buckets quando o grafo esta desconexo.
- **Acao:** Removido completamente do escopo. Ver `inativado.md` para detalhes.

### 1.2 Falhas Massivas em Dijk-Std
| Cenario | Falhas Dijk-Std | Falhas esperadas (apos correcao) |
|---------|----------------|----------------------------------|
| BR-06 | 37 / 400 (9.3%) | < 5% |
| CN-01 | 26 / 99 (26.3%) | < 10% |

**Causas identificadas:**
1. **Pedestres nao deterministicos por frame** — RNG reamostrava a cada busca no mesmo frame.
2. **Ocupacao gaussiana com soma incorreta** — valores de `max_occupancy` chegavam a ~329, causando bloqueio excessivo.
3. **Falta de replanejamento de goal** — robo ficava preso por >5 frames sem mudar de objetivo.
4. **Metricas de tempo inconsistentes** — falhas deveriam ser detectadas em <1ms, mas as vezes demoravam.

### 1.3 Cobertura Baixa
| Cenario | Cobertura atual | Esperado | Minimo aceitavel |
|---------|----------------|----------|------------------|
| BR-06 | ~324 celulas (~51%) | ~639 (100%) | > 90% |
| CN-01 | ~73 celulas (~12%) | ~599 (100%) | > 80% |

**Causa:** Robo parou em posicoes intermediarias sem limpar tudo:
- BR-06 parou em (21, 26) no frame 400.
- CN-01 parou em (20, 0) no frame 99.

---

## 2. SCOPE: Dijkstra Padrao APENAS

### 2.1 Algoritmo mantido
- **Dijk-Std:** Implementacao com `heapq` (stdlib Python).
- **Complexidade:** O((V + E) log V) por busca.

### 2.2 Algoritmos removidos
- A* (heuristica inadmissivel com pesos dinamicos)
- Dijk-Bi (implementacao quebrada, mais lento que Std)
- Dijk-Dial (loop infinito em falhas)

> Ver `inativado.md` para pseudocodigo e motivos de remocao.

---

## 3. PARAMETROS FIXOS (Nao Negociaveis)

```python
# Constantes fisicas e de simulacao
CELL_SIZE = 0.40              # metros
BASE_WEIGHT = 1.0             # peso base da aresta
PMAX = 500.0                  # penalidade maxima gaussiana
INFLUENCE_RADIUS = 1.5        # celulas (sigma)
PROXIMAL_FACTOR = 2.0         # amplificador
PEDESTRIAN_RADIUS_M = 0.45    # metros (para calculo de footprint)
BLOCK_THRESHOLD = 400.0       # se peso > threshold, aresta e removida
MAX_STEPS = 15000             # limite de passos do robo
SEED = 42                     # reprodutibilidade total

# Cenarios
SCENARIOS = {
    "BR-06": {
        "arena_m": (25, 10),
        "grid": (25, 63),           # linhas, colunas
        "navigable_estimate": 639,
        "frames": 400,
        "lambda_poisson": 8.79,
        "start": (0, 0),
        "end": (24, 62),            # canto oposto
    },
    "CN-01": {
        "arena_m": (15, 20),
        "grid": (50, 38),           # linhas, colunas
        "navigable_estimate": 599,
        "frames": 99,
        "lambda_poisson": 34.32,
        "start": (0, 0),
        "end": (49, 37),            # canto oposto
    }
}
```

---

## 4. FIX 1: Geracao de Pedestres (Frame a Frame) — DETERMINISTICA

### 4.1 Regra de ouro
Cada frame deve ter pedestres **independentes e deterministicos**. Nao persistentes entre frames. Nao reamostrados.

### 4.2 Implementacao correta
```python
import random
import math

def generate_frame_pedestrians(frame_idx, lambda_val, grid_rows, grid_cols, seed=42):
    rng = random.Random(seed + frame_idx * 1000003)  # Seed unica por frame

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
```

### 4.3 Verificacao
- Frame 3 de BR-06 deve **sempre** gerar os mesmos 8 pedestres (com seed=42).
- Nao deve haver multiplas linhas para o mesmo frame no CSV.

---

## 5. FIX 2: Calculo de Ocupacao Gaussiana

### 5.1 Formula correta
```
occ(r,c) = sum PMAX * exp(-d^2 / (2*sigma^2)) * PROXIMAL_FACTOR
```
onde:
- `d` = distancia euclidiana em celulas entre (r,c) e o pedestre
- `sigma = INFLUENCE_RADIUS = 1.5`
- Se `d > 3*sigma` (4.5 celulas), o termo e desprezivel

### 5.2 Verificacao da formula
| Situacao | Calculo | Resultado |
|----------|---------|-----------|
| Pedestre na celula (d=0) | 500 * exp(0) * 2.0 | 1000 |
| 1 celula de distancia (d=1) | 500 * exp(-1/4.5) * 2.0 | 800.7 |
| 2 celulas (d=2) | 500 * exp(-4/4.5) * 2.0 | 411.1 |
| 3 celulas (d=3) | 500 * exp(-9/4.5) * 2.0 | 135.3 |

**Comportamento esperado:**
- Celula do pedestre: sempre bloqueada (1000 > 400).
- 1 celula de distancia: bloqueada se houver 2 pedestres proximos.
- 2 celulas: pode estar bloqueada dependendo da soma.

### 5.3 Peso da aresta
```python
def compute_edge_weight(occ_grid, u, v, base_weight=1.0, block_threshold=400.0):
    occ_u = occ_grid[u[0], u[1]]
    occ_v = occ_grid[v[0], v[1]]
    weight = base_weight + max(occ_u, occ_v)
    if weight > block_threshold:
        return None  # Aresta removida
    return weight
```

### 5.4 Otimizacao: janela de influencia
```python
radius = math.ceil(3 * INFLUENCE_RADIUS)  # 5 celulas
for pr, pc in pedestrians:
    for dr in range(-radius, radius + 1):
        for dc in range(-radius, radius + 1):
            r, c = pr + dr, pc + dc
            if 0 <= r < rows and 0 <= c < cols:
                d = math.sqrt(dr*dr + dc*dc)
                if d <= 3 * INFLUENCE_RADIUS:
                    occ[r, c] += PMAX * math.exp(-d*d / (2*sigma*sigma)) * PROXIMAL_FACTOR
```

---

## 6. FIX 3: Regra de WAIT vs BACKTRACK

### 6.1 Comportamento correto
```
Se path existe e len(path) >= 2:
    robot = path[1]      # avanca 1 celula
    cleaned.add(robot)
    passos += 1
    action = "ADVANCE"
    consecutive_fails = 0
Senao:
    robot nao se move      # WAIT
    frame += 1
    action = "WAIT"
    consecutive_fails += 1

    Se consecutive_fails > 5:
        # BACKTRACK: escolher outro goal aleatorio entre nao-limpados proximos
        candidates = [cell for cell in navigable if cell not in cleaned 
                      and manhattan_distance(robot, cell) < 20]
        if candidates:
            next_goal = random.choice(candidates)
        else:
            next_goal = random.choice([c for c in navigable if c not in cleaned])
        consecutive_fails = 0
```

### 6.2 Contador de falhas consecutivas
- Deve ser **persistente entre frames** (nao resetar a cada busca).
- Se o robo muda de goal, o contador zera.
- Se o robo avanca, o contador zera.

---

## 7. FIX 4: Metricas de Tempo

### 7.1 Problema anterior
Falhas demoravam ~3ms em vez de <0.1ms porque o heap esvaziava lentamente.

### 7.2 Solucao
```python
def dijkstra_std(graph, source, target):
    import heapq, time

    dist = {v: float('inf') for v in graph}
    pred = {v: None for v in graph}
    dist[source] = 0
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
    if dist[target] == float('inf'):
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
```

### 7.3 Verificacao de tempo
- **Falha rapida:** Se target isolado, heap esvazia em O(V log V), mas V < 2000 -> <1ms.
- **Sucesso rapido:** Goal adjacente -> 1-2 nos expandidos -> <0.05ms.

---

## 8. Estrutura do Grafo

### 8.1 Representacao
```python
graph = {
    (r, c): [((r+1, c), weight_s), ((r-1, c), weight_n), 
             ((r, c+1), weight_e), ((r, c-1), weight_w)],
    ...
}
```

### 8.2 Geracao do grafo base
1. Criar grade 4-conectada (vizinhanca Von Neumann).
2. Remover ~5% dos nos como obstaculos fixos (seed fixa, deterministico).
3. Garantir que start e end nunca sejam obstaculos.
4. Todas as arestas iniciam com peso BASE_WEIGHT = 1.0.

### 8.3 Atualizacao frame a frame
```python
def build_frame_graph(base_graph, occ_grid, block_threshold=400.0):
    frame_graph = {}
    for u, neighbors in base_graph.items():
        frame_graph[u] = []
        for v, _ in neighbors:
            w = compute_edge_weight(occ_grid, u, v)
            if w is not None:
                frame_graph[u].append((v, w))
    return frame_graph
```

---

## 9. Simulacao Frame a Frame (Cobertura Total)

### 9.1 Algoritmo principal
```python
def run_simulation(scenario_config, seed=42):
    random.seed(seed)
    np.random.seed(seed)

    rows, cols = scenario_config['grid']
    n_frames = scenario_config['frames']
    lambda_val = scenario_config['lambda_poisson']
    start = scenario_config['start']

    # 1. Criar grafo base
    base_graph, navigable = create_base_grid(rows, cols, obstacle_ratio=0.05, seed=seed)

    # 2. Estado do robo
    robot = start
    cleaned = {robot}
    passos = 0
    consecutive_fails = 0

    records = []

    for frame in range(1, n_frames + 1):
        # 3. Gerar pedestres deste frame (DETERMINISTICO)
        pedestrians = generate_frame_pedestrians(
            frame_idx=frame,
            lambda_val=lambda_val,
            grid_rows=rows,
            grid_cols=cols,
            seed=seed
        )

        # 4. Calcular ocupacao
        occ_grid = compute_occupancy(rows, cols, pedestrians)

        # 5. Construir grafo dinamico
        frame_graph = build_frame_graph(base_graph, occ_grid)

        # 6. Escolher goal
        if passos >= MAX_STEPS or len(cleaned) >= len(navigable):
            break

        not_cleaned = [cell for cell in navigable if cell not in cleaned]
        if not not_cleaned:
            break

        # Goal: nao-limpado mais proximo (Manhattan)
        next_goal = min(not_cleaned, 
                       key=lambda c: abs(c[0]-robot[0]) + abs(c[1]-robot[1]))

        # 7. Busca Dijkstra
        result = dijkstra_std(frame_graph, robot, next_goal)

        # 8. Executar acao
        if result['success'] and result['path_length'] >= 1:
            robot = result['path'][1]  # avanca 1 celula
            cleaned.add(robot)
            passos += 1
            action = "ADVANCE"
            consecutive_fails = 0
        else:
            action = "WAIT"
            consecutive_fails += 1

            if consecutive_fails > 5:
                # BACKTRACK: escolher goal aleatorio proximo
                nearby = [c for c in not_cleaned 
                         if abs(c[0]-robot[0]) + abs(c[1]-robot[1]) < 15]
                if nearby:
                    next_goal = random.choice(nearby)
                consecutive_fails = 0

        # 9. Registrar metricas
        records.append({
            'scenario': scenario_config['name'],
            'frame': frame,
            'robot_r': robot[0],
            'robot_c': robot[1],
            'goal_r': next_goal[0],
            'goal_c': next_goal[1],
            'nodes_expanded': result['nodes_expanded'],
            'max_queue_size': result['max_queue_size'],
            'time_ms': result['time_ms'],
            'path_cost': result['path_cost'],
            'path_length': result['path_length'],
            'success': result['success'],
            'action': action,
            'pedestrians_count': len(pedestrians),
            'max_occupancy': occ_grid.max(),
            'coverage': len(cleaned) / len(navigable) * 100
        })

    return records
```

---

## 10. Metricas Finais por Cenario

| Metrica | Descricao | Alvo BR-06 | Alvo CN-01 |
|---------|-----------|------------|------------|
| total_steps | Passos de ADVANCE | ~400 | ~99 |
| coverage_percent | len(cleaned) / navigable * 100 | > 95% | > 85% |
| avg_nodes_expanded | Media por busca bem-sucedida | < 50 | < 100 |
| max_nodes_expanded | Pico em uma busca | < 500 | < 1000 |
| avg_queue_size | Media do pico da fila | < 20 | < 50 |
| total_time_ms | Soma de todos os tempos | < 500 | < 200 |
| avg_time_ms | Media por busca | < 1 | < 2 |
| success_rate | Buscas com path != None / total | > 95% | > 90% |
| wait_count | Frames com WAIT | < 20 | < 15 |
| max_consecutive_waits | Maximo de waits seguidos | < 5 | < 5 |

---

## 11. Entregaveis

### 11.1 Codigo
- `dijkstra_nav.py`: modulo principal (grafo, Dijkstra, simulacao)
- `run_simulation.py`: script que roda BR-06 e CN-01

### 11.2 Dados (output/dijkstra/)
- `dijkstra_raw.csv`: uma linha por frame
- `dijkstra_summary.csv`: uma linha por cenario (metricas agregadas)

### 11.3 Graficos (output/dijkstra/, 300 DPI, PNG)
1. `fig_coverage_over_time.png`: cobertura (%) vs frame
2. `fig_nodes_expanded_dist.png`: histograma de nos expandidos
3. `fig_time_per_search.png`: tempo (ms) por frame
4. `fig_queue_size.png`: pico da fila por frame
5. `fig_occupancy_heatmap.png`: mapa de calor de ocupacao (ultimo frame)

---

## 12. Restricoes Tecnicas

- Python 3.10+
- Dependencias: numpy, pandas, matplotlib, heapq (stdlib), random, time
- Proibido: networkx no loop de busca
- Performance: simulacao completa < 2 minutos em CPU comum
- Seed: random.seed(42) e np.random.seed(42) para reprodutibilidade

---

## 13. Criterios de Aceitacao

- [ ] Dijkstra-Std encontra caminho minimo em grade 3x3 sem obstaculos (teste unitario)
- [ ] Dijkstra-Std retorna path=None quando target isolado (teste unitario)
- [ ] BR-06 e CN-01 rodam sem erros e geram CSVs
- [ ] Cobertura final > 90% em BR-06, > 80% em CN-01
- [ ] Tempo medio por busca < 1ms (BR-06) e < 2ms (CN-01)
- [ ] Nenhum tempo de busca > 100ms (detecta loops)
- [ ] Graficos gerados em output/dijkstra/
- [ ] dijkstra_summary.csv contem 2 linhas (BR-06 e CN-01)
- [ ] inativado.md existe com variantes removidas

---

## 14. Rascunho de Paragrafo para o Artigo

"Como baseline deterministico para o roteamento em grades de ocupacao dinamica, implementou-se o algoritmo de Dijkstra com fila de prioridade por heap binaria. A busca foi executada quadro a quadro sobre grafos 4-conectados com pesos atualizados por campo gaussiano proximal (Pmax = 500, raio de influencia 1,5 celulas, fator proximal 2,0). Nos cenarios BR-06 (lambda = 8,79) e CN-01 (lambda = 34,32) do Cultural Crowds Dataset, o Dijkstra expandiu em media [X] nos por busca, com pico de fila de [Y] elementos e tempo medio de [Z] ms, alcancando [W]% de cobertura do ambiente em [V] passos. A ausencia de heuristica admissivel torna o metodo robusto a distorcoes de custo introduzidas por pedestres dinamicos, embora exija maior exploracao da fronteira de busca em comparacao a abordagens informadas."

---

## 15. Checklist de Correcoes vs v2.0

| Problema | v2.0 | v2.1 (esta spec) |
|----------|------|------------------|
| Algoritmos | A* + Dijk-Std + Dijk-Bi + Dijk-Dial | Apenas Dijk-Std |
| Pedestres | RNG global, possivel reamostragem | Seed por frame, deterministico |
| Ocupacao | Possivel soma incorreta | Janela de 5 celulas, max() na aresta |
| WAIT/BACKTRACK | Regra vaga | Contador persistente, troca de goal apos 5 falhas |
| Tempo em falhas | Podia demorar | Heapq esvazia naturalmente, <1ms |
| Cobertura | ~51% BR-06, ~12% CN-01 | Alvo >90% BR-06, >80% CN-01 |
| Variantes quebradas | Incluidas no benchmark | Movidas para inativado.md |

---

**Status:** Pronta para implementacao.
**Proximo passo:** Codificar, rodar e validar contra criterios de aceitacao.
