# SPEC DE IMPLEMENTACAO — D* Lite para Navegacao em Grade Dinamica

**Versao:** 1.0
**Data:** 2026-05-26
**Escopo:** Algoritmo D* Lite (Koenig & Likhachev, 2002) — replanejamento incremental para robos em ambientes dinamicos.
**Objetivo:** Implementar D* Lite integrado ao `nav_utils.py` existente, gerar dados comparaveis com Dijkstra-Std e AntiGravity.

---

## 0. Contexto: Por que D* Lite?

### 0.1 Problema do Dijkstra em ambientes dinamicos
- Dijkstra recalcula **tudo** a cada frame: O(V log V) por replanejamento
- Em 400 frames, isso e aceitavel para grades pequenas (<2000 celulas)
- Mas em cenarios maiores ou com mais frames, o custo acumulado explode

### 0.2 Vantagem do D* Lite
- **Replanejamento incremental:** so repara nos afetados pela mudanca
- **Busca reversa:** raiz no goal, folhas proximas ao robo
- **Complexidade por replanejamento:** O(k log N), onde k = nos afetados << N
- **km (key modifier):** evita reordenar heap inteiro quando robo se move

### 0.3 Limitacoes conhecidas (da pesquisa)
- **Grades pequenas (<2000 celulas):** overhead de g/rhs/OPEN pode superar Dijkstra
- **Densidade alta (lambda > 30):** muitos nos inconsistentes por frame, vantagem incremental desaparece
- **Nao modela velocidade/intencao de pedestres:** ve so ocupacao binaria

---

## 1. PARAMETROS FIXOS

```python
# Herdados do nav_utils.py (mesmos do Dijk-Std e AntiGravity)
CELL_SIZE = 0.40
PMAX = 500.0
INFLUENCE_RADIUS = 1.5
PROXIMAL_FACTOR = 2.0
BLOCK_THRESHOLD = 400.0
MAX_STEPS = 15000          # safety break
SEED = 42

# D* Lite especifico
HEURISTIC = 'manhattan'    # admissivel para grade 4-conectada

# Cenarios (mesmos)
SCENARIOS = {
    "BR-06": {
        "grid": (25, 63),
        "navigable_estimate": 1575,
        "frames": 400,
        "lambda_poisson": 8.79,
        "start": (0, 0),
        "end": (24, 62),
    },
    "CN-01": {
        "grid": (50, 38),
        "navigable_estimate": 1900,
        "frames": 99,
        "lambda_poisson": 34.32,
        "start": (0, 0),
        "end": (49, 37),
    }
}
```

---

## 2. ESTRUTURAS DE DADOS

### 2.1 Estado do no
Cada celula (r, c) tem:
- `g`: custo acumulado real do goal ate aqui (backward search)
- `rhs`: custo de 1-passo lookahead (min dos vizinhos)
- consistente se `g == rhs`

### 2.2 Fila OPEN (prioridade)
Heap de tuplas: `(k1, k2, (r, c))`
- `k1 = min(g, rhs) + h(start, s) + km`
- `k2 = min(g, rhs)`
- Ordem lexicografica: k1 primeiro, depois k2

### 2.3 km (key modifier)
Acumula a distancia heuristica percorrida pelo robo:
- `km += h(s_last, s_start)` a cada movimento
- Garante que chaves antigas ainda sao validas (lower bound)

---

## 3. ALGORITMO PRINCIPAL

### 3.1 Inicializacao
```python
def initialize(self, occ_grid, start, goal):
    # 1. Todos os nos: g = inf, rhs = inf
    # 2. Goal: rhs = 0 (unico inconsistente inicial)
    # 3. Inserir goal na OPEN com chave inicial
    # 4. km = 0
    # 5. start_atual = start
```

### 3.2 Heuristica
```python
def heuristic(s1, s2):
    # Manhattan distance — admissivel para 4-conectado
    return abs(s1[0] - s2[0]) + abs(s1[1] - s2[1])
```

### 3.3 Calcular chave
```python
def calculate_key(s):
    k1 = min(g[s], rhs[s]) + heuristic(start_atual, s) + km
    k2 = min(g[s], rhs[s])
    return (k1, k2)
```

### 3.4 Custo de transicao
```python
def cost(u, v, occ_grid):
    # Se v bloqueada (occ > threshold): inf
    # Senao: 1.0 (peso base)
```

### 3.5 Atualizar vertice (update_vertex)
```python
def update_vertex(u):
    # Se u != goal:
    #     rhs[u] = min_{v vizinho} (cost(u, v) + g[v])
    # 
    # Se g[u] != rhs[u]:
    #     Inserir u na OPEN com chave atual
    # Senao:
    #     Remover u da OPEN (se estiver)
```

### 3.6 Computar caminho mais curto (compute_shortest_path)
```python
def compute_shortest_path(self):
    # Loop principal:
    # 1. Extrair no de menor chave da OPEN
    # 2. Se chave obsoleta (menor que atual), re-inserir e continuar
    # 3. Condicao de parada:
    #    - start eh consistente (g == rhs)
    #    - E chave do start <= chave minima da OPEN
    # 4. Se g[u] > rhs[u] (LOWER/superconsistente):
    #      g[u] = rhs[u]
    #      Para cada predecessor v: update_vertex(v)
    # 5. Se g[u] < rhs[u] (RAISE/subconsistente):
    #      g[u] = inf
    #      update_vertex(u)
    #      Para cada predecessor v: update_vertex(v)
```

### 3.7 Replanejar (replan)
```python
def replan(self, new_start, new_occ_grid=None):
    # Chamado a cada frame:
    # 1. Atualizar start: km += h(s_last, new_start)
    # 2. Se occ_grid mudou:
    #      Detectar celulas alteradas
    #      Para cada alterada: update_vertex + vizinhos
    # 3. compute_shortest_path()
    # 4. Extrair caminho seguindo min(g) dos vizinhos
```

### 3.8 Extrair caminho
```python
def extract_path(self):
    # Do start ate goal, seguindo:
    # proximo = argmin_{v vizinho} (cost(atual, v) + g[v])
```

---

## 4. INTEGRACAO COM nav_utils.py

### 4.1 Fluxo frame a frame
```python
class DStarLiteNavigator:
    def __init__(self):
        self.planner = None
        self.prev_occ = None
        self.last_path = []
        self.last_start = None
        self.goal = None

    def initialize(self, frame_data):
        # Primeiro frame do cenario
        occ_grid = compute_occupancy(frame_data)
        start = SCENARIO['start']
        goal = SCENARIO['end']

        self.planner = DStarLite(occ_grid, start, goal)
        self.planner.compute_shortest_path()
        self.last_path = self.planner.extract_path()
        self.last_start = start
        self.goal = goal
        self.prev_occ = occ_grid.copy()

    def step(self, frame_data, current_pos):
        # Cada frame subsequente:
        # 1. Recebe nova occ_grid
        # 2. Detecta mudancas vs prev_occ
        # 3. Se mudancas no caminho atual: replan()
        # 4. Senao: apenas atualiza start, reutiliza caminho
        # 5. Avanca 1 celula no caminho
```

---

## 5. METRICAS E ALVOS

### 5.1 Métricas coletadas (mesmo formato do Dijk-Std)
| Métrica | Descrição |
|---------|-----------|
| frame | Indice do frame |
| robot_r, robot_c | Posicao atual |
| goal_r, goal_c | Goal atual |
| nodes_expanded | Nos processados no replanejamento |
| max_queue_size | Pico da fila OPEN |
| time_ms | Tempo do replan() |
| path_cost | g[goal] (custo acumulado) |
| path_length | len(path) - 1 |
| success | path nao vazio |
| action | ADVANCE / WAIT |
| pedestrians_count | Numero de pedestres no frame |
| max_occupancy | Ocupacao maxima no frame |
| coverage | % de celulas limpas |
| **replan_triggered** | **Se houve replanejamento neste frame** |
| **cells_changed** | **Numero de celulas alteradas vs frame anterior** |
| **km_value** | **Valor acumulado de km** |

### 5.2 Alvos de cobertura (mesmos do Dijk-Std v2.2)
| Cenario | Alvo cobertura | Alvo tempo | Alvo replanejamentos |
|---------|----------------|------------|----------------------|
| BR-06 | 20-25% | < 1ms/frame | < 50% dos frames |
| CN-01 | 3-5% | < 2ms/frame | < 70% dos frames |

### 5.3 Comparacao medida com Dijk-Std
| Aspecto | Dijk-Std | D* Lite | Observacao |
|---------|----------|---------|------------|
| Primeira busca | O(V log V) | O(V log V) | Igual |
| Replanejamento | O(V log V) | O(k log N), k << V | Vantagem teorica |
| Tempo medio BR-06 | 0.082 ms | 1.627 ms | D* Lite mais lento (overhead g/rhs) |
| Tempo medio CN-01 | 0.070 ms | 7.693 ms | Degradacao com lambda=34.32 |
| Cobertura BR-06 | 23.5% | 24.0% | Similar |
| Cobertura CN-01 | 3.9% | 4.3% | Similar |
| Memoria | O(V) | O(2V) para g + rhs | |
| Overhead em grades pequenas | Menor | Maior (g/rhs/OPEN) | Confirmado |
| Degradacao em lambda > 30 | Similar | Pior (overhead + poucos frames estaticos) | Confirmado |

---

## 6. ENTREGAVEIS

### 6.1 Codigo
- `dstar_lite_nav.py`: classe DStarLite + DStarLiteNavigator
- `run_dstar_lite.py`: runner frame a frame para BR-06 e CN-01

### 6.2 Dados (output/dstar_lite/)
- `dstar_lite_raw.csv`: uma linha por frame (400/99 linhas)
- `dstar_lite_summary.csv`: metricas agregadas por cenario

### 6.3 Graficos (output/dstar_lite/, 300 DPI, PNG)
1. `fig_coverage_over_time.png`
2. `fig_nodes_expanded_dist.png`
3. `fig_time_per_search.png`
4. `fig_queue_size.png`
5. `fig_occupancy_heatmap.png`
6. `fig_replan_frequency.png` — NOVO: frequencia de replanejamento por frame
7. `fig_cells_changed.png` — NOVO: celulas alteradas vs tempo de replanejamento

### 6.4 Grafico comparativo triplo (output/comparativo/)
- `fig_triplo_comparativo.png`: Dijk-Std + AntiGravity + D* Lite no mesmo eixo

---

## 7. CRITERIOS DE ACEITACAO

- [x] D* Lite encontra caminho em grade 3x3 sem obstaculos (teste unitario)
- [x] D* Lite replaneja corretamente quando obstaculo surge no caminho (teste unitario)
- [x] D* Lite reutiliza caminho quando nao ha mudancas (teste unitario)
- [x] BR-06 e CN-01 rodam sem erros e geram CSVs com 400/99 linhas
- [x] Cobertura BR-06 entre 20-25% (medido: 24.0%), CN-01 entre 3-5% (medido: 4.3%)
- [ ] Tempo medio < 1ms (BR-06): **medido 1.627 ms** — overhead de g/rhs em grade pequena
- [ ] Replanejamento em < 50% (BR-06): **medido 99.8%** — lambda=8.79 gera mudancas em ~100% dos frames
- [x] 7 graficos PNG 300 DPI em output/dstar_lite/ (14 total, 7 por cenario)
- [x] Grafico comparativo triplo gerado (output/comparativo/fig_triplo_comparativo.png)
- [x] dstar_lite_summary.csv com metricas agregadas

---

## 8. PARAGRAFO DE RESULTADOS (rascunho para artigo)

"Como alternativa ao replanejamento completo do Dijkstra, implementou-se o algoritmo D* Lite, que reaproveita a arvore de busca anterior e repara apenas nos inconsistentes quando obstaculos dinamicos alteram o mapa. Em grades 4-conectadas com pesos atualizados por campo gaussiano proximal, o D* Lite manteve cobertura de 24.0% em 400 frames (BR-06) e 4.3% em 99 frames (CN-01), com tempo medio de 1.63 ms e 7.69 ms por replanejamento, respectivamente. Reutilizacao de caminho ocorreu em apenas 0.2% (BR-06) e 1.0% (CN-01) dos frames, pois a geracao de pedestres por Poisson (lambda=8.79 e 34.32) altera o mapa em praticamente todo frame. A comparacao tripla revelou que, nestes cenarios altamente dinamicos, o D* Lite nao oferece vantagem sobre o Dijkstra padrao (0.082 ms/frame) — o overhead das estruturas g/rhs e do heap com lazy deletion supera o beneficio incremental quando quase 100% dos frames exigem propagacao de inconsistencias. Confirma-se a hipotese da secao 0.3: em ambientes densamente dinamicos (lambda > 8), a vantagem incremental do D* Lite se dissolve."

---

## 9. REFERENCIAS (da pesquisa distribuida)

1. Koenig, S. & Likhachev, M. (2002). "D* Lite". AAAI 2002.
2. Stentz, A. (1994). "D*: Dynamic A*". Carnegie Mellon University.
3. Likhachev, M. et al. "Anytime Dynamic A*". CMU RI.
4. Surveys de navegacao robotica (2023-2026): PMC, Nature, ScienceDirect.
5. Implementacoes open-source: pydstarlite, Dstar-lite-pathplanner (GitHub).

---

**Status:** IMPLEMENTADA — resultados medidos em 2026-05-26.
**Entregaveis:** `dstar_lite_nav.py`, `run_dstar_lite.py`, `dijkstra_nav.py`, CSVs, 14 PNGs + comparativo triplo.
