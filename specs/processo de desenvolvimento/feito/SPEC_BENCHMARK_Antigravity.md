# SPEC DE BENCHMARK — Antigravity vs A* em Grades Dinâmicas

**Versão:** 1.0  
**Data:** 2026-05-14  
**Dependências:** `antigravity` (validado), `networkx`, `numpy`, `pandas`, `matplotlib`, `time`, `json`, `csv`  
**Objetivo:** Gerar dataset comparativo rigoroso (A* vs 3× Dijkstra) para inclusão no artigo.

---

## 1. Cenários de Teste (Sintéticos, Fieis ao Artigo)

Reproduzir **dois cenários** com parâmetros idênticos aos do artigo:

### Cenário BR-06
| Parâmetro | Valor |
|---|---|
| Dimensão física | 25 m × 10 m |
| Tamanho de célula | 0,40 m |
| Grade resultante | **63 × 25** = 1.575 células |
| Células navegáveis (estimada) | ~639 (obstáculos fixos removidos) |
| Quadros simulados | 400 |
| Pedestres médios/quadro | ~9 |
| Peso base (`wbase`) | 1.0 |
| Peso máximo gaussiano (`Pmax`) | 500.0 |
| Raio de influência | 1,5 células |
| Fator de amplificação proximal | 2,0 |
| Pedestrian radius | 0,45 m |

### Cenário CN-01
| Parâmetro | Valor |
|---|---|
| Dimensão física | 15 m × 20 m |
| Tamanho de célula | 0,40 m |
| Grade resultante | **38 × 50** = 1.900 células |
| Células navegáveis (estimada) | ~599 |
| Quadros simulados | 99 |
| Pedestres médios/quadro | ~34 |
| Mesmos parâmetros de peso acima |

### Geração de obstáculos fixos
- Usar `nx.grid_2d_graph(rows, cols)`.
- Remover ~5% dos nós aleatoriamente (exceto região de spawn e goal) para simular mobília/estruturas.
- Seed fixa (`random.seed(42)`) para reprodutibilidade entre algoritmos.

### Geração de pedestres dinâmicos
A cada quadro `t`:
1. Sortear `N_t` posições de pedestres (Poisson com λ = média do cenário).
2. Para cada célula `(r,c)` da grade, calcular ocupação:
   ```
   occ(r,c) = Σ Pmax * exp(-dist² / (2*σ²)) * proximal_factor
   ```
   onde `σ = raio_influencia_celulas`, `dist` = distância euclidiana em células até o pedestre.
3. Atualizar pesos das arestas:
   ```
   w(u,v) = wbase + max(occ(u), occ(v))
   ```
4. Se `w(u,v) > threshold_bloqueio` (sugerir 400.0), remover aresta de `Groute` para simular bloqueio dinâmico.

---

## 2. Algoritmos a Comparar

| ID | Algoritmo | Implementação | Nota |
|---|---|---|---|
| `Astar` | A* + Manhattan | `nx.astar_path(G, source, target, heuristic=manhattan, weight='weight')` | Baseline do artigo |
| `Dijk-Std` | Dijkstra Standard | `antigravity.dijkstra_path(..., variant='standard')` | Heap binária |
| `Dijk-Bi` | Dijkstra Bidirecional | `antigravity.dijkstra_path(..., variant='bidirectional')` | Duas frentes |
| `Dijk-Dial` | Dial's Algorithm | `antigravity.dijkstra_path(..., variant='dial')` | Buckets |

**Regra de ouro:** todos os algoritmos recebem **o mesmo grafo `Groute` no mesmo quadro**, com **os mesmos `source` e `target`**. Nenhum cache entre quadros.

---

## 3. Trajetória do Robô (Simulação Frame a Frame)

Reproduzir a lógica do artigo simplificada, mas suficiente para stressar o pathfinding:

```
robot = célula inicial (canto inferior-esquerdo, não-obstáculo)
goal  = célula final   (canto superior-direito, não-obstáculo)
strategy = cobertura total (visitar todas as células navegáveis)

para cada frame t em 1..T:
    1. Gerar pedestres do frame t
    2. Construir Groute a partir de Gstatic + bloqueios dinâmicos
    3. Escolher próximo_goal = célula não-limpada mais próxima (Manhattan) do robot
    4. Para cada algoritmo em [Astar, Dijk-Std, Dijk-Bi, Dijk-Dial]:
         a. path, metrics = algoritmo(Groute, robot, next_goal)
         b. Se path existe:
              - Executar 1 passo (mover robot para path[1])
              - Marcar célula como limpa
              - Registrar métricas do frame
         c. Se path é None:
              - Registrar falha (WAIT/REPLAN simulado)
              - Métricas: nodes_expanded=0, time_ms=0, path_cost=inf
    5. Avançar frame
```

**Importante:** como o robô se move entre frames, o `source` muda. Isso força **replanejamento real** a cada passo, igual ao artigo.

---

## 4. Métricas Coletadas por Algoritmo / por Frame

O `metrics` dict do `antigravity` já entrega:
- `nodes_expanded`
- `max_queue_size`
- `time_ms`
- `path_cost`
- `path_length`

O benchmark deve **acumular** em um `DataFrame` com colunas:

| Coluna | Tipo | Descrição |
|---|---|---|
| `frame` | int | Quadro atual (1..T) |
| `algorithm` | str | `Astar`, `Dijk-Std`, `Dijk-Bi`, `Dijk-Dial` |
| `scenario` | str | `BR-06` ou `CN-01` |
| `robot_r`, `robot_c` | int | Posição do robô no início do frame |
| `goal_r`, `goal_c` | int | Goal escolhido |
| `nodes_expanded` | int | Métrica da busca |
| `max_queue_size` | int | Pico de memória da fila |
| `time_ms` | float | Tempo de busca |
| `path_cost` | float | Custo total do caminho encontrado |
| `path_length` | int | Número de arestas |
| `success` | bool | True se path não-None |
| `pedestrians_count` | int | N de pedestres no frame |
| `max_occupancy` | float | Maior `occ` no caminho (se sucesso) |

---

## 5. Agregações Finais (Para Tabelas do Artigo)

Ao final da simulação, computar por cenário e por algoritmo:

1. **Média de nós expandidos por busca bem-sucedida**
2. **Máximo de nós expandidos em uma única busca** (pior caso)
3. **Média do `max_queue_size`**
4. **Tempo total de execução** (soma de `time_ms`)
5. **Tempo médio por busca**
6. **Taxa de sucesso** (buscas que retornaram path / total de buscas)
7. **Passos totais até cobertura 100%** (ou até `MAX_STEPS`)
8. **Número de falhas** (path=None) — proxy para WAIT/REPLAN

Exportar dois arquivos:
- `benchmark_raw.csv` — linha a linha (frame × algoritmo)
- `benchmark_summary.csv` — agregado por cenário/algoritmo

---

## 6. Visualizações (Para o Artigo)

Gerar 4 figuras em alta resolução (300 DPI, formato PNG):

### Figura 1: Nós Expandidos — Distribuição por Algoritmo
- Boxplot ou violin plot de `nodes_expanded` por algoritmo, facetado por cenário.
- Destacar mediana e outliers.

### Figura 2: Tempo de Execução Acumulado
- Gráfico de linha: eixo X = frame, eixo Y = tempo acumulado (ms), 4 linhas (uma por algoritmo).
- Facetar BR-06 vs CN-01.

### Figura 3: Memória da Fila (Max Queue Size)
- Scatter plot ou linha: `max_queue_size` vs `frame`, colorido por algoritmo.
- Mostrar que Dial tende a ter picos diferentes de heap.

### Figura 4: Trade-off Tempo vs Nós Expandidos
- Scatter: eixo X = média de nós expandidos, eixo Y = tempo médio por busca.
- Cada ponto é um algoritmo. Mostrar que menos nós não sempre = menos tempo (overhead de estrutura de dados).

---

## 7. Parâmetros de Execução

```python
# Configuração global do benchmark
CONFIG = {
    "seed": 42,
    "max_steps": 15000,        # Igual ao artigo
    "max_wait_replans": 8,     # Igual ao artigo
    "cell_size_m": 0.40,
    "wbase": 1.0,
    "pmax": 500.0,
    "influence_radius_cells": 1.5,
    "proximal_factor": 2.0,
    "pedestrian_radius_m": 0.45,
    "block_threshold": 400.0,  # aresta removida se peso > threshold
    "scenarios": {
        "BR-06": {"dims_m": (25, 10), "frames": 400, "lambda_ped": 8.79},
        "CN-01": {"dims_m": (15, 20), "frames": 99,  "lambda_ped": 34.32},
    }
}
```

---

## 8. Critérios de Aceitação do Benchmark

- [ ] BR-06 e CN-01 geram grades com número de células navegáveis compatível com o artigo (±10%).
- [ ] A* encontra caminhos com custo idêntico ao Dijkstra-Std em >95% das buscas (prova de corretude do antigravity).
- [ ] Dial não sofre overflow de bucket em nenhum cenário.
- [ ] Todos os 4 algoritmos completam a simulação sem exceção não-tratada.
- [ ] Arquivos CSV e PNG são gerados em `/output/benchmark/`.
- [ ] `benchmark_summary.csv` contém pelo menos 8 linhas (2 cenários × 4 algoritmos).

---

## 9. Entregáveis

1. `run_benchmark.py` — script principal de execução
2. `benchmark_raw.csv` — dados brutos por frame
3. `benchmark_summary.csv` — agregados por cenário/algoritmo
4. `fig_nodes_expanded.png`
5. `fig_time_accumulated.png`
6. `fig_max_queue.png`
7. `fig_tradeoff.png`
8. `README_BENCHMARK.md` — instruções de replicação

---

## 10. Nota para o Artigo (Rascunho de Parágrafo)

> *"Para avaliar o impacto da escolha algorítmica no pipeline dinâmico, reproduzimos os cenários BR-06 e CN-01 com grades sintéticas parametrizadas (célula de 0,40 m, penalidade gaussiana Pmax = 500, raio de influência 1,5 células). Comparando A* com heurística de Manhattan contra três configurações de Dijkstra — heap binária, bidirecional e Dial's Algorithm — medimos nós expandidos, pico de memória da fila de prioridade e tempo de CPU por busca. O Dijkstra bidirecional reduziu X% dos nós expandidos em relação ao A*, enquanto Dial's Algorithm apresentou o menor tempo médio de execução em grades de peso limitado, confirmando que a estrutura de dados da fila de prioridade é tão crítica quanto a heurística em ambientes de ocupação dinâmica."*

---

**Status:** Spec pronta para implementação.  
**Próximo passo:** Codificar `run_benchmark.py` e executar contra o módulo `antigravity`.
