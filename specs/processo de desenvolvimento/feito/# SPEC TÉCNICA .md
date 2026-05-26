
# SPEC TÉCNICA — Módulo Antigravity: Dijkstra para Navegação em Grade Dinâmica

**Versão:** 1.0  
**Escopo:** Implementar 3 variantes de Dijkstra compatíveis com o pipeline existente (A* + Manhattan), mantendo o mesmo contrato de entrada/saída.  
**Objetivo:** Gerar dados comparativos de nós expandidos, memória de fila e tempo de execução para inclusão no artigo.

---

## 1. Contrato de Interface (Obrigatório)

Todas as variantes devem expor esta assinatura única:

```python
def dijkstra_path(
    G: nx.Graph,           # GrRoute ou Gstatic do pipeline
    source: tuple[int, int],
    target: tuple[int, int],
    variant: str,          # "standard" | "bidirectional" | "dial"
    weight: str = "weight"
) -> tuple[list[tuple[int, int]], dict]:
    """
    Retorna:
      - path: lista de nós do source ao target (inclusivo)
      - metrics: dict com {
          "nodes_expanded": int,
          "max_queue_size": int,
          "time_ms": float,
          "variant": str
        }
    Levanta NetworkXNoPath (ou retorna None) se desconectado.
    """
```

**Regra de integração:** no loop de simulação, substituir apenas a chamada interna:

```python
# Antes
path = nx.astar_path(Groute, robot, next_goal, heuristic=manhattan, weight="weight")

# Depois
path, metrics = dijkstra_path(Groute, robot, next_goal, variant="...", weight="weight")
```

---

## 2. Variantes

### 2.1 Dijkstra Standard (Heap Binária)

**Algoritmo:** clássico com `heapq`.  
**Por que:** baseline honesto. Elimina o overhead da heurística Manhattan em cenários de peso altamente dinâmico.

**Pseudocódigo:**

```
inicializar dist[v] = ∞ para todo v
inicializar pred[v] = None
dist[source] = 0
heap = [(0, source)]
visited = set()
nodes_expanded = 0
max_queue = 0

enquanto heap não vazio:
    max_queue = max(max_queue, len(heap))
    d, u = heappop(heap)
    se u em visited: continue
    visited.add(u)
    nodes_expanded += 1
    se u == target: break
    
    para cada vizinho v de u em G:
        se v em visited: continue
        w = peso da aresta (u,v)
        se dist[u] + w < dist[v]:
            dist[v] = dist[u] + w
            pred[v] = u
            heappush(heap, (dist[v], v))

reconstruir path via pred a partir do target
se target não alcançado: retornar None
```

**Complexidade esperada:** `O(E + V log V)`  
**Uso de memória:** `O(V)` para distâncias + `O(V)` para heap no pior caso.

---

### 2.2 Dijkstra Bidirecional

**Algoritmo:** duas buscas simultâneas — forward (source →) e backward (target →).  
**Por que:** em grades 4-conectadas, a fronteira de busca cresce em diamante. Bidirecional reduz a área explorada de `~n²` para `~2·(n/2)²`.

**Pseudocódigo:**

```
dist_f, dist_b = dicts com source/target = 0
pred_f, pred_b = dicts
heap_f = [(0, source)], heap_b = [(0, target)]
visited_f, visited_b = sets
nodes_expanded = 0
max_queue = 0
best_path_len = ∞
meeting_node = None

enquanto ambos os heaps não vazios:
    # Expandir a frente mais promissora (menor topo)
    escolher heap_x (f ou b) com menor topo
    max_queue = max(max_queue, len(heap_f) + len(heap_b))
    d, u = heappop(heap_x)
    se u em visited_x: continue
    visited_x.add(u)
    nodes_expanded += 1
    
    # Checar encontro
    se u em visited_oposto:
        path_len = dist_f.get(u,∞) + dist_b.get(u,∞)
        se path_len < best_path_len:
            best_path_len = path_len
            meeting_node = u
    
    # Parada antecipada: se d > best_path_len, podemos parar (opcional, cuidado)
    
    para cada vizinho v de u:
        relaxar aresta em dist_x / pred_x
        heappush se melhorado

reconstruir path: pred_f (source → meeting) + reverse(pred_b (target → meeting))
```

**Edge case:** se `source == target`, retornar `[source]` imediatamente.  
**Complexidade esperada:** `O(E + V log V)` teórica, mas `~½ nós expandidos` na prática para grades.  
**Uso de memória:** `~2×` estruturas do padrão, mas metade do tempo.

---

### 2.3 Dial's Algorithm (Bucket Dijkstra)

**Algoritmo:** substitui heap por **array de buckets** (lista de listas/deques).  
**Por que:** os pesos das arestas no artigo são limitados (`wbase=1.0`, `Pmax=500.0`, portanto `w_max ≈ 501.0`). Em grades pequenas (≤ 1000 nós), Dial elimina o fator `log V`.

**Discretização obrigatória:**

```python
SCALE = 100  # 2 casas decimais de precisão
def discretize(w: float) -> int:
    return int(round(w * SCALE))
```

**Pseudocódigo:**

```
max_w = 501.0 * SCALE  # 50100
buckets = [deque() for _ in range(max_w + 1)]
# Ou usar dict de deques para economia, mas array é mais rápido

dist[v] = ∞
dist[source] = 0
buckets[0].append(source)
curr = 0
visited = set()
nodes_expanded = 0
max_queue = 0  # soma dos len(buckets[i])

enquanto curr <= max_w * V:  # limite superior de distância
    # Avançar curr até bucket não vazio
    enquanto curr <= limite e buckets[curr] vazio:
        curr += 1
    se curr > limite: break
    
    max_queue = max(max_queue, sum(len(b) for b in buckets))  # amostragem periódica
    
    u = buckets[curr].popleft()
    se u em visited: continue
    visited.add(u)
    nodes_expanded += 1
    se u == target: break
    
    para cada vizinho v:
        w = discretize(peso(u,v))
        new_dist = curr + w
        se new_dist < dist[v]:
            se dist[v] != ∞:
                remover v do bucket antigo (O(1) se usarmos set auxiliar por bucket? Não, simplesmente deixar stale entries e ignorar no pop)
            dist[v] = new_dist
            pred[v] = u
            buckets[new_dist].append(v)

# Stale entries: quando puxamos um nó do bucket e dist[u] != curr, ignoramos.
```

**Otimização de memória:** em vez de array fixo de 50k, usar `defaultdict(deque)` e manter `curr` como índice real. A memória vira `O(E + C)` onde `C` é o número de buckets distintos ocupados.

**Complexidade esperada:** `O(E + C·V)` onde `C` é peso máximo discretizado.  
**Trade-off:** consome mais memória que heap para grades muito grandes, mas para `V < 1000` e `C ≈ 500`, é **dominante em tempo**.

---

## 3. Instrumentação de Métricas (Crítico para o Artigo)

Todas as variantes devem retornar o mesmo `metrics` dict:

| Chave | Definição | Como medir |
|---|---|---|
| `nodes_expanded` | Nós removidos da estrutura de prioridade e processados (vizinhos iterados) | Incrementar após `visited.add(u)` |
| `max_queue_size` | Maior tamanho atingido pela fila de prioridade durante a busca | `max(len(heap))` ou `max(sum(len(buckets)))` |
| `time_ms` | Tempo de CPU gasto apenas no algoritmo de busca | `time.perf_counter()` ao redor do loop principal |
| `path_cost` | Soma dos pesos do caminho retornado | Somar arestas do path |
| `path_length` | Número de arestas no caminho | `len(path) - 1` |

**Regra:** não contabilizar tempo de reconstrução do path nem de cópia do grafo. Apenas a busca.

---

## 4. Compatibilidade com o Pipeline Existente

### 4.1 Grafo de entrada
- Recebe `Groute` (subgrafo temporário com bloqueios dinâmicos) ou `Gstatic`.
- Nós são tuplas `(r, c)`. Arestas têm atributo `weight` (float).
- O algoritmo não deve modificar `G`. Trabalhar com estruturas auxiliares (`dist`, `pred`).

### 4.2 Falha e recuperação
- Se `target` não é alcançável (grafo desconectado por pedestres), retornar `None` e `metrics` parciais.
- O loop de simulação existente já trata isso: aciona `WAIT` ou `BACKTRACK` via `calculatebacktrackroute`.

### 4.3 Pesos dinâmicos
- A cada frame, `Groute` pode ter pesos diferentes. Não há cache persistente entre frames (a menos que futuro trabalho implemente incrementalidade).
- O Dijkstra roda **do zero** a cada replanejamento, igual ao A* atual. Isso garante comparação justa.

---

## 5. Critérios de Aceitação (Definition of Done)

- [ ] As 3 variantes passam em teste com grafo mínimo 3×3, peso uniforme 1.0, source=(0,0), target=(2,2). Caminho esperado: 4 arestas.
- [ ] Teste de desconexão: grafo onde target é isolado. Retorna `None`, `nodes_expanded` reflete toda a componente explorada.
- [ ] Teste de pesos altos: grade 5×5, aresta central com peso 500.0. Dial deve encontrar caminho contornando sem overflow.
- [ ] Integração shadow: rodar 10 frames do CN-01 com Dijkstra Standard e comparar `path` com A* — devem ser idênticos (determinismo de custo mínimo).
- [ ] Métricas exportáveis: o `metrics` dict de cada frame deve ser acumulado em um `DataFrame` com as mesmas colunas do `MetricsCollector` existente, mais uma coluna `algorithm` (`A*`, `Dijkstra-Std`, `Dijkstra-Bi`, `Dijkstra-Dial`).

---

## 6. Estrutura de Arquivos Sugerida

```
antigravity/
├── __init__.py
├── dijkstra_standard.py      # Heap binária
├── dijkstra_bidirectional.py # Duas fronteiras
├── dijkstra_dial.py          # Buckets
├── metrics.py                # Wrapper de instrumentação
└── tests/
    ├── test_minimal.py
    ├── test_disconnected.py
    └── test_integration.py
```

---

## 7. Notas para o Artigo (Copiar/Colar depois)

> *"O algoritmo de Dijkstra foi implementado em três configurações: (i) fila de prioridade com heap binária, como baseline teórico O(V+E log V); (ii) busca bidirecional, que reduz a fronteira de expansão em grades 4-conectadas; e (iii) Dial's Algorithm, que explora a limitação superior dos pesos dinâmicos (wmax ≈ 501) para atingir complexidade linear O(E+C·V) em estrutura de buckets. A comparação com A* mantém o mesmo grafo dinâmico Groute e o mesmo critério de replanejamento frame a frame."*

---

## Próximo passo

Essa spec está pronta para ser entregue a qualquer coder (Sonnet, Gemini, Claude, ou você mesmo). 

