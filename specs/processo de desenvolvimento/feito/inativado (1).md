# Código Inativado — Variantes de Busca Removidas da SPEC Atual

> **Arquivo de referência.** Contém specs e pseudocódigo de algoritmos que foram removidos do escopo da SPEC de Correção Dijkstra v2.1.
> Mantido apenas para consulta futura ou reativação se necessário.

---

## 1. A* (Astar)

### Motivo da remoção
- Não era o foco do benchmark solicitado pelo chefe.
- Heurística de Manhattan conflita com pesos dinâmicos de pedestres (não admissível quando pesos mudam).
- Dados do benchmark_raw.csv mostram comportamento idêntico ao Dijk-Std em termos de falhas (mesmos frames falhos).

### Pseudocódigo original
```python
def astar(graph, source, target, heuristic=manhattan):
    dist = {v: inf for v in graph}
    pred = {v: None for v in graph}
    dist[source] = 0
    heap = [(heuristic(source, target), 0, source)]  # (f, g, node)
    visited = set()

    while heap:
        f, g, u = heappop(heap)
        if u in visited:
            continue
        visited.add(u)
        if u == target:
            break
        for v, w in graph.neighbors(u):
            if v in visited:
                continue
            new_g = g + w
            new_f = new_g + heuristic(v, target)
            if new_g < dist[v]:
                dist[v] = new_g
                pred[v] = u
                heappush(heap, (new_f, new_g, v))

    return reconstruct_path(pred, source, target), nodes_expanded
```

### Problemas identificados nos dados
- Mesmas falhas que Dijk-Std (frames 103, 104 em BR-06; frames 16-18, 50-52 em CN-01).
- `nodes_expanded` e `max_queue_size` vazios em muitos registros (colunas em branco no CSV).
- path_cost = `inf` em falhas, mas sem métrica de tempo consistente.

---

## 2. Dijkstra Bidirecional (Dijk-Bi)

### Motivo da remoção
- Implementação quebrada: tempos de busca maiores que Dijk-Std em muitos frames.
- Expande mais nós que Dijk-Std em cenários com alta densidade de pedestres (CN-01).
- Lógica de interseção das duas frentes não estava otimizada para pesos dinâmicos.

### Pseudocódigo original
```python
def dijkstra_bidirectional(graph, source, target):
    # Frente forward
    dist_f = {v: inf for v in graph}
    pred_f = {v: None for v in graph}
    dist_f[source] = 0
    heap_f = [(0, source)]
    visited_f = set()

    # Frente backward
    dist_b = {v: inf for v in graph}
    pred_b = {v: None for v in graph}
    dist_b[target] = 0
    heap_b = [(0, target)]
    visited_b = set()

    best_path = None
    best_cost = inf

    while heap_f or heap_b:
        # Expandir frente forward
        if heap_f:
            d, u = heappop(heap_f)
            if u in visited_f:
                continue
            visited_f.add(u)
            if u in visited_b and dist_f[u] + dist_b[u] < best_cost:
                best_cost = dist_f[u] + dist_b[u]
                best_path = merge_paths(pred_f, pred_b, u)
            for v, w in graph.neighbors(u):
                if v not in visited_f and dist_f[u] + w < dist_f[v]:
                    dist_f[v] = dist_f[u] + w
                    pred_f[v] = u
                    heappush(heap_f, (dist_f[v], v))

        # Expandir frente backward
        if heap_b:
            d, u = heappop(heap_b)
            if u in visited_b:
                continue
            visited_b.add(u)
            if u in visited_f and dist_f[u] + dist_b[u] < best_cost:
                best_cost = dist_f[u] + dist_b[u]
                best_path = merge_paths(pred_f, pred_b, u)
            for v, w in graph.neighbors(u):  # PROBLEMA: precisa de grafo reverso
                if v not in visited_b and dist_b[u] + w < dist_b[v]:
                    dist_b[v] = dist_b[u] + w
                    pred_b[v] = u
                    heappush(heap_b, (dist_b[v], v))

    return best_path, nodes_expanded
```

### Problemas identificados nos dados
- **BR-06**: 32 falhas (vs 37 do Dijk-Std), mas tempos maiores em frames críticos.
- **CN-01**: 26 falhas, mesmo número do Dijk-Std.
- Nós expandidos frequentemente maiores que Dijk-Std (ex: frame 17 BR-06: 822 vs 3).
- `merge_paths` não estava implementado corretamente — podia retornar caminho inválido.

---

## 3. Dijkstra Dial (Dijk-Dial) — Bucket-Based

### Motivo da remoção
- **CRÍTICO: Loop infinito / travamento em falhas.**
- Tempos de ~5000ms em frames onde o grafo está desconexo (goal inalcançável).
- Bucket dimensionado incorretamente para pesos de aresta que variam de 1.0 a 1000+.
- C_max mal calculado → buckets circulares nunca esvaziam.

### Pseudocódigo original (COM BUGS)
```python
def dijkstra_dial(graph, source, target, max_weight):
    # BUG 1: max_weight não era calculado dinamicamente por frame
    # BUG 2: C_max = max_weight * num_nodes — estoura memória
    # BUG 3: Buckets circulares sem verificação de esvaziamento

    C = int(max_weight * len(graph))  # PROBLEMA: pesos são float!
    buckets = [[] for _ in range(C + 1)]
    dist = {v: inf for v in graph}
    pred = {v: None for v in graph}
    dist[source] = 0
    buckets[0].append(source)

    current = 0
    nodes_expanded = 0

    while True:
        # PROBLEMA: loop infinito se todos os buckets vazios e target não alcançado
        while current <= C and not buckets[current]:
            current += 1

        if current > C:
            break  # Deveria retornar None, mas às vezes não chegava aqui

        u = buckets[current].pop()
        if dist[u] < current:
            continue
        nodes_expanded += 1

        if u == target:
            break

        for v, w in graph.neighbors(u):
            new_dist = dist[u] + w
            if new_dist < dist[v]:
                old_dist = dist[v]
                dist[v] = new_dist
                pred[v] = u
                bucket_idx = int(new_dist) % (C + 1)  # PROBLEMA: wrap-around errado
                buckets[bucket_idx].append(v)

        # PROBLEMA: current não volta para trás — pode pular nós com distância menor

    return reconstruct_path(pred, source, target), nodes_expanded
```

### Evidências do travamento (benchmark_raw.csv)
| Frame | Cenário | Tempo (ms) | nodes_expanded | max_queue | Observação |
|-------|---------|-----------|----------------|-----------|------------|
| 3 | BR-06 | 4922.77 | 1.0 | 1.0 | Loop infinito, 1 nó expandido |
| 47 | BR-06 | 4910.12 | 1.0 | 1.0 | Loop infinito, 1 nó expandido |
| 56 | BR-06 | 4874.52 | 1.0 | 1.0 | Loop infinito, 1 nó expandido |
| 66 | BR-06 | 5201.26 | 1397.0 | 103.0 | Loop após expandir muitos nós |
| 89 | BR-06 | 4856.58 | 1.0 | 1.0 | Loop infinito, 1 nó expandido |
| 103 | BR-06 | 4961.29 | 1.0 | 1.0 | Loop infinito |
| 104 | BR-06 | 4861.82 | 1.0 | 1.0 | Loop infinito |
| 122 | BR-06 | 5498.16 | 1328.0 | 90.0 | Loop após expansão |
| 163 | BR-06 | 4823.26 | 1.0 | 1.0 | Loop infinito |
| 241 | BR-06 | 5332.99 | 1359.0 | 120.0 | Loop após expansão |
| 250 | BR-06 | 4878.79 | 1.0 | 1.0 | Loop infinito |
| 270 | BR-06 | 4861.01 | 1.0 | 1.0 | Loop infinito |
| 278 | BR-06 | 4930.90 | 1.0 | 1.0 | Loop infinito |
| 295 | BR-06 | 4853.83 | 1.0 | 1.0 | Loop infinito |
| 304 | BR-06 | 4934.55 | 1.0 | 1.0 | Loop infinito |
| 307 | BR-06 | 5015.99 | 1.0 | 1.0 | Loop infinito |
| 311 | BR-06 | 4933.89 | 1.0 | 1.0 | Loop infinito |
| 332 | BR-06 | 5324.17 | 1383.0 | 116.0 | Loop após expansão |
| 333 | BR-06 | 4876.98 | 1.0 | 1.0 | Loop infinito |
| 348 | BR-06 | 4871.88 | 1.0 | 1.0 | Loop infinito |
| 350 | BR-06 | 4802.14 | 1.0 | 1.0 | Loop infinito |
| 356 | BR-06 | 4962.03 | 1.0 | 1.0 | Loop infinito |
| 382 | BR-06 | 5118.04 | 1364.0 | 111.0 | Loop após expansão |
| 384 | BR-06 | 5295.21 | 1298.0 | 128.0 | Loop após expansão |
| 390 | BR-06 | 5332.99 | 1404.0 | 133.0 | Loop após expansão |

**Padrão:** Sempre que `success=False` e `nodes_expanded` é baixo (1 ou poucos), o tempo é ~5000ms — indica loop infinito na detecção de esvaziamento de buckets.

---

## 4. Estruturas de Dados Removidas

### Grafo com NetworkX
```python
# REMOVIDO — proibido pela spec original, mas estava em testes
import networkx as nx
# G = nx.Graph()  # NÃO USAR no core
```

### Matriz de pesos numpy (alternativa ao dict de adjacência)
```python
# REMOVIDO — complexidade desnecessária para grids pequenos (<2000 células)
weights = np.full((rows, cols, 4), BASE_WEIGHT)  # N, S, E, W
```

---

## 5. Métricas Coletadas que Não Serão Mais Geradas

| Métrica | Motivo da remoção |
|---------|-------------------|
| `algorithm` | Só existe Dijk-Std agora |
| `max_queue_size` (para Dijk-Dial) | Não aplicável (usa buckets, não heap) |
| `path_cost = inf` | Substituído por path=None com path_cost=0.0 |
| Tentativas múltiplas por frame | Reamostragem removida — pedestres são determinísticos |

---

## 6. Como Reativar (se necessário)

1. Copiar o pseudocódigo deste arquivo para a nova spec.
2. Corrigir os bugs identificados acima.
3. Adicionar de volta à pipeline de benchmark.
4. **Para Dijk-Dial:** reimplementar do zero com buckets de tamanho fixo e verificação correta de esvaziamento.

---

*Gerado em: 2026-05-18*
*Versão da spec ativa: SPEC_Dijkstra_Correcao_v2.1.md*
