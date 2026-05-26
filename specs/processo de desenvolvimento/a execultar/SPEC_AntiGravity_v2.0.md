# SPEC DE MELHORIA — AntiGravity v2.0

**Versao:** 2.0 (Melhoria)
**Data:** 2026-05-26
**Base:** v1.1 (ja implementada e auditada)
**Objetivo:** Corrigir 3 problemas conhecidos do AntiGravity e melhorar cobertura/eficiencia.

---

## 1. PROBLEMAS IDENTIFICADOS NA v1.1

### 1.1 Minimos locais (fallback aleatorio funciona, mas eh lento)
- **Sintoma:** Robo fica 2-3 frames parado antes do fallback escapar
- **Causa:** Fallback totalmente aleatorio, sem memoria de direcoes ja tentadas
- **Impacto:** ~5% dos frames sao WAIT desnecessario

### 1.2 Corredores estreitos (2 pedestres dos lados)
- **Sintoma:** Robo nunca consegue passar entre 2 pedestres proximos
- **Causa:** Forca repulsiva dos 2 lados cancela a forca atrativa
- **Impacto:** Bloqueio permanente em corredores de 1 celula

### 1.3 Oscilacao em torno do goal
- **Sintoma:** Robo circunda o goal sem parar
- **Causa:** Forca repulsiva de pedestres proximos ao goal empurra o robo para longe
- **Impacto:** Nunca limpa a celula do goal

---

## 2. MELHORIAS v2.0

### 2.1 Fallback com memoria (anti-ciclo)
```python
# NOVO: historico de posicoes recentes
visited_history = deque(maxlen=10)  # ultimas 10 posicoes

def smart_fallback(robot, occ_grid, block_threshold=400.0):
    # Em vez de aleatorio puro, escolher direcao que:
    # 1. Nao esta no historico recente
    # 2. Aumenta distancia do centro de gravidade dos obstaculos
    # 3. Mantem componente atrativa ao goal

    directions = [(0,1), (0,-1), (1,0), (-1,0)]
    scores = []

    for dr, dc in directions:
        new_r, new_c = robot[0] + dr, robot[1] + dc
        if not is_valid_move(robot, dr, dc, ...):
            continue

        score = 0
        # Penaliza se ja visitou recentemente
        if (new_r, new_c) in visited_history:
            score -= 50

        # Bonifica se aumenta distancia dos obstaculos
        occ_value = occ_grid[new_r, new_c]
        score -= occ_value  # menor ocupacao = melhor

        # Bonifica se aproxima do goal
        goal_dist = abs(new_r - goal[0]) + abs(new_c - goal[1])
        score -= goal_dist  # menor distancia = melhor

        scores.append((score, dr, dc))

    if scores:
        return max(scores, key=lambda x: x[0])[1:]
    return (0, 0)
```

### 2.2 Modo "corredor" (reduzir K_REPULSIVO quando bloqueado dos 2 lados)
```python
# NOVO: detectar quando esta em corredor estreito
def detect_corridor(robot, occ_grid, block_threshold):
    r, c = robot
    left_blocked = (c > 0 and occ_grid[r, c-1] > block_threshold)
    right_blocked = (c < cols-1 and occ_grid[r, c+1] > block_threshold)
    up_blocked = (r > 0 and occ_grid[r-1, c] > block_threshold)
    down_blocked = (r < rows-1 and occ_grid[r+1, c] > block_threshold)

    # Se bloqueado em ambos os lados de um eixo
    if (left_blocked and right_blocked) or (up_blocked and down_blocked):
        return True  # esta em corredor
    return False

# Na forca repulsiva:
if detect_corridor(robot, occ_grid, BLOCK_THRESHOLD):
    k_rep_efetivo = K_REPULSIVE * 0.3  # reduz repulsao lateral
else:
    k_rep_efetivo = K_REPULSIVE
```

### 2.3 Desativar repulsao proxima ao goal
```python
# NOVO: zona de exclusao de repulsao
def compute_resultant_force_v2(robot, goal, occ_grid, ...):
    dist_to_goal = heuristic(robot, goal)

    if dist_to_goal <= GOAL_THRESHOLD * 3:
        # Proximo do goal: reduzir repulsao, aumentar atracao
        k_att_efetivo = K_ATTRACTIVE * 3.0
        k_rep_efetivo = K_REPULSIVE * 0.1
        # Ignorar pedestres dentro da zona do goal
    else:
        k_att_efetivo = K_ATTRACTIVE
        k_rep_efetivo = K_REPULSIVE

    F_att = attractive_force(robot, goal, k_att_efetivo)
    F_rep = repulsive_force(robot, occ_grid, k_rep_efetivo, ...)
    return F_att + F_rep
```

---

## 3. PARAMETROS AJUSTAVEIS POR CENARIO

```python
# v2.0: parametros diferenciados por densidade de pedestres

AG_CONFIG = {
    "BR-06": {  # lambda = 8.79 (baixa densidade)
        "K_ATTRACTIVE": 1.0,
        "K_REPULSIVE": 500.0,
        "D0": 1.5,
        "GOAL_THRESHOLD": 1.0,
        "CORRIDOR_MODE": True,
        "GOAL_ZONE_RADIUS": 3,
    },
    "CN-01": {  # lambda = 34.32 (alta densidade)
        "K_ATTRACTIVE": 2.0,      # mais atracao (menos dispersao)
        "K_REPULSIVE": 300.0,     # menos repulsao (evita bloqueio)
        "D0": 2.0,                # raio de influencia maior
        "GOAL_THRESHOLD": 2.0,    # zona de chegada maior
        "CORRIDOR_MODE": True,
        "GOAL_ZONE_RADIUS": 5,    # desativa repulsao mais longe
    }
}
```

---

## 4. NOVAS METRICAS

| Metrica | Descricao | Alvo BR-06 | Alvo CN-01 |
|---------|-----------|------------|------------|
| coverage_percent | (mesmo) | > 25% | > 5% |
| wait_count | Frames com WAIT | < 20 | < 10 |
| minima_locais_resolvidos | Fallback bem-sucedido | > 80% | > 70% |
| corredores_bloqueados | Vezes que ficou preso em corredor | < 5 | < 10 |
| goal_oscillation_count | Vezes que oscilou no goal | < 3 | < 5 |
| tempo_ate_goal | Frames para limpar celula do goal | < 50 | < 20 |

---

## 5. CRITERIOS DE ACEITACAO

- [ ] Cobertura BR-06 > 25% (vs 22.8% da v1.1)
- [ ] Cobertura CN-01 > 5% (vs 3.5% da v1.1)
- [ ] WAIT count BR-06 < 20 (vs ~40 da v1.1)
- [ ] Minimos locais resolvidos > 80%
- [ ] Corredores bloqueados < 5
- [ ] Goal oscillation < 3
- [ ] Tempo medio < 0.5ms (mantido)

---

## 6. ENTREGAVEIS

- `antigravity_nav_v2.py` — nova implementacao
- `run_antigravity_v2.py` — runner com parametros por cenario
- CSVs em `output/antigravity_v2/`
- Graficos comparativos: v1.1 vs v2.0 lado a lado

---

**Status:** Contrato pronto para implementacao futura.
**Prioridade:** Media — D* Lite e o foco atual do artigo.
