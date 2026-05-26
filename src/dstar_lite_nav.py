"""
dstar_lite_nav.py
=================
Implementacao do algoritmo D* Lite (Koenig & Likhachev, 2002) para navegacao
em grade dinamica 4-conectada com campo de ocupacao gaussiano.

Referencia:
    Koenig, S. & Likhachev, M. (2002). "D* Lite". AAAI 2002.

Integracao:
    Compativel com nav_utils.py (generate_frame_pedestrians, compute_occupancy,
    create_base_grid, get_navigable_cells).

Notas de design:
    - Heap OPEN usa lazy deletion (heapq nao suporta decrease-key).
    - Busca reversa: raiz no goal, folhas proximas ao start (robo).
    - km acumula distancia heuristica percorrida pelo robo para manter
      chaves validas sem reordenar o heap inteiro.
    - Goals dinamicos: quando o goal muda, o planner e reinicializado
      (perde vantagem incremental nesse frame, mas e documentado como
      limitacao real do algoritmo em cenarios de cobertura).
"""

import heapq
import math
import numpy as np
from collections import defaultdict

# ---------------------------------------------------------------------------
# Constantes herdadas do projeto
# ---------------------------------------------------------------------------
CELL_SIZE         = 0.40
PMAX              = 500.0
INFLUENCE_RADIUS  = 1.5
PROXIMAL_FACTOR   = 2.0
BLOCK_THRESHOLD   = 400.0
MAX_STEPS         = 15000
SEED              = 42

INF = float('inf')

# ---------------------------------------------------------------------------
# Heuristica
# ---------------------------------------------------------------------------
def heuristic(s1, s2):
    """Manhattan distance — admissivel para grade 4-conectada."""
    return abs(s1[0] - s2[0]) + abs(s1[1] - s2[1])


# ---------------------------------------------------------------------------
# Classe principal: D* Lite
# ---------------------------------------------------------------------------
class DStarLite:
    """
    D* Lite para grade 2-D com campo de ocupacao continuo.

    A busca e REVERSA: parte do goal em direcao ao robo (start).
    - g[s]   : custo acumulado real (goal -> s)
    - rhs[s] : custo lookahead de 1-passo (min dos sucessores + custo)
    - s eh consistente se g[s] == rhs[s]

    Parametros
    ----------
    occ_grid : np.ndarray (rows, cols), float
        Grade de ocupacao inicial.
    start : tuple (r, c)
        Posicao inicial do robo.
    goal : tuple (r, c)
        Destino (fixo durante uma rodada de replanejamento).
    block_threshold : float
        Ocupacao acima deste valor = celula bloqueada (custo inf).
    """

    def __init__(self, occ_grid, start, goal, block_threshold=BLOCK_THRESHOLD):
        self.rows, self.cols = occ_grid.shape
        self.occ_grid        = occ_grid.copy()
        self.start           = start   # posicao atual do robo
        self.goal            = goal
        self.block_threshold = block_threshold

        # Tabelas g e rhs — defaultdict retorna INF para nos nao visitados
        self.g   = defaultdict(lambda: INF)
        self.rhs = defaultdict(lambda: INF)

        # km: acumulador de distancia percorrida pelo robo
        self.km = 0.0

        # s_last: ultima posicao do robo (usada para atualizar km)
        self.s_last = start

        # OPEN heap: entradas (k1, k2, node)
        # Lazy deletion: open_set rastreia nos VALIDOS no heap
        self._open_heap = []
        self._open_set  = {}  # node -> (k1, k2) mais recente inserida

        # Inicializar: goal tem rhs = 0
        self.rhs[self.goal] = 0.0
        key = self._calculate_key(self.goal)
        heapq.heappush(self._open_heap, (key[0], key[1], self.goal))
        self._open_set[self.goal] = key

    # ------------------------------------------------------------------
    # Metodos internos
    # ------------------------------------------------------------------

    def _calculate_key(self, s):
        """Chave lexicografica (k1, k2) do no s."""
        g_rhs_min = min(self.g[s], self.rhs[s])
        k1 = g_rhs_min + heuristic(self.start, s) + self.km
        k2 = g_rhs_min
        return (k1, k2)

    def _neighbors(self, s):
        """Vizinhos 4-conectados validos (dentro dos limites da grade)."""
        r, c = s
        for dr, dc in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
            nr, nc = r + dr, c + dc
            if 0 <= nr < self.rows and 0 <= nc < self.cols:
                yield (nr, nc)

    def _cost(self, u, v):
        """
        Custo de transicao u -> v.
        INF se v estiver bloqueada; 1.0 caso contrario.
        (Busca reversa: predecessores de v sao seus vizinhos validos.)
        """
        if self.occ_grid[v[0], v[1]] > self.block_threshold:
            return INF
        return 1.0

    def _insert_or_update(self, s, key):
        """Insere s no heap com a chave dada (lazy: entradas obsoletas sao ignoradas)."""
        self._open_set[s] = key
        heapq.heappush(self._open_heap, (key[0], key[1], s))

    def _remove_from_open(self, s):
        """Marca s como removido do OPEN (lazy deletion)."""
        if s in self._open_set:
            del self._open_set[s]

    def _top_key(self):
        """Retorna a menor chave valida do heap, ou (INF, INF) se vazio."""
        while self._open_heap:
            k1, k2, s = self._open_heap[0]
            if s in self._open_set and self._open_set[s] == (k1, k2):
                return (k1, k2)
            heapq.heappop(self._open_heap)  # entrada obsoleta
        return (INF, INF)

    def _update_vertex(self, u):
        """
        Recalcula rhs[u] e atualiza OPEN.
        Para a busca reversa, os 'sucessores' de u (na direcao do goal)
        sao seus vizinhos v — rhs[u] = min_v(cost(u,v) + g[v]).
        """
        if u != self.goal:
            self.rhs[u] = min(
                self._cost(u, v) + self.g[v]
                for v in self._neighbors(u)
            )

        if u in self._open_set:
            self._remove_from_open(u)

        if self.g[u] != self.rhs[u]:
            key = self._calculate_key(u)
            self._insert_or_update(u, key)

    # ------------------------------------------------------------------
    # API publica
    # ------------------------------------------------------------------

    def compute_shortest_path(self):
        """
        Loop principal do D* Lite.
        Expande nos inconsistentes ate que o start seja consistente
        e sua chave seja menor ou igual a menor chave do heap.

        Retorna
        -------
        nodes_expanded : int
        max_queue_size : int
        """
        nodes_expanded = 0
        max_queue_size = len(self._open_set)

        while True:
            top_key = self._top_key()
            s_key   = self._calculate_key(self.start)

            # Condicao de parada: start consistente E chave do start <= top
            if top_key >= s_key and self.rhs[self.start] == self.g[self.start]:
                break

            if not self._open_set:
                break

            # Extrair no de menor chave
            while self._open_heap:
                k1, k2, u = heapq.heappop(self._open_heap)
                if u not in self._open_set or self._open_set[u] != (k1, k2):
                    continue  # entrada obsoleta (lazy deletion)
                del self._open_set[u]
                break
            else:
                break

            nodes_expanded += 1
            max_queue_size  = max(max_queue_size, len(self._open_set))

            k_old = (k1, k2)
            k_new = self._calculate_key(u)

            if k_old < k_new:
                # Chave obsoleta — re-inserir com chave atualizada
                self._insert_or_update(u, k_new)

            elif self.g[u] > self.rhs[u]:
                # LOWER (superconsistente): propagar reducao de custo
                self.g[u] = self.rhs[u]
                for v in self._neighbors(u):
                    self._update_vertex(v)

            else:
                # RAISE (subconsistente): propagar aumento de custo
                self.g[u] = INF
                self._update_vertex(u)
                for v in self._neighbors(u):
                    self._update_vertex(v)

        return nodes_expanded, max_queue_size

    def extract_path(self):
        """
        Extrai o caminho do start ate o goal seguindo min(cost + g) dos vizinhos.
        Retorna lista de (r, c) ou [] se nao houver caminho.
        """
        if self.g[self.start] == INF:
            return []

        path = [self.start]
        current = self.start
        visited = {self.start}

        for _ in range(MAX_STEPS):
            if current == self.goal:
                break

            best_next = None
            best_cost = INF

            for v in self._neighbors(current):
                if v in visited:
                    continue
                c = self._cost(current, v) + self.g[v]
                if c < best_cost:
                    best_cost = c
                    best_next = v

            if best_next is None or best_cost == INF:
                return []  # sem caminho

            path.append(best_next)
            visited.add(best_next)
            current = best_next

        return path if path[-1] == self.goal else []

    def replan(self, new_start, new_occ_grid):
        """
        Atualiza o planner para um novo frame:
        1. Atualiza km com a distancia percorrida pelo robo.
        2. Detecta celulas alteradas em new_occ_grid vs occ_grid atual.
        3. Propaga mudancas via update_vertex.
        4. Chama compute_shortest_path().

        Retorna
        -------
        nodes_expanded : int
        max_queue_size : int
        cells_changed  : int   — numero de celulas alteradas
        replan_triggered : bool — True se houve mudancas
        """
        # 1. Atualizar km
        self.km    += heuristic(self.s_last, new_start)
        self.s_last = new_start
        self.start  = new_start

        # 2. Detectar mudancas
        diff = np.abs(new_occ_grid.astype(float) - self.occ_grid.astype(float))
        changed_positions = list(zip(*np.where(diff > 1e-6)))
        cells_changed    = len(changed_positions)
        replan_triggered = cells_changed > 0

        # 3. Propagar mudancas
        if replan_triggered:
            self.occ_grid = new_occ_grid.copy()
            for (r, c) in changed_positions:
                s = (r, c)
                self._update_vertex(s)
                for nb in self._neighbors(s):
                    self._update_vertex(nb)

        # 4. Recomputar
        nodes_expanded, max_queue_size = self.compute_shortest_path()

        return nodes_expanded, max_queue_size, cells_changed, replan_triggered


# ---------------------------------------------------------------------------
# Navigator: wrapper frame-a-frame
# ---------------------------------------------------------------------------
class DStarLiteNavigator:
    """
    Interface de alto nivel para o D* Lite, compativel com o padrao
    run_simulation.py / run_antigravity.py do projeto.

    Gerencia:
    - Inicializacao no primeiro frame
    - Replanejamento (ou reutilizacao) a cada frame subsequente
    - Reinicializacao automatica quando o goal muda
    """

    def __init__(self, block_threshold=BLOCK_THRESHOLD):
        self.planner         = None
        self.prev_occ        = None
        self.last_path       = []
        self.current_goal    = None
        self.block_threshold = block_threshold

    def initialize(self, occ_grid, start, goal, cells_changed=None):
        """
        Configura o planner para o primeiro frame ou quando o goal muda.

        Parametros
        ----------
        occ_grid      : np.ndarray
        start         : (r, c)
        goal          : (r, c)
        cells_changed : int ou None
            Numero de celulas alteradas no grid (para replan_triggered).
            None no primeiro frame (nao conta como replanejamento dinamico).

        Retorna metricas do frame.
        """
        self.planner      = DStarLite(occ_grid, start, goal, self.block_threshold)
        self.current_goal = goal
        self.prev_occ     = occ_grid.copy()

        nodes_expanded, max_queue_size = self.planner.compute_shortest_path()
        self.last_path = self.planner.extract_path()

        path_cost   = self.planner.g[start] if self.last_path else INF
        path_length = len(self.last_path) - 1 if self.last_path else 0

        # replan_triggered = True apenas se houve mudanca real no mapa
        cc = cells_changed if cells_changed is not None else 0

        return {
            'nodes_expanded'  : nodes_expanded,
            'max_queue_size'  : max_queue_size,
            'path_cost'       : path_cost if path_cost != INF else -1,
            'path_length'     : path_length,
            'success'         : len(self.last_path) > 0,
            'replan_triggered': cc > 0,
            'cells_changed'   : cc,
            'km_value'        : self.planner.km,
        }

    def step(self, new_occ_grid, current_pos, new_goal):
        """
        Executa um passo do simulador (um frame).

        Parametros
        ----------
        new_occ_grid : np.ndarray
            Grade de ocupacao do frame atual.
        current_pos : (r, c)
            Posicao atual do robo (apos avanco do frame anterior).
        new_goal : (r, c)
            Goal deste frame (pode ser diferente do anterior).

        Retorna
        -------
        dict com metricas do frame + path
        """
        # Detectar mudancas no grid (independente do goal)
        if self.prev_occ is not None:
            diff = np.abs(new_occ_grid.astype(float) - self.prev_occ.astype(float))
            cells_changed = int(np.sum(diff > 1e-6))
        else:
            cells_changed = 0

        # Se goal mudou, reinicializar o planner (mas reportar cells_changed real)
        if new_goal != self.current_goal:
            metrics = self.initialize(new_occ_grid, current_pos, new_goal,
                                      cells_changed=cells_changed)
            self.prev_occ = new_occ_grid.copy()
            return metrics

        # Goal igual: replan incremental
        nodes_expanded, max_queue_size, cc_internal, _ = \
            self.planner.replan(current_pos, new_occ_grid)

        self.last_path = self.planner.extract_path()
        self.prev_occ  = new_occ_grid.copy()

        path_cost   = self.planner.g[current_pos] if self.last_path else INF
        path_length = len(self.last_path) - 1 if self.last_path else 0

        return {
            'nodes_expanded'  : nodes_expanded,
            'max_queue_size'  : max_queue_size,
            'path_cost'       : path_cost if path_cost != INF else -1,
            'path_length'     : path_length,
            'success'         : len(self.last_path) > 0,
            'replan_triggered': cells_changed > 0,
            'cells_changed'   : cells_changed,
            'km_value'        : self.planner.km,
        }

    @property
    def path(self):
        return self.last_path


# ---------------------------------------------------------------------------
# Testes unitarios (executados com: python dstar_lite_nav.py)
# ---------------------------------------------------------------------------
if __name__ == '__main__':
    print("=" * 60)
    print("D* Lite — Testes Unitarios")
    print("=" * 60)

    # ------------------------------------------------------------------
    # Teste 1: Grade 3x3 sem obstaculos — deve encontrar caminho (0,0)->(2,2)
    # ------------------------------------------------------------------
    print("\n[Teste 1] Grade 3x3 sem obstaculos...")
    occ_empty = np.zeros((3, 3))
    planner   = DStarLite(occ_empty, start=(0, 0), goal=(2, 2))
    planner.compute_shortest_path()
    path = planner.extract_path()
    assert len(path) > 0,       "FALHOU: caminho vazio"
    assert path[0]  == (0, 0),  "FALHOU: inicio errado"
    assert path[-1] == (2, 2),  "FALHOU: fim errado"
    assert len(path) - 1 == 4,  f"FALHOU: comprimento esperado 4, obtido {len(path)-1}"
    print(f"  PASSOU — caminho: {path}")

    # ------------------------------------------------------------------
    # Teste 2: Obstaculo surge no caminho — deve replanear
    # ------------------------------------------------------------------
    print("\n[Teste 2] Obstaculo surge no caminho...")
    occ_blocked            = np.zeros((3, 3))
    occ_blocked[1, 1]      = 500.0   # bloquear centro
    planner2               = DStarLite(np.zeros((3, 3)), start=(0, 0), goal=(2, 2))
    planner2.compute_shortest_path()
    path_before            = planner2.extract_path()
    # Novo frame: centro bloqueado
    nodes2, qs2, cc2, rt2  = planner2.replan((0, 0), occ_blocked)
    path_after             = planner2.extract_path()
    assert len(path_after) > 0, "FALHOU: sem caminho apos replanejamento"
    assert (1, 1) not in path_after, "FALHOU: caminho passa pela celula bloqueada"
    assert rt2 == True,         "FALHOU: replan_triggered deveria ser True"
    assert cc2 > 0,             "FALHOU: cells_changed deveria ser > 0"
    print(f"  PASSOU — caminho replanejado: {path_after}, cells_changed={cc2}")

    # ------------------------------------------------------------------
    # Teste 3: Frame sem mudancas — replan_triggered = False, nodes_expanded = 0
    # ------------------------------------------------------------------
    print("\n[Teste 3] Frame sem mudancas...")
    occ_static = np.zeros((5, 5))
    planner3   = DStarLite(occ_static, start=(0, 0), goal=(4, 4))
    planner3.compute_shortest_path()
    # Mesmo occ, robo nao se moveu
    nodes3, qs3, cc3, rt3 = planner3.replan((0, 0), occ_static.copy())
    assert rt3 == False,  "FALHOU: replan_triggered deveria ser False"
    assert cc3 == 0,      "FALHOU: cells_changed deveria ser 0"
    assert nodes3 == 0,   f"FALHOU: nodes_expanded deveria ser 0, obtido {nodes3}"
    print(f"  PASSOU — replan_triggered={rt3}, cells_changed={cc3}, nodes_expanded={nodes3}")

    # ------------------------------------------------------------------
    # Teste 4: Navigator — initialize + step
    # ------------------------------------------------------------------
    print("\n[Teste 4] DStarLiteNavigator initialize + step...")
    occ_nav  = np.zeros((5, 5))
    nav      = DStarLiteNavigator()
    m_init   = nav.initialize(occ_nav, (0, 0), (4, 4))
    assert m_init['success'] == True, "FALHOU: initialize deveria ter sucesso"

    occ_nav2   = np.zeros((5, 5))
    occ_nav2[2, 2] = 500.0  # obstaculo novo
    m_step     = nav.step(occ_nav2, (0, 1), (4, 4))
    assert m_step['replan_triggered'] == True,  "FALHOU: deveria replanejar"
    assert m_step['cells_changed']    > 0,      "FALHOU: cells_changed > 0"
    print(f"  PASSOU — step success={m_step['success']}, km={m_step['km_value']:.2f}")

    print("\n" + "=" * 60)
    print("Todos os testes passaram!")
    print("=" * 60)
