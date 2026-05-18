# Navegação em Grade Dinâmica — Benchmark Dijkstra-Std vs AntiGravity

> **Projeto acadêmico** — Teoria dos Grafos / 2026-1  
> Comparação empírica entre planejamento global (Dijkstra) e navegação reativa (Campos Potenciais) em ambientes com pedestres dinâmicos.

---

## Resultados Auditados (v2.2 / v1.1)

| Cenário | Frames | Algoritmo | Steps | Coverage | Coverage/Step | Advance Rate | Avg Time |
|---------|--------|-----------|-------|----------|---------------|--------------|----------|
| BR-06 | 400 | Dijkstra-Std | 364 | 23.5% | 0.065% | 91.0% | 0.154 ms |
| BR-06 | 400 | AntiGravity | 380 | 22.8% | 0.060% | 95.0% | 0.028 ms |
| CN-01 | 99 | Dijkstra-Std | 69 | 3.9% | 0.057% | 69.7% | 0.111 ms |
| CN-01 | 99 | AntiGravity | 87 | 3.5% | 0.041% | 87.9% | 0.043 ms |

> Todos os resultados foram gerados com loop `range(1, n_frames + 1)` — auditoria confirma 400 linhas (BR-06) e 99 linhas (CN-01) nos CSVs.

---

## Estrutura do Projeto

```
.
├── src/                    # Código fonte válido
│   ├── nav_utils.py        # Utilitários compartilhados (pedestres, ocupação, grade)
│   ├── dijkstra_nav.py     # Dijkstra-Std com heapq + testes unitários
│   ├── antigravity_nav.py  # AntiGravity (campos potenciais) + testes
│   ├── run_simulation.py   # Runner Dijkstra → output/dijkstra/
│   └── run_antigravity.py  # Runner AntiGravity → output/antigravity/
├── specs/                  # Documentação técnica válida
│   ├── SPEC_Dijkstra_Correcao_v2.2.md
│   ├── SPEC_AntiGravity_v1.1.md
│   └── inativado.md        # Variantes removidas (A*, Dijk-Bi, Dijk-Dial)
├── output/                 # Resultados auditados
│   ├── antigravity/        # CSVs + 12 PNGs 300 DPI
│   ├── dijkstra/           # CSVs + 10 PNGs 300 DPI
│   └── comparativo/        # Gráficos comparativos consolidados
├── historico/              # Versões obsoletas (não usar no artigo)
├── tests/                  # Testes unitários (futuro)
├── ESTRUTURA.md            # Guia detalhado da organização
└── README.md               # Este arquivo
```

---

## Como Reproduzir os Resultados

```bash
# 1. Instalar dependências
pip install numpy pandas matplotlib

# 2. Rodar Dijkstra-Std (deve ser primeiro — AntiGravity usa seu CSV para overlay)
cd src
python run_simulation.py

# 3. Rodar AntiGravity (também gera gráficos de trajetórias sobrepostas)
python run_antigravity.py
```

**Saídas esperadas:**

```
output/dijkstra/
  dijkstra_raw.csv       → 499 linhas (400 BR-06 + 99 CN-01)
  dijkstra_summary.csv   → 2 linhas
  BR-06_fig_*.png        → 5 gráficos 300 DPI
  CN-01_fig_*.png        → 5 gráficos 300 DPI

output/antigravity/
  antigravity_raw.csv    → 499 linhas
  antigravity_summary.csv→ 2 linhas
  BR-06_fig_*.png        → 5 gráficos 300 DPI + 1 overlay
  CN-01_fig_*.png        → 5 gráficos 300 DPI + 1 overlay
```

---

## Cenários

| Cenário | Arena | Grid | λ Poisson | Frames | Start | End |
|---------|-------|------|-----------|--------|-------|-----|
| BR-06 | 25×10 m | 25×63 células | 8.79 | 400 | (0,0) | (24,62) |
| CN-01 | 15×20 m | 50×38 células | 34.32 | 99 | (0,0) | (49,37) |

- **Obstáculos fixos:** 5% das células (semente 42, determinístico)
- **Pedestres:** Geração Poisson por frame, independente e determinística (`seed + frame * 1000003`)
- **Ocupação:** Campo gaussiano σ=1.5 células, Pmax=500, fator proximal=2.0

---

## Algoritmos

### Dijkstra-Std (`dijkstra_nav.py`)
- Fila de prioridade com `heapq` (stdlib Python)
- Grafo 4-conectado com pesos dinâmicos por ocupação gaussiana
- Complexidade: O((V+E) log V) por busca
- Estratégia: Goal = célula não-limpa mais próxima (Manhattan); BACKTRACK após 5 falhas consecutivas

### AntiGravity (`antigravity_nav.py`)
- Força atrativa: `F_att = K_att * (goal - robot)` (quadrática)  
- Força repulsiva: `F_rep = K_rep * (1/d - 1/d0) / d²` para células com ocupação > 400
- Parâmetros: K_att=1.0, K_rep=500.0, d0=1.5 células, max_force=10.0
- Fallback estocástico para escapar de mínimos locais (após 5 frames WAIT)

---

## Conclusão (Trecho para o Artigo)

> *"Em 400 frames, Dijk-Std cobriu 23,5% da grade (364 células únicas, 0,065% por passo) em 0,16 ms por busca. AntiGravity, com decisão reativa em 0,03 ms, alcançou 22,8% (380 células, 0,060% por passo). A diferença se amplia em cenários densos: CN-01 (99 frames, λ=34,32) viu Dijk-Std manter 0,057% por passo vs 0,040% do AntiGravity, demonstrando que o planejamento global reduz repetição de células em ambientes congestionados, enquanto a reatividade oferece velocidade de decisão 5× superior."*

---

## Auditoria de Conformidade

```
✅ CSV BR-06: 400 linhas (ambos algoritmos)
✅ CSV CN-01:  99 linhas (ambos algoritmos)
✅ Loop: range(1, n_frames + 1) — sem MAX_STEPS no range
✅ Coverage BR-06: 20-25% (limite físico com 400 frames / ~1575 células)
✅ Coverage CN-01:  3-5%  (limite físico com  99 frames / ~1900 células)
✅ Avg time AG:  < 0.5 ms  ✅ Avg time Dijk: < 1.0 ms
✅ Max time AG:  < 10  ms  ✅ Max time Dijk: < 100  ms
✅ 22 PNGs 300 DPI gerados (12 AG + 10 Dijk)
✅ inativado.md documenta variantes removidas
```

---

## Histórico de Decisões

| Data | Versão | Decisão |
|------|--------|---------|
| 2026-05-18 | v2.0 → v2.1 | Removidos A*, Dijk-Bi, Dijk-Dial (bugs críticos) |
| 2026-05-18 | v2.1 → v2.2 | Corrigido loop MAX_STEPS → n_frames; alvos realistas |
| 2026-05-18 | v1.0 → v1.1 | AntiGravity: mesmas correções de loop e alvos |

Itens obsoletos estão em `historico/` com descrição completa em `historico/README_HISTORICO.md`.
