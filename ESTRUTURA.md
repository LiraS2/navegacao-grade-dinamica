# Estrutura do Projeto — Navegacao em Grade Dinamica

Atualizado em: 2026-05-18 12:28

## Visao Geral

```
.
├── src/                    # Codigo fonte valido (v2.2 / v1.1)
│   ├── antigravity_nav.py
│   ├── dijkstra_nav.py
│   ├── nav_utils.py
│   ├── run_antigravity.py
│   └── run_simulation.py
├── specs/                  # Documentacao tecnica valida
│   ├── SPEC_Dijkstra_Correcao_v2.2.md
│   ├── SPEC_AntiGravity_v1.1.md
│   └── inativado.md
├── output/                 # Resultados auditados
│   ├── antigravity/        # 400 linhas BR-06, 99 linhas CN-01
│   ├── dijkstra/           # 400 linhas BR-06, 99 linhas CN-01
│   └── comparativo/        # Graficos comparativos consolidados
├── historico/              # Arquivos obsoletos (NAO USAR no artigo)
│   ├── README_HISTORICO.md
│   ├── codigo/
│   ├── specs/
│   ├── docs/
│   └── output/
├── tests/                  # Testes unitarios (futuro)
├── .gitignore              # Ignora cache e output temporario
└── ESTRUTURA.md            # Este arquivo
```

## src/ — Codigo Fonte

| Arquivo | Responsabilidade | Versao |
|---------|------------------|--------|
| `nav_utils.py` | Utilitarios compartilhados (pedestres, ocupacao, grade base) | v2.2/v1.1 |
| `dijkstra_nav.py` | Algoritmo Dijkstra-Std com heapq + testes unitarios | v2.2 |
| `antigravity_nav.py` | Algoritmo AntiGravity (campos potenciais) + testes | v1.1 |
| `run_simulation.py` | Runner do Dijkstra — gera CSVs e graficos BR-06/CN-01 | v2.2 |
| `run_antigravity.py` | Runner do AntiGravity — gera CSVs, graficos e overlays | v1.1 |

## specs/ — Documentacao

| Arquivo | Escopo |
|---------|--------|
| `SPEC_Dijkstra_Correcao_v2.2.md` | Dijkstra-Std — alvos realistas, loop corrigido, metricas novas |
| `SPEC_AntiGravity_v1.1.md` | AntiGravity — alvos realistas, comparacao com Dijk-Std |
| `inativado.md` | Variantes removidas (A*, Dijk-Bi, Dijk-Dial) com bugs documentados |

## output/ — Resultados Validados

### Auditoria de Conformidade

| Check | AntiGravity | Dijkstra |
|-------|-------------|----------|
| Linhas CSV BR-06 | 400 | 400 |
| Linhas CSV CN-01 | 99 | 99 |
| Cobertura BR-06 | 22.8% | 23.5% |
| Cobertura CN-01 | 3.5% | 3.9% |
| Tempo medio | 0.028 ms | 0.154 ms |
| Max time | 0.61 ms | 4.0 ms |
| PNGs 300 DPI | 12 | 10 |

### Graficos Comparativos

- `comparativo/fig_comparativo_consolidado.png` — 4 paineis: cobertura acumulada + taxa de cobertura para BR-06 e CN-01

## historico/ — Arquivos Obsoletos

**REGRA DE OURO:** Nenhum arquivo desta pasta deve ser citado no artigo.

| Pasta | Conteudo | Motivo do Arquivamento |
|-------|----------|------------------------|
| `codigo/` | `run_benchmark.py` | Loop MAX_STEPS gerava dados invalidos |
| `specs/` | v2.0, v2.1, v1.0 | Alvos de cobertura fisicamente impossiveis |
| `docs/` | `walkthrough.md`, `README_BENCHMARK.md` | Documentam processo, nao sao entregaveis |
| `output/` | `benchmark/` | Resultados do runner quebrado (cobertura 100% falsa) |

## Como usar este projeto

### Gerar resultados (se necessario)
```bash
cd src
python run_simulation.py      # Dijkstra-Std
python run_antigravity.py     # AntiGravity
```

### Validar
```bash
python -c "import pandas as pd; print(pd.read_csv('output/dijkstra/dijkstra_raw.csv').groupby('scenario').size().to_dict())"
```

## Regras para o Artigo

1. **Usar apenas** `output/antigravity/` e `output/dijkstra/` para tabelas e graficos.
2. **Nunca citar** `run_benchmark.py`, `output/benchmark/` ou specs v2.0/v2.1/v1.0.
3. **Cobertura deve ser reportada como** `X% em N frames` — nunca como porcentagem absoluta do ambiente sem contexto.
4. **Comparar tempo de decisao** (ms/frame) e **eficiencia de exploracao** (%/passo) como metricas principais.