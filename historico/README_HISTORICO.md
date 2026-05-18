# Historico do Projeto — Itens Arquivados

Gerado em: 2026-05-18 12:28

## Por que existe esta pasta?

Esta pasta contem versoes antigas, codigo quebrado, specs obsoletas e resultados invalidos.
NENHUM arquivo aqui deve ser usado no artigo ou no benchmark final.
Mantemos por razoes de auditoria, reproducao e registro de decisoes.

## Estrutura

### codigo/

- **run_benchmark.py** — Runner antigo com loop MAX_STEPS=15000 (bug). Gerou dados invalidos de cobertura 100%.

### specs/

- **SPEC_Dijkstra_Correcao_v2.0.md** — Versao inicial com 4 algoritmos (A*, Dijk-Bi, Dijk-Dial, Dijk-Std).
- **SPEC_Dijkstra_Correcao_v2.1.md** — Primeira correcao — ainda com alvos de cobertura impossiveis (>90%).
- **SPEC_AntiGravity_v1.0.md** — Primeira versao — alvos de cobertura impossiveis (>85%).

### docs/

- **walkthrough.md** — Walkthrough da implementacao. Descreve o erro MAX_STEPS e a correcao.
- **README_BENCHMARK.md** — README da versao antiga do benchmark (dados invalidos).

### output/

- **benchmark/** — Resultados do run_benchmark.py — cobertura 100% falsa, loop infinito em Dijk-Dial.

## Decisoes de Arquivamento

| Data | Decisao | Motivo |
|------|---------|--------|
| 2026-05-18 | Mover `run_benchmark.py` para historico | Loop MAX_STEPS gerava dados invalidos |
| 2026-05-18 | Mover specs v2.0/v2.1/v1.0 para historico | Alvos de cobertura fisicamente impossiveis |
| 2026-05-18 | Mover `output/benchmark/` para historico | Cobertura 100% falsa, variantes quebradas incluidas |
| 2026-05-18 | Mover `walkthrough.md` para historico | Documenta processo, nao eh entregavel final |

## Como restaurar (se necessario)

Cada item pode ser copiado de volta para o root manualmente.
Recomendacao: nunca restaurar `run_benchmark.py` ou `output/benchmark/` — usar apenas como referencia.