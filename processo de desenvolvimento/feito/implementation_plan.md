# Implementação do Benchmark Antigravity

Este documento descreve o plano para implementar o benchmark especificado em `SPEC_BENCHMARK_Antigravity.md`.

## Proposed Changes

### [NEW] run_benchmark.py
Este script será criado na raiz do projeto (`c:\Users\gabri\OneDrive\Ambiente de Trabalho\Faculdade\2026_1\B_2\TEORIA DOS GRAFOS\run_benchmark.py`) e fará o seguinte:
1. **Configuração**: Definirá o dicionário `CONFIG` conforme a spec.
2. **Geração do Cenário**:
   - Função `create_base_grid`: cria o grafo `nx.grid_2d_graph`, calcula dimensões via `dim_m / 0.40`, remove 5% de obstáculos estáticos (garantindo que não corte todos os caminhos para facilitar, e poupando os cantos).
3. **Simulação Dinâmica**:
   - Função `add_pedestrians_and_weights`: A cada frame, gera N pedestres (Poisson).
   - Aplica a penalidade Gaussiana nas arestas e remove arestas com peso > 400.
4. **Execução**:
   - Um loop principal por cenário.
   - O robô busca cobrir as células. Em cada frame, tenta rotear para a célula não visitada mais próxima (Manhattan).
   - Chama os 4 algoritmos (A*, Dijk-Std, Dijk-Bi, Dijk-Dial).
   - O A* usará a heurística Manhattan.
   - Acumula métricas no formato especificado.
5. **Sumarização e Exportação**:
   - Gera `benchmark_raw.csv` a partir dos dados do loop na pasta `output/benchmark/`.
   - Gera `benchmark_summary.csv` agrupando e calculando médias, máximos e totais (conforme regras de agregação da spec).
6. **Visualizações (Plots)**:
   - Usa `matplotlib` para gerar os 4 gráficos (boxplot, linha acumulada, scatter memória, scatter trade-off) na pasta `output/benchmark/`.

### [NEW] README_BENCHMARK.md
- Instruções curtas de como configurar o ambiente (pip install pandas matplotlib networkx numpy scipy) e como rodar o script `run_benchmark.py`. Será criado na raiz do projeto.

## Open Questions

- A spec menciona "goal = célula final (canto superior-direito)" e logo em seguida "próximo_goal = célula não-limpada mais próxima". Assumirei que a intenção é a "cobertura total", ou seja, manter um conjunto de `unvisited_nodes` e sempre mirar no mais próximo, para que o robô explore a grade inteira até acabar as células não-visitadas ou atingir `T` frames, exatamente como a etapa de "cobertura" detalha.

## Verification Plan

### Automated Tests
1. Instalar dependências (pandas, matplotlib, numpy).
2. Rodar `python run_benchmark.py`.
3. Verificar se o terminal loga os progressos.
4. Checar a pasta `output/benchmark` para garantir que `benchmark_raw.csv`, `benchmark_summary.csv` e as 4 figuras PNG foram geradas e não estão vazias.
5. O `benchmark_summary.csv` deve conter os 4 algoritmos para BR-06 e CN-01 (total 8 linhas).
