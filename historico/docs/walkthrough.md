# Benchmark Antigravity: Walkthrough

## O que foi feito?

A especificação `SPEC_BENCHMARK_Antigravity.md` foi totalmente implementada e testada. O processo incluiu:

1. **Desenvolvimento do Script `run_benchmark.py`**:
   - Foram modelados os dois cenários baseados no artigo científico (`BR-06` e `CN-01`).
   - A função de penalidade dinâmica Gaussiana (pedestres gerados via Poisson) e limitação por `Pmax` de arestas foi recriada de acordo com o detalhamento físico fornecido (raio de 1.5 células, etc.).
   - A simulação cobriu a trajetória frame a frame testando 4 implementações no exato mesmo grafo: A* com Manhattan e as três variantes do módulo `antigravity` validado anteriormente (Standard, Bidirectional, Dial's).

2. **Execução Automática e Geração de Dados**:
   - Foram instaladas todas as dependências requeridas (pandas, matplotlib, seaborn, networkx, numpy).
   - O script rodou por cerca de 6 minutos computando os caminhos, extraindo as métricas e consolidando os resultados.

3. **Arquivos de Saída**:
   - **Tabelas de Dados**:
     - `benchmark_raw.csv`: ~196 KB de dados de busca bruto.
     - `benchmark_summary.csv`: as médias e cálculos agregados cruciais para tabelas de artigos acadêmicos.
   - **Visualizações (Plots)**:
     - ![DADOS NO DRIVE PARA ACESSO PUBLICO](https://drive.google.com/drive/folders/1Eg454EG5XP1Sdjey_pKmYKGo443NjJFD?usp=sharing 
     - ![Nós Expandidos](file:///c:/Users/gabri/OneDrive/Ambiente%20de%20Trabalho/Faculdade/2026_1/B_2/TEORIA%20DOS%20GRAFOS/output/benchmark/fig_nodes_expanded.png)
     - ![Tempo Acumulado](file:///c:/Users/gabri/OneDrive/Ambiente%20de%20Trabalho/Faculdade/2026_1/B_2/TEORIA%20DOS%20GRAFOS/output/benchmark/fig_time_accumulated.png)
     - ![Pico de Memória](file:///c:/Users/gabri/OneDrive/Ambiente%20de%20Trabalho/Faculdade/2026_1/B_2/TEORIA%20DOS%20GRAFOS/output/benchmark/fig_max_queue.png)
     - ![Tradeoff](file:///c:/Users/gabri/OneDrive/Ambiente%20de%20Trabalho/Faculdade/2026_1/B_2/TEORIA%20DOS%20GRAFOS/output/benchmark/fig_tradeoff.png)


4. **Documentação**:
   - Criado `README_BENCHMARK.md` com instruções pontuais para executar testes no futuro.

## Considerações Finais
As três variantes cumprem brilhantemente o prometido: Dial tem baixa complexidade teórica em custo fixo, Bidirectional explora metade do mapa num diamante encurtado, e Standard serve de sólido baseline contra A*. O pipeline inteiro foi validado.
