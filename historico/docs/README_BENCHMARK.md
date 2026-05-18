# Benchmark Antigravity

Este repositório contém o script de benchmark usado para avaliar as variantes do algoritmo Dijkstra (implementadas no módulo `antigravity`) contra o A* do NetworkX, focado em ambientes com grades de navegação dinâmicas e variação de peso influenciada por ocupação de pedestres.

## Cenários Avaliados

Os cenários são baseados na especificação do artigo:
- **BR-06**: Espaço de 25m × 10m com aproximadamente 8.8 pedestres médios por frame. Simulado por 400 frames.
- **CN-01**: Espaço de 15m × 20m com aproximadamente 34.3 pedestres médios por frame. Simulado por 99 frames.

A penalidade de obstáculo dinâmico segue uma distribuição gaussiana (onde o peso base é 1.0, subindo até perto de 500.0) calculada a partir de posições (Poisson) dos pedestres.

## Requisitos

Instale as dependências executando o comando abaixo:

```bash
pip install networkx numpy pandas matplotlib seaborn pytest
```

## Como executar

Para executar o benchmark completo e gerar os resultados brutos, sumarizados e os gráficos, rode:

```bash
python run_benchmark.py
```

O processo pode levar de alguns minutos a algumas dezenas de minutos, dependendo da máquina, pois a cada frame o robô buscará a célula livre mais próxima (exploração total) e recriará o grafo com pesos atualizados.

## Saídas Geradas

As saídas serão salvas no diretório `output/benchmark/`:

- `benchmark_raw.csv`: Tabela contendo frame a frame o resultado de cada algoritmo, incluindo tempos, custos, nós expandidos e memória da fila de prioridade.
- `benchmark_summary.csv`: Uma tabela resumida com as médias e totais por cenário e algoritmo.
- `fig_nodes_expanded.png`: Boxplot do número de nós expandidos.
- `fig_time_accumulated.png`: Gráfico do tempo acumulado das simulações.
- `fig_max_queue.png`: Scatter plot detalhando o pico de memória das estruturas de fila de prioridade.
- `fig_tradeoff.png`: Gráfico Trade-off avaliando Média de Nós Expandidos vs Tempo Médio de Busca.
