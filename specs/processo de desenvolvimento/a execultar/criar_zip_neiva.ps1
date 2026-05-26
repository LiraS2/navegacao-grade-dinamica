#!/usr/bin/env powershell
# criar_zip_neiva.ps1 — só o essencial pro artigo

$root = "C:\Users\gabri\OneDrive\Ambiente de Trabalho\Faculdade\2026_1\B_2\TEORIA DOS GRAFOS"
$temp = "$env:TEMP\neiva_artigo"
$zip = "$root\neiva_artigo.zip"

# Limpar
if (Test-Path $temp) { Remove-Item $temp -Recurse -Force }
if (Test-Path $zip) { Remove-Item $zip -Force }

# Criar pastas
New-Item -ItemType Directory -Path "$temp\resultados" -Force | Out-Null
New-Item -ItemType Directory -Path "$temp\figuras" -Force | Out-Null

# Copiar CSVs
Copy-Item "$root\output\dijkstra\dijkstra_summary.csv" "$temp\resultados\" -Force
Copy-Item "$root\output\antigravity\antigravity_summary.csv" "$temp\resultados\" -Force
Copy-Item "$root\output\dstar_lite\dstar_lite_summary.csv" "$temp\resultados\" -Force

# Copiar figuras principais (5 só)
Copy-Item "$root\output\comparativo\fig_triplo_comparativo.png" "$temp\figuras\" -Force
Copy-Item "$root\output\dijkstra\BR-06_fig_coverage_over_time.png" "$temp\figuras\dijkstra_br06_cobertura.png" -Force
Copy-Item "$root\output\dijkstra\CN-01_fig_coverage_over_time.png" "$temp\figuras\dijkstra_cn01_cobertura.png" -Force
Copy-Item "$root\output\dstar_lite\BR-06_fig_replan_frequency.png" "$temp\figuras\dstar_lite_br06_replan.png" -Force
Copy-Item "$root\output\dstar_lite\CN-01_fig_replan_frequency.png" "$temp\figuras\dstar_lite_cn01_replan.png" -Force

# Criar descricao pro artigo
$descricao = @"
RESULTADOS DO BENCHMARK — Navegacao em Grade Dinamica

Tres algoritmos foram implementados e comparados nos cenarios BR-06 (25x63, 400 frames, lambda=8.79) e CN-01 (50x38, 99 frames, lambda=34.32) do Cultural Crowds Dataset.

DIJKSTRA-STD (planejamento global)
- BR-06: 364 passos, 23.5% cobertura, 0.08 ms/busca
- CN-01: 69 passos, 3.9% cobertura, 0.13 ms/busca
- Garantia de otimalidade, recalcula tudo a cada frame

ANTIGRAVITY v2.0 (campos potenciais — reativo)
- BR-06: 388 passos, 25.0% cobertura, 0.02 ms/frame
- CN-01: 94 passos, 5.0% cobertura, 0.02 ms/frame
- 5x mais rapido que Dijkstra, sem busca em grafo

D* LITE (replanejamento incremental)
- BR-06: 372 passos, 24.0% cobertura, 1.63 ms/replan
- CN-01: 76 passos, 4.3% cobertura, 7.52 ms/replan
- Reaproveita busca anterior, mas degenera em ambientes densamente dinamicos

CONCLUSAO: Em grades pequenas com alta densidade de pedestres, Dijkstra-Std mantem melhor relacao custo-beneficio. AntiGravity oferece velocidade extrema de decisao. D* Lite so e vantajoso em cenarios com mudancas esparsas.

Repositorio completo: https://github.com/LiraS2/navegacao-grade-dinamica
"@

$descricao | Out-File -Encoding utf8 "$temp\descricao_resultados.txt"

# Criar ZIP
Compress-Archive -Path $temp -DestinationPath $zip -Force

Write-Host "ZIP criado: $zip" -ForegroundColor Green
Write-Host "Tamanho: $([math]::Round((Get-Item $zip).Length/1KB, 1)) KB" -ForegroundColor Cyan