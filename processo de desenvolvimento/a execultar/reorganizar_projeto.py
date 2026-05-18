#!/usr/bin/env python3
"""
reorganizar_projeto.py

Script de auditoria e reorganizacao do projeto de navegacao em grade.
Avalia cada arquivo/pasta, separa o que eh valido do que eh historico,
gera documentacao e limpa caches.

USO:
    python reorganizar_projeto.py

O script NAO apaga arquivos permanentemente — move para `historico/`.
"""

import os
import shutil
import json
from datetime import datetime
from pathlib import Path

# ============================================================
# CONFIGURACAO
# ============================================================
ROOT = Path(".")
HISTORICO = ROOT / "historico"
SRC = ROOT / "src"
SPECS = ROOT / "specs"
OUTPUT = ROOT / "output"
TESTS = ROOT / "tests"

# Arquivos/pastas que podem ser DELETADOS (cache, temporarios)
LIXO = [
    "__pycache__",
    ".pytest_cache",
    "*.pyc",
    "*.pyo",
    ".DS_Store",
]

# ============================================================
# MAPEAMENTO: o que eh VALIDO vs HISTORICO
# ============================================================

VALIDOS = {
    # Codigo fonte atual (v2.2 / v1.1)
    "src": [
        "antigravity_nav.py",
        "dijkstra_nav.py",
        "nav_utils.py",
        "run_antigravity.py",
        "run_simulation.py",
    ],
    # Specs validas
    "specs": [
        "SPEC_Dijkstra_Correcao_v2.2.md",
        "SPEC_AntiGravity_v1.1.md",
        "inativado.md",
    ],
    # Output valido (auditado)
    "output": [
        "antigravity/antigravity_raw.csv",
        "antigravity/antigravity_summary.csv",
        "antigravity/*.png",
        "dijkstra/dijkstra_raw.csv",
        "dijkstra/dijkstra_summary.csv",
        "dijkstra/*.png",
        "comparativo/fig_comparativo_consolidado.png",
    ]
}

HISTORICOS = {
    # Codigo quebrado / obsoleto
    "codigo": [
        ("run_benchmark.py", "Runner antigo com loop MAX_STEPS=15000 (bug). Gerou dados invalidos de cobertura 100%."),
    ],
    # Specs antigas
    "specs": [
        ("SPEC_Dijkstra_Correcao_v2.0.md", "Versao inicial com 4 algoritmos (A*, Dijk-Bi, Dijk-Dial, Dijk-Std)."),
        ("SPEC_Dijkstra_Correcao_v2.1.md", "Primeira correcao — ainda com alvos de cobertura impossiveis (>90%)."),
        ("SPEC_AntiGravity_v1.0.md", "Primeira versao — alvos de cobertura impossiveis (>85%)."),
    ],
    # Documentacao de processo
    "docs": [
        ("walkthrough.md", "Walkthrough da implementacao. Descreve o erro MAX_STEPS e a correcao."),
        ("README_BENCHMARK.md", "README da versao antiga do benchmark (dados invalidos)."),
    ],
    # Output antigo (bugado)
    "output": [
        ("benchmark/", "Resultados do run_benchmark.py — cobertura 100% falsa, loop infinito em Dijk-Dial."),
    ]
}

# ============================================================
# FUNCOES
# ============================================================

def ensure_dir(path: Path):
    path.mkdir(parents=True, exist_ok=True)

def move_to_historico(src: Path, dest_folder: str, description: str):
    """Move arquivo/pasta para historico/<dest_folder>/ com descricao no README."""
    dest = HISTORICO / dest_folder / src.name
    ensure_dir(dest.parent)

    if src.exists():
        if dest.exists():
            # Renomear se ja existe
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            dest = dest.parent / f"{src.stem}_{timestamp}{src.suffix}"

        if src.is_dir():
            shutil.copytree(src, dest, dirs_exist_ok=True)
            shutil.rmtree(src)
        else:
            shutil.move(str(src), str(dest))

        print(f"  [HISTORICO] {src} -> {dest}")
        return True
    return False

def move_to_src(src: Path):
    """Move codigo valido para src/."""
    dest = SRC / src.name
    ensure_dir(SRC)
    if src.exists():
        if dest.exists():
            shutil.move(str(dest), str(HISTORICO / "codigo" / f"{dest.name}.bak"))
        shutil.move(str(src), str(dest))
        print(f"  [SRC] {src} -> {dest}")

def move_to_specs(src: Path):
    """Move spec valida para specs/."""
    dest = SPECS / src.name
    ensure_dir(SPECS)
    if src.exists():
        if dest.exists():
            shutil.move(str(dest), str(HISTORICO / "specs" / f"{dest.name}.bak"))
        shutil.move(str(src), str(dest))
        print(f"  [SPECS] {src} -> {dest}")

def clean_lixo():
    """Remove caches e temporarios."""
    for pattern in LIXO:
        if pattern.startswith("*"):
            # Glob pattern
            for f in ROOT.rglob(pattern):
                if f.is_dir():
                    shutil.rmtree(f)
                    print(f"  [LIXO REMOVIDO] {f}")
                else:
                    f.unlink()
                    print(f"  [LIXO REMOVIDO] {f}")
        else:
            # Nome exato
            for f in ROOT.rglob(pattern):
                if f.is_dir():
                    shutil.rmtree(f)
                    print(f"  [LIXO REMOVIDO] {f}")

def generate_historico_readme():
    """Gera README_HISTORICO.md descrevendo cada item no historico."""
    lines = [
        "# Historico do Projeto — Itens Arquivados",
        "",
        f"Gerado em: {datetime.now().strftime('%Y-%m-%d %H:%M')}",
        "",
        "## Por que existe esta pasta?",
        "",
        "Esta pasta contem versoes antigas, codigo quebrado, specs obsoletas e resultados invalidos.",
        "NENHUM arquivo aqui deve ser usado no artigo ou no benchmark final.",
        "Mantemos por razoes de auditoria, reproducao e registro de decisoes.",
        "",
        "## Estrutura",
        "",
    ]

    for folder, items in HISTORICOS.items():
        lines.append(f"### {folder}/")
        lines.append("")
        for item, desc in items:
            lines.append(f"- **{item}** — {desc}")
        lines.append("")

    lines.extend([
        "## Decisoes de Arquivamento",
        "",
        "| Data | Decisao | Motivo |",
        "|------|---------|--------|",
        "| 2026-05-18 | Mover `run_benchmark.py` para historico | Loop MAX_STEPS gerava dados invalidos |",
        "| 2026-05-18 | Mover specs v2.0/v2.1/v1.0 para historico | Alvos de cobertura fisicamente impossiveis |",
        "| 2026-05-18 | Mover `output/benchmark/` para historico | Cobertura 100% falsa, variantes quebradas incluidas |",
        "| 2026-05-18 | Mover `walkthrough.md` para historico | Documenta processo, nao eh entregavel final |",
        "",
        "## Como restaurar (se necessario)",
        "",
        "Cada item pode ser copiado de volta para o root manualmente.",
        "Recomendacao: nunca restaurar `run_benchmark.py` ou `output/benchmark/` — usar apenas como referencia.",
    ])

    readme_path = HISTORICO / "README_HISTORICO.md"
    ensure_dir(HISTORICO)
    with open(readme_path, 'w', encoding='utf-8') as f:
        f.write("\n".join(lines))
    print(f"\n  [README] Gerado {readme_path}")

def generate_estrutura_md():
    """Gera ESTRUTURA.md documentando a organizacao atual."""
    lines = [
        "# Estrutura do Projeto — Navegacao em Grade Dinamica",
        "",
        f"Atualizado em: {datetime.now().strftime('%Y-%m-%d %H:%M')}",
        "",
        "## Visao Geral",
        "",
        "```",
        ".",
        "├── src/                    # Codigo fonte valido (v2.2 / v1.1)",
        "│   ├── antigravity_nav.py",
        "│   ├── dijkstra_nav.py",
        "│   ├── nav_utils.py",
        "│   ├── run_antigravity.py",
        "│   └── run_simulation.py",
        "├── specs/                  # Documentacao tecnica valida",
        "│   ├── SPEC_Dijkstra_Correcao_v2.2.md",
        "│   ├── SPEC_AntiGravity_v1.1.md",
        "│   └── inativado.md",
        "├── output/                 # Resultados auditados",
        "│   ├── antigravity/        # 400 linhas BR-06, 99 linhas CN-01",
        "│   ├── dijkstra/           # 400 linhas BR-06, 99 linhas CN-01",
        "│   └── comparativo/        # Graficos comparativos consolidados",
        "├── historico/              # Arquivos obsoletos (NAO USAR no artigo)",
        "│   ├── README_HISTORICO.md",
        "│   ├── codigo/",
        "│   ├── specs/",
        "│   ├── docs/",
        "│   └── output/",
        "├── tests/                  # Testes unitarios (futuro)",
        "├── .gitignore              # Ignora cache e output temporario",
        "└── ESTRUTURA.md            # Este arquivo",
        "```",
        "",
        "## src/ — Codigo Fonte",
        "",
        "| Arquivo | Responsabilidade | Versao |",
        "|---------|------------------|--------|",
        "| `nav_utils.py` | Utilitarios compartilhados (pedestres, ocupacao, grade base) | v2.2/v1.1 |",
        "| `dijkstra_nav.py` | Algoritmo Dijkstra-Std com heapq + testes unitarios | v2.2 |",
        "| `antigravity_nav.py` | Algoritmo AntiGravity (campos potenciais) + testes | v1.1 |",
        "| `run_simulation.py` | Runner do Dijkstra — gera CSVs e graficos BR-06/CN-01 | v2.2 |",
        "| `run_antigravity.py` | Runner do AntiGravity — gera CSVs, graficos e overlays | v1.1 |",
        "",
        "## specs/ — Documentacao",
        "",
        "| Arquivo | Escopo |",
        "|---------|--------|",
        "| `SPEC_Dijkstra_Correcao_v2.2.md` | Dijkstra-Std — alvos realistas, loop corrigido, metricas novas |",
        "| `SPEC_AntiGravity_v1.1.md` | AntiGravity — alvos realistas, comparacao com Dijk-Std |",
        "| `inativado.md` | Variantes removidas (A*, Dijk-Bi, Dijk-Dial) com bugs documentados |",
        "",
        "## output/ — Resultados Validados",
        "",
        "### Auditoria de Conformidade",
        "",
        "| Check | AntiGravity | Dijkstra |",
        "|-------|-------------|----------|",
        "| Linhas CSV BR-06 | 400 | 400 |",
        "| Linhas CSV CN-01 | 99 | 99 |",
        "| Cobertura BR-06 | 22.8% | 23.5% |",
        "| Cobertura CN-01 | 3.5% | 3.9% |",
        "| Tempo medio | 0.028 ms | 0.154 ms |",
        "| Max time | 0.61 ms | 4.0 ms |",
        "| PNGs 300 DPI | 12 | 10 |",
        "",
        "### Graficos Comparativos",
        "",
        "- `comparativo/fig_comparativo_consolidado.png` — 4 paineis: cobertura acumulada + taxa de cobertura para BR-06 e CN-01",
        "",
        "## historico/ — Arquivos Obsoletos",
        "",
        "**REGRA DE OURO:** Nenhum arquivo desta pasta deve ser citado no artigo.",
        "",
        "| Pasta | Conteudo | Motivo do Arquivamento |",
        "|-------|----------|------------------------|",
        "| `codigo/` | `run_benchmark.py` | Loop MAX_STEPS gerava dados invalidos |",
        "| `specs/` | v2.0, v2.1, v1.0 | Alvos de cobertura fisicamente impossiveis |",
        "| `docs/` | `walkthrough.md`, `README_BENCHMARK.md` | Documentam processo, nao sao entregaveis |",
        "| `output/` | `benchmark/` | Resultados do runner quebrado (cobertura 100% falsa) |",
        "",
        "## Como usar este projeto",
        "",
        "### Gerar resultados (se necessario)",
        "```bash",
        "cd src",
        "python run_simulation.py      # Dijkstra-Std",
        "python run_antigravity.py     # AntiGravity",
        "```",
        "",
        "### Validar",
        "```bash",
        "python -c \"import pandas as pd; print(pd.read_csv('output/dijkstra/dijkstra_raw.csv').groupby('scenario').size().to_dict())\"",
        "```",
        "",
        "## Regras para o Artigo",
        "",
        "1. **Usar apenas** `output/antigravity/` e `output/dijkstra/` para tabelas e graficos.",
        "2. **Nunca citar** `run_benchmark.py`, `output/benchmark/` ou specs v2.0/v2.1/v1.0.",
        "3. **Cobertura deve ser reportada como** `X% em N frames` — nunca como porcentagem absoluta do ambiente sem contexto.",
        "4. **Comparar tempo de decisao** (ms/frame) e **eficiencia de exploracao** (%/passo) como metricas principais.",
    ]

    with open(ROOT / "ESTRUTURA.md", 'w', encoding='utf-8') as f:
        f.write("\n".join(lines))
    print(f"  [ESTRUTURA] Gerado ESTRUTURA.md")

def generate_gitignore():
    """Gera .gitignore adequado."""
    content = """# Python cache
__pycache__/
*.py[cod]
*$py.class
.pytest_cache/

# Ambiente virtual
venv/
env/

# IDEs
.vscode/
.idea/
*.swp
*.swo

# OS
.DS_Store
Thumbs.db

# Output temporario (mantemos apenas o valido)
# Descomente se quiser ignorar output/ completamente no git
# output/

# Historico (opcional — se for grande, ignore)
# historico/
"""
    with open(ROOT / ".gitignore", 'w', encoding='utf-8') as f:
        f.write(content)
    print(f"  [GITIGNORE] Gerado .gitignore")

# ============================================================
# EXECUCAO PRINCIPAL
# ============================================================

def main():
    print("=" * 60)
    print("REORGANIZACAO DO PROJETO")
    print("=" * 60)
    print(f"\nData: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print("\nEsta operacao vai:")
    print("  1. Mover codigo valido para src/")
    print("  2. Mover specs validas para specs/")
    print("  3. Mover arquivos obsoletos para historico/")
    print("  4. Remover caches (__pycache__, .pytest_cache)")
    print("  5. Gerar README_HISTORICO.md, ESTRUTURA.md, .gitignore")
    print("\n" + "=" * 60)

    # 1. Limpar lixo
    print("\n[1/5] Removendo caches e temporarios...")
    clean_lixo()

    # 2. Mover codigo valido para src/
    print("\n[2/5] Organizando codigo fonte valido...")
    ensure_dir(SRC)
    for fname in VALIDOS["src"]:
        src = ROOT / fname
        if src.exists():
            move_to_src(src)
        else:
            print(f"  [AVISO] {fname} nao encontrado no root")

    # 3. Mover specs validas
    print("\n[3/5] Organizando specs validas...")
    ensure_dir(SPECS)
    for fname in VALIDOS["specs"]:
        # Procurar no root e em "processo de desenvolvimento"
        found = False
        for search_dir in [ROOT, ROOT / "processo de desenvolvimento"]:
            src = search_dir / fname
            if src.exists():
                move_to_specs(src)
                found = True
                break
        if not found:
            print(f"  [AVISO] {fname} nao encontrado")

    # 4. Mover historicos
    print("\n[4/5] Arquivando itens obsoletos...")

    # Codigo obsoleto
    for fname, desc in HISTORICOS["codigo"]:
        src = ROOT / fname
        if src.exists():
            move_to_historico(src, "codigo", desc)

    # Specs obsoletas
    for fname, desc in HISTORICOS["specs"]:
        found = False
        for search_dir in [ROOT, ROOT / "processo de desenvolvimento"]:
            src = search_dir / fname
            if src.exists():
                move_to_historico(src, "specs", desc)
                found = True
                break
        if not found:
            print(f"  [AVISO] {fname} nao encontrado para arquivar")

    # Docs obsoletos
    for fname, desc in HISTORICOS["docs"]:
        src = ROOT / fname
        if src.exists():
            move_to_historico(src, "docs", desc)

    # Output obsoleto
    for fname, desc in HISTORICOS["output"]:
        src = ROOT / "output" / fname
        if src.exists():
            move_to_historico(src, "output", desc)

    # 5. Gerar documentacao
    print("\n[5/5] Gerando documentacao...")
    generate_historico_readme()
    generate_estrutura_md()
    generate_gitignore()

    # 6. Criar pastas de output valido
    ensure_dir(OUTPUT / "comparativo")
    ensure_dir(TESTS)

    print("\n" + "=" * 60)
    print("REORGANIZACAO CONCLUIDA")
    print("=" * 60)
    print(f"\nEstrutura final:")
    print(f"  src/         : {len(list(SRC.glob('*')))} arquivos")
    print(f"  specs/       : {len(list(SPECS.glob('*')))} arquivos")
    print(f"  output/      : {len(list(OUTPUT.glob('*')))} pastas")
    print(f"  historico/   : {len(list(HISTORICO.rglob('*')))} itens")
    print(f"\nLeia ESTRUTURA.md para entender a organizacao.")
    print(f"Leia historico/README_HISTORICO.md para entender o que foi arquivado.")

if __name__ == "__main__":
    main()
