"""
antigravity — Módulo Dijkstra para Navegação em Grade Dinâmica
Expõe a função unificada dijkstra_path compatível com o pipeline A* existente.
"""

from .metrics import dijkstra_path  # noqa: F401

__all__ = ["dijkstra_path"]
