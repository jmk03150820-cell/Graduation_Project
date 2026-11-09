"""Simulation Layer (Sim Backend) — Phase 1.

External contract comes from interfaces/ (v0.3 bundle). This package only adds
the layer-internal implementation: Frame Input Gate, Registry, snapshot, mock
backend. No message/type is redefined here.
"""
import os
import sys

_IFACE = os.path.join(os.path.dirname(__file__), "..", "interfaces", "generated", "python")
if _IFACE not in sys.path:
    sys.path.insert(0, os.path.abspath(_IFACE))
