#!/usr/bin/env python3
"""
Entry point for the LinuxCNC PID & Feedforward Auto-Tuner GUI and MCP Server.

Usage on LinuxCNC PC (GUI):
    python3 main.py

Usage as an MCP Server (stdio, for AI clients like Antigravity / Claude / Cursor):
    python3 main.py --mcp

Usage as an MCP HTTP Server (over LAN):
    python3 main.py --mcp-http --host 0.0.0.0 --port 8765

Usage in offline / simulation mode:
    python3 main.py --sim
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

# Ensure `src/` is on sys.path when running directly from a git clone
ROOT_DIR = Path(__file__).resolve().parent
SRC_DIR = ROOT_DIR / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from lcnc_pid_tuner.gui import launch_gui
from lcnc_pid_tuner.mcp_server import run_mcp_http, run_mcp_stdio


def main() -> None:
    parser = argparse.ArgumentParser(
        description="LinuxCNC PID & Feedforward Auto-Tuner (Velocity Servo + Linear Scale)"
    )
    parser.add_argument(
        "--sim",
        action="store_true",
        help="Start in offline physics simulation mode even if LinuxCNC is running.",
    )
    parser.add_argument(
        "--lang",
        choices=["fr", "en", "de"],
        default="fr",
        help="Initial interface language ('fr' for Français, 'en' for English, 'de' for Deutsch).",
    )
    parser.add_argument(
        "--mcp",
        action="store_true",
        help="Run as a Model Context Protocol (MCP) stdio JSON-RPC server for AI integration.",
    )
    parser.add_argument(
        "--mcp-http",
        action="store_true",
        help="Run as an HTTP JSON-RPC MCP server for remote/LAN AI integration.",
    )
    parser.add_argument(
        "--host",
        default="127.0.0.1",
        help="Host address for --mcp-http (default: 127.0.0.1).",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=8765,
        help="Port for --mcp-http (default: 8765).",
    )
    args = parser.parse_args()

    if args.mcp:
        run_mcp_stdio(force_sim=args.sim)
    elif args.mcp_http:
        run_mcp_http(host=args.host, port=args.port, force_sim=args.sim)
    else:
        launch_gui(force_sim=args.sim, default_lang=args.lang)


if __name__ == "__main__":
    main()
