#!/usr/bin/env python3
"""
Entry point for the LinuxCNC PID & Feedforward Auto-Tuner GUI.

Usage on LinuxCNC PC:
    python3 main.py

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
        choices=["fr", "en"],
        default="fr",
        help="Initial interface language ('fr' for Français, 'en' for English).",
    )
    args = parser.parse_args()
    launch_gui(force_sim=args.sim, default_lang=args.lang)


if __name__ == "__main__":
    main()
