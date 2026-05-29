#!/usr/bin/env bash
set -e
cd "$(dirname "$0")"
if [ -x ".venv/bin/python" ]; then
    exec .venv/bin/python motor_gui.py
fi

exec python3 motor_gui.py
