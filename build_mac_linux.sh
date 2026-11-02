#!/usr/bin/env bash
# Builds GoProMapPlayer for macOS (.app) or Linux. Needs python3 on THIS machine only.
set -euo pipefail
cd "$(dirname "$0")"
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
python build.py
