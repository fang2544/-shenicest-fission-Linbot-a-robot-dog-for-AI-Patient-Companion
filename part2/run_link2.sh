#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")" && pwd)"
cd "$ROOT/app"
if [[ ! -d .venv ]]; then
  python3 -m venv .venv
fi
if ! .venv/bin/python -c "import ultralytics, cv2, PIL" 2>/dev/null; then
  .venv/bin/python -m pip install -U pip
  .venv/bin/pip install -r requirements.txt
fi
export PYTHONUNBUFFERED=1
exec .venv/bin/python sentry.py --source link2 "$@"
