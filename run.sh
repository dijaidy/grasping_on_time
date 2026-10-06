#!/bin/sh
# Invoke mjpython through Python: macOS may truncate the launcher shebang
# when this project's absolute path contains long decomposed Korean names.
set -eu
PROJECT_DIR=$(CDPATH= cd "$(dirname "$0")" && pwd)
cd "$PROJECT_DIR"
exec "$PROJECT_DIR/.venv/bin/python" "$PROJECT_DIR/.venv/bin/mjpython" \
  "$PROJECT_DIR/simulate.py" "$@"
