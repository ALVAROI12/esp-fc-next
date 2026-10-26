#!/usr/bin/env bash
# Run idf.py inside the pinned ESP-IDF container.
# Usage: tools/idf.sh <project-dir> <idf.py args...>
#   tools/idf.sh firmware/flight -B build-esp32p4 -D IDF_TARGET=esp32p4 build
set -euo pipefail

IDF_IMAGE="${IDF_IMAGE:-espressif/idf:release-v5.5}"
ROOT="$(cd "$(dirname "$0")/.." && pwd)"

if [[ $# -lt 2 ]]; then
  echo "usage: tools/idf.sh <project-dir> <idf.py args...>" >&2
  exit 2
fi

PROJECT="$1"
shift

if [[ ! -f "$ROOT/$PROJECT/CMakeLists.txt" ]]; then
  echo "error: $PROJECT is not an ESP-IDF project" >&2
  exit 2
fi

TTY_FLAGS=()
if [[ -t 0 && -t 1 ]]; then
  TTY_FLAGS=(-it)
fi

# ${arr[@]+...} keeps macOS bash 3.2 happy with an empty array under `set -u`.
exec docker run --rm ${TTY_FLAGS[@]+"${TTY_FLAGS[@]}"} \
  -v "$ROOT":/project \
  -w "/project/$PROJECT" \
  -e HOME=/tmp \
  "$IDF_IMAGE" \
  idf.py "$@"
