#!/bin/sh
# Build firmware in the ESP-IDF Docker image. Extra args go to idf.py (default: build).
set -e
cd "$(dirname "$0")"
[ -f main/secrets.h ] || { echo "Copy main/secrets.h.example to main/secrets.h and fill in Wi-Fi credentials"; exit 1; }
[ $# -eq 0 ] && set -- build
exec docker run --rm ${DOCKER_ARGS} -v "$PWD":/project -w /project \
  -u "$(id -u):$(id -g)" -e HOME=/tmp \
  espressif/idf:v5.4 idf.py "$@"
