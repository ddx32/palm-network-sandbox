#!/bin/sh
# Build, flash and open serial monitor (Ctrl+] to exit). PORT=/dev/ttyUSB0 by default.
PORT=${PORT:-/dev/ttyUSB0}
DOCKER_ARGS="-it --device $PORT --group-add $(stat -c %g "$PORT")" \
  exec "$(dirname "$0")/build.sh" -p "$PORT" flash monitor
