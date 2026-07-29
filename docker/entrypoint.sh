#!/usr/bin/env bash
# Entrypoint for the Isaac Sim service. Builds the scene and starts the
# simulation. Everything runs from the Python API - nothing is done in the GUI.
set -euo pipefail

ISAAC_PY=/isaac-sim/python.sh
SCENE=/workspace/scripts/build_scene.py

echo "[entrypoint] Isaac Sim 5.1 | ROS 2 Jazzy | use_sim_time=true"
echo "[entrypoint] HEADLESS=${HEADLESS:-1}"

exec "$ISAAC_PY" "$SCENE" --headless "${HEADLESS:-1}"
