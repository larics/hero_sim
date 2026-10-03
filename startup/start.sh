#!/bin/bash
# Start the hero_sim tmux session (layout in session.yml).
#
#   bash start.sh                  # DEFAULT_CONFIG_NAME from _setup.sh
#   bash start.sh my.yaml          # bare filename -> config/my.yaml
#   bash start.sh path/to/my.yaml  # any path (relative to where you run this)
#
# Stop it with: tmux kill-session -t hero_sim
set -e

CONFIG_ARG=$1
# Resolve a relative path before changing directory.
case "$CONFIG_ARG" in
    */*) CONFIG_ARG=$(realpath "$CONFIG_ARG") ;;
esac

cd "$(dirname "$(readlink -f "$0")")"
source ./_setup.sh "$CONFIG_ARG"

tmuxinator start -p session.yml config="$HERO_SIM_CONFIG"
