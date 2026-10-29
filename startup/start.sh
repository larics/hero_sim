#!/bin/bash
# Start the hero_sim tmux session (layout in session.yml).
#
#   bash start.sh                  # DEFAULT_CONFIG_NAME from _setup.sh
#   bash start.sh my.yaml          # bare filename, found in config/ or any folder in it
#   bash start.sh tether/my.yaml   # relative to config/ ...
#   bash start.sh path/to/my.yaml  # ... unless it's an existing path (relative to where you run this)
#
# Stop it with: tmux kill-session -t hero_sim
set -e

CONFIG_ARG=$1
# Resolve an existing relative path before changing directory; anything else
# (tether/my.yaml) is looked up under config/ by _setup.sh.
case "$CONFIG_ARG" in
    */*) [ -f "$CONFIG_ARG" ] && CONFIG_ARG=$(realpath "$CONFIG_ARG") ;;
esac

cd "$(dirname "$(readlink -f "$0")")"
source ./_setup.sh "$CONFIG_ARG"

tmuxinator start -p session.yml config="$HERO_SIM_CONFIG"
