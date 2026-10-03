#!/bin/bash
# Source me: picks the scenario config and exports HERO_SIM_CONFIG.
#
#   source _setup.sh                  # DEFAULT_CONFIG_NAME below
#   source _setup.sh my.yaml          # bare filename -> ../config/my.yaml
#   source _setup.sh path/to/my.yaml  # any path
#
# Order: the argument, then an already-exported HERO_SIM_CONFIG, then
# DEFAULT_CONFIG_NAME. Edit DEFAULT_CONFIG_NAME to change your day-to-day default.
DEFAULT_CONFIG_NAME="test_deploy_2.yaml"

_hero_sim_config_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")/../config" && pwd)

_hero_sim_resolve() {
    case "$1" in
        */*) echo "$1" ;;
        *)   echo "$_hero_sim_config_dir/$1" ;;
    esac
}

if [ -n "$1" ]; then
    _hero_sim_cfg=$(_hero_sim_resolve "$1")
elif [ -n "$HERO_SIM_CONFIG" ] && [ -f "$HERO_SIM_CONFIG" ]; then
    _hero_sim_cfg=$HERO_SIM_CONFIG
else
    _hero_sim_cfg=$(_hero_sim_resolve "$DEFAULT_CONFIG_NAME")
fi

if [ ! -f "$_hero_sim_cfg" ]; then
    echo "[hero_sim] ERROR: config not found: $_hero_sim_cfg" >&2
    unset -f _hero_sim_resolve
    return 1 2>/dev/null || exit 1
fi

export HERO_SIM_CONFIG=$(realpath "$_hero_sim_cfg")
echo "[hero_sim] HERO_SIM_CONFIG=$HERO_SIM_CONFIG"
unset -f _hero_sim_resolve
unset _hero_sim_cfg _hero_sim_config_dir
