#!/bin/bash
# Source me: picks the scenario config and exports HERO_SIM_CONFIG.
#
#   source _setup.sh                  # DEFAULT_CONFIG_NAME below
#   source _setup.sh my.yaml          # bare filename, found in ../config/ or any folder in it
#   source _setup.sh tether/my.yaml   # relative to ../config/ ...
#   source _setup.sh path/to/my.yaml  # ... unless it's an existing path
#
# Filenames are unique across config/ and its folders (tether/, mrs/); a name
# in two folders is an error, so name the folder. Same rules as
# hero_sim/config.py find_config, which the launch files use.
#
# Order: the argument, then an already-exported HERO_SIM_CONFIG, then
# DEFAULT_CONFIG_NAME. Edit DEFAULT_CONFIG_NAME to change your day-to-day default.
DEFAULT_CONFIG_NAME="test_branch.yaml"

_hero_sim_config_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")/../config" && pwd)

_hero_sim_resolve() {
    case "$1" in
        /*)  echo "$1" ;;
        */*)
            if [ -f "$1" ]; then echo "$1"; else echo "$_hero_sim_config_dir/$1"; fi ;;
        *)
            local matches
            matches=$(find "$_hero_sim_config_dir" -type f -name "$1" | sort)
            if [ "$(printf '%s\n' "$matches" | grep -c .)" -gt 1 ]; then
                echo "[hero_sim] ERROR: $1 is in more than one config folder; name the folder too:" >&2
                printf '%s\n' "$matches" | sed "s|^$_hero_sim_config_dir/|  |" >&2
                return 1
            fi
            echo "${matches:-$_hero_sim_config_dir/$1}" ;;
    esac
}

if [ -n "$1" ]; then
    _hero_sim_name=$1
elif [ -n "$HERO_SIM_CONFIG" ] && [ -f "$HERO_SIM_CONFIG" ]; then
    _hero_sim_name=$HERO_SIM_CONFIG
else
    _hero_sim_name=$DEFAULT_CONFIG_NAME
fi

if ! _hero_sim_cfg=$(_hero_sim_resolve "$_hero_sim_name") || [ ! -f "$_hero_sim_cfg" ]; then
    [ -n "$_hero_sim_cfg" ] && echo "[hero_sim] ERROR: config not found: $_hero_sim_cfg" >&2
    unset -f _hero_sim_resolve
    unset _hero_sim_cfg _hero_sim_name _hero_sim_config_dir
    return 1 2>/dev/null || exit 1
fi

export HERO_SIM_CONFIG=$(realpath "$_hero_sim_cfg")
echo "[hero_sim] HERO_SIM_CONFIG=$HERO_SIM_CONFIG"
unset -f _hero_sim_resolve
unset _hero_sim_cfg _hero_sim_name _hero_sim_config_dir
