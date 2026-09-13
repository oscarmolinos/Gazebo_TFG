#!/bin/bash

# Configurar el resource path para mundos personalizados
# Se deriva de la ubicacion de este script para que funcione en cualquier pc
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:${PROJECT_DIR}/worlds

usage() {
    echo "  options:"
    echo "      -n: select drones namespace to launch, values are comma separated."
    echo "      -s: if set, the simulation will not be launched."
    echo "      -g: launch using gnome-terminal instead of tmux."
    echo "      -c: path to simulation config YAML file. Default config/world_problem1.yaml"
}

# Initialize variables with default values
drones_namespace_comma=""
launch_simulation="true"
use_gnome="false"
simulation_config="config/world_problem1.yaml" # Mapa por defecto si no se pasa -c

# Arg parser
while getopts "n:sgc:" opt; do
  case ${opt} in
    n ) drones_namespace_comma="${OPTARG}" ;;
    s ) launch_simulation="false" ;;
    g ) use_gnome="true" ;;
    c ) simulation_config="${OPTARG}" ;;
    \? ) echo "Invalid option: -$OPTARG" >&2; usage; exit 1 ;;
    : ) echo "Option -$OPTARG requires an argument" >&2; usage; exit 1 ;;
  esac
done

# If no drone namespaces are provided, get them from the world description config file
if [ -z "$drones_namespace_comma" ]; then
  drones_namespace_comma=$(python3 utils/get_drones.py -p ${simulation_config} --sep ',')
fi
IFS=',' read -r -a drone_namespaces <<< "$drones_namespace_comma"

# Select between tmux and gnome-terminal
tmuxinator_mode="start"
tmuxinator_end="wait"
tmp_file="/tmp/as2_project_launch_${drone_namespaces[@]}.txt"
if [[ ${use_gnome} == "true" ]]; then
  tmuxinator_mode="debug"
  tmuxinator_end="> ${tmp_file} && python3 utils/tmuxinator_to_genome.py -p ${tmp_file} && wait"
fi

# Launch aerostack2 for each drone namespace
for namespace in ${drone_namespaces[@]}; do
  base_launch="false"
  if [[ ${namespace} == ${drone_namespaces[0]} && ${launch_simulation} == "true" ]]; then
    base_launch="true"
  fi
  eval "tmuxinator ${tmuxinator_mode} -n ${namespace} -p tmuxinator/aerostack2.yaml \
    drone_namespace=${namespace} \
    simulation_config_file=${simulation_config} \
    base_launch=${base_launch} \
    ${tmuxinator_end}"

  sleep 1.0 # Wait for tmuxinator to finish
done

# Attach to tmux session
if [[ ${use_gnome} == "false" ]]; then
  tmux attach-session -t ${drone_namespaces[0]}
# If tmp_file exists, remove it
elif [[ -f ${tmp_file} ]]; then
  rm ${tmp_file}
fi