#!/bin/bash

# Configurar el resource path para mundos personalizados
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:/home/oscar/Aerostack2/project_gazebo/worlds

usage() {
    echo "  options:"
    echo "      -m: multi agent. Default not set"
    echo "      -n: select drones namespace to launch, values are comma separated."
    echo "      -s: if set, the simulation will not be launched."
    echo "      -g: launch using gnome-terminal instead of tmux."
    echo "      -c: path to custom simulation config YAML file."
}

# Initialize variables with default values
swarm="false"
drones_namespace_comma=""
launch_simulation="true"
use_gnome="false"
simulation_config="" # Variable para el mapa personalizado
config_provided="false" # Bandera para saber si el usuario eligió mapa

# Arg parser
while getopts "mn:sgc:" opt; do
  case ${opt} in
    m ) swarm="true" ;;
    n ) drones_namespace_comma="${OPTARG}" ;;
    s ) launch_simulation="false" ;;
    g ) use_gnome="true" ;;
    c ) 
        simulation_config="${OPTARG}"
        config_provided="true"
        ;;
    \? ) echo "Invalid option: -$OPTARG" >&2; usage; exit 1 ;;
    : ) echo "Option -$OPTARG requires an argument" >&2; usage; exit 1 ;;
  esac
done

# Set simulation world description config file
# Si el usuario NO pasó -c, aplicamos la lógica por defecto
if [[ ${config_provided} == "false" ]]; then
    if [[ ${swarm} == "true" ]]; then
      simulation_config="config/world_swarm.yaml"
    else
      simulation_config="config/world.yaml"
    fi
fi

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