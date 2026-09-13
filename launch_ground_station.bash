#!/bin/bash

usage() {
    echo "  options:"
    echo "      -t: launch keyboard teleoperation. Default not launch"
    echo "      -v: open rviz. Default not launch"
    echo "      -r: record rosbag. Default not launch"
    echo "      -n: drone namespaces, comma separated. Default get from world description config file"
    echo "      -g: launch using gnome-terminal instead of tmux."
    echo "      -c: path to simulation config YAML file. Default config/world_problem1.yaml"
}

# Initialize variables
keyboard_teleop="false"
rviz="false"
rosbag="false"
drones_namespace_comma=""
use_gnome="false"
simulation_config="config/world_problem1.yaml" # Mapa por defecto si no se pasa -c

# Parse command line arguments
while getopts "tvrn:gc:" opt; do
  case ${opt} in
    t ) keyboard_teleop="true" ;;
    v ) rviz="true" ;;
    r ) rosbag="true" ;;
    n ) drones_namespace_comma="${OPTARG}" ;;
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

# Select between tmux and gnome-terminal
tmuxinator_mode="start"
tmuxinator_end="wait"
tmp_file="/tmp/as2_project_ground_station.txt"
if [[ ${use_gnome} == "true" ]]; then
  tmuxinator_mode="debug"
  tmuxinator_end="> ${tmp_file} && python3 utils/tmuxinator_to_genome.py -p ${tmp_file} && wait"
fi

# Launch aerostack2 ground station
eval "tmuxinator ${tmuxinator_mode} -n ground_station -p tmuxinator/ground_station.yaml \
  drone_namespace=${drones_namespace_comma} \
  keyboard_teleop=${keyboard_teleop} \
  rviz=${rviz} \
  rosbag=${rosbag} \
  simulation_config_file=${simulation_config} \
  ${tmuxinator_end}"

# Attach to tmux session
if [[ ${use_gnome} == "false" ]]; then
  tmux attach-session -t ground_station
elif [[ -f ${tmp_file} ]]; then
  rm ${tmp_file}
fi