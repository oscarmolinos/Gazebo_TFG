#!/bin/bash

usage() {
    echo "  options:"
    echo "      -m: multi agent. Default not set"
    echo "      -t: launch keyboard teleoperation. Default not launch"
    echo "      -v: open rviz. Default not launch"
    echo "      -r: record rosbag. Default not launch"
    echo "      -n: drone namespaces, comma separated. Default get from world description config file"
    echo "      -g: launch using gnome-terminal instead of tmux."
    echo "      -c: path to custom simulation config YAML file."
}

# Initialize variables
swarm="false"
keyboard_teleop="false"
rviz="false"
rosbag="false"
drones_namespace_comma=""
use_gnome="false"
simulation_config=""
config_provided="false"

# Parse command line arguments
while getopts "mtvrn:gc:" opt; do
  case ${opt} in
    m ) swarm="true" ;;
    t ) keyboard_teleop="true" ;;
    v ) rviz="true" ;;
    r ) rosbag="true" ;;
    n ) drones_namespace_comma="${OPTARG}" ;;
    g ) use_gnome="true" ;;
    c ) 
        simulation_config="${OPTARG}"
        config_provided="true"
        ;;
    \? ) echo "Invalid option: -$OPTARG" >&2; usage; exit 1 ;;
    : ) echo "Option -$OPTARG requires an argument" >&2; usage; exit 1 ;;
  esac
done

# Set simulation world description config file (Logic applied only if -c NOT provided)
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