#!/bin/bash

# Lee los parámetros
cont_folder=$1
rate=$2
start=$3
duration=$4

# Haz lo que necesites con los parámetros
echo "Containing folder: $cont_folder"
echo "Execution rate: $rate"
echo "Execution start at $start s"
echo "Execution duration: $duration"

rosbag play --pause --rate $rate --start $start --duration $duration $cont_folder/*/m*.bag $cont_folder/*/p*.bag