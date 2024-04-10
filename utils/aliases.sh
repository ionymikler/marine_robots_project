#!/bin/bash
base_dir="/home/iony/DTU/s24/autonomous_marine_robots/final_project"
alias start_container='cd ${base_dir}&&docker compose up -d'
alias stop_container='cd ${base_dir}&&docker compose down'
alias enter_container='docker exec -it final_project-dev-1 bash'