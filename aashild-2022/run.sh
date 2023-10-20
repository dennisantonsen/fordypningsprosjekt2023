#!/bin/bash

#######################################################################################
### PURPOSE

# This script combines 
# 1. Configuration of access to host x11 for gui running in docker
# 2. Run docker compose with options
# 3. Clean up configuration after exiting docker compose


#######################################################################################
### HOW TO USE

# ./run.sh --help

post_usage()
{
    printf '%s\n'  "Usage   : source run.sh [ -h | --help ] [ -b | --build ]"
    printf '%s\n'  "Example : ./run.sh     # Build if no cache is available, then run it. Fastest option."
    printf '%s\n'  "Example : ./run.sh -b  # Force rebuild then run. This is the slow option."
    exit 2
}


#######################################################################################
### INPUTS

compose_up_args=""
arg_build=""

# No arguments?
# if [ "$#" -eq 0 ]; then
#     post_usage
# fi


while :; do
  case "$1" in
    -h | --help)
        post_usage
        ;;

    -b | --build)
        arg_build="--build"
        shift
        ;;   

    --)
        # -- means the end of the arguments; drop this, and break out of the while loop
        shift
        break
        ;;

    -?*)
        printf 'WARN: Unknown option (ignored): %s\n' "$1" >&2
        ;;
    *)
        # Default case: No more options, so break out of the loop.
        break
  esac

  shift
done

# Combine args into a shell array.
# An array will expand as a single string when we later expand it after a command.
# If we concatenated the args into a single string like this, all_args="arg1 arg2 arg3", then the shell expansion will treat each space as a signal that the next word is a new command.
compose_up_args=(${arg_build})



#######################################################################################
### MAIN


xhost +local:root

docker-compose up "${compose_up_args[@]}"

xhost -local:root

